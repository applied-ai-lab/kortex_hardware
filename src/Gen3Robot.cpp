#include "Gen3Robot.h"

#include <cmath> // std::abs
#include <thread>
#include <typeinfo>

#if __has_include(<pthread.h>) && __has_include(<sched.h>)
  #include <pthread.h>
  #include <sched.h>
  #define KORTEX_HARDWARE__THREAD_PRIORITY
#endif

#include <unistd.h>

#include "kortex_hardware/ModeService.h"
#include "pinocchio/parsers/urdf.hpp"
#include <std_msgs/UInt64MultiArray.h>

using namespace std;

int64_t GetTickUs()
{
#if defined(_MSC_VER)
  LARGE_INTEGER start, frequency;
  LARGE_INTEGER start, frequency;

  QueryPerformanceFrequency(&frequency);
  QueryPerformanceCounter(&start);

  return (start.QuadPart * 1000000) / frequency.QuadPart;
#else
  struct timespec start;
  clock_gettime(CLOCK_MONOTONIC, &start);

  return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}

void Gen3Robot::addGravityCompensation(
    pinocchio::Model& model,
    pinocchio::Data& data,
    std::vector<double>& config,
    std::vector<double>& command)
{
  q_.setZero();
  // convert ROS joint config to pinocchio config
  for (int i = 0; i < model.nv; i++)
  {
    int jidx = model.getJointId(model.names[i + 1]);
    int qidx = model.idx_qs[jidx];
    // nqs[i] is 2 for continuous joints in pinocchio
    if (model.nqs[jidx] == 2)
    {
      q_[qidx] = std::cos(config[i]);
      q_[qidx + 1] = std::sin(config[i]);
    }
    else
    {
      q_[qidx] = config[i];
    }
  }

  // Defensive: wrap the Pinocchio call AND scan the output for
  // non-finite entries.  Two failure modes are folded together:
  //
  //   1. Throw — `computeGeneralizedGravity` raises (std::exception,
  //      Eigen runtime_error, etc.) when `q_` is dimensionally wrong
  //      or contains non-finite values that trip an internal assert.
  //      Catch, log throttled, zero gravity for this cycle.
  //
  //   2. Non-finite output — `cos(NaN)` / `sin(NaN)` from a bad
  //      `config[i]` propagates through RNEA without throwing.
  //      Scan the result; non-finite entries are zeroed individually
  //      so the rest of the gravity vector still applies.
  //
  // Either failure increments `m_nan_gravity_count` so the
  // diagnostics topic surfaces it.  Combined with the boundary
  // guard at `set_torque_joint`, no non-finite value can reach the
  // wire regardless of upstream state.
  try
  {
    gravity_ = pinocchio::computeGeneralizedGravity(model, data, q_);
  }
  catch (const std::exception& ex)
  {
    m_nan_gravity_count.fetch_add(1, std::memory_order_relaxed);
    ROS_ERROR_THROTTLE(
        1.0,
        "[kortex_hardware] Pinocchio computeGeneralizedGravity threw: %s "
        "— zeroing gravity for this cycle",
        ex.what());
    gravity_.setZero();
  }

  bool any_nan = false;
  for (int i = 0; i < model.nv; i++)
  {
    if (!std::isfinite(gravity_[i]))
    {
      gravity_[i] = 0.0;
      any_nan = true;
    }
  }
  if (any_nan)
  {
    m_nan_gravity_count.fetch_add(1, std::memory_order_relaxed);
    ROS_WARN_THROTTLE(
        1.0,
        "[kortex_hardware] gravity comp produced non-finite output — "
        "zeroed affected joints");
  }

  // add gravity compensation torque to base command
  for (int i = 0; i < model.nv; i++)
  {
    command[i] = command[i] + gravity_[i];
  }
}

Gen3Robot::Gen3Robot(ros::NodeHandle nh)
{
  ROS_INFO("Retreiving ROS parameters");
  ros::param::get("~username", m_username);
  ros::param::get("~password", m_password);
  ros::param::get("~ip_address", m_ip_address);
  ros::param::get(
      "~api_session_inactivity_timeout_ms",
      m_api_session_inactivity_timeout_ms);
  ros::param::get(
      "~api_connection_inactivity_timeout_ms",
      m_api_connection_inactivity_timeout_ms);
  ros::param::get("~dof", num_arm_dof);
  ros::param::get("~current_control", current_control);
  ros::param::get("~use_gripper", mUseGripper);
  ros::param::get("~use_admittance", mUseAdmittance);
  ros::param::get("~urdf_file", mURDFFile);
  ros::param::get("~prefix", mPrefix);

  ROS_INFO("Starting to initialize kortex_hardware");
  num_full_dof = num_arm_dof + num_finger_dof;
  // From kortex_driver/src/non-generated/kortex_arm_driver.cpp
  m_tcp_transport = new k_api::TransportClientTcp();
  m_udp_transport = new k_api::TransportClientUdp();
  m_tcp_transport->connect(m_ip_address, TCP_PORT);
  m_udp_transport->connect(m_ip_address, UDP_PORT);
  m_tcp_router
      = new k_api::RouterClient(m_tcp_transport, [](k_api::KError err) {
          ROS_ERROR(
              "Kortex API error was encountered with the TCP router: %s",
              err.toString().c_str());
        });
  m_udp_router
      = new k_api::RouterClient(m_udp_transport, [](k_api::KError err) {
          ROS_ERROR(
              "Kortex API error was encountered with the UDP router: %s",
              err.toString().c_str());
        });
  m_tcp_session_manager = new k_api::SessionManager(m_tcp_router);
  m_udp_session_manager = new k_api::SessionManager(m_udp_router);
  mBase = new k_api::Base::BaseClient(m_tcp_router);
  mBaseCyclic = new k_api::BaseCyclic::BaseCyclicClient(m_udp_router);

  mActuatorConfig
      = new k_api::ActuatorConfig::ActuatorConfigClient(m_tcp_router);
  mServoingMode = k_api::Base::ServoingModeInformation();
  mControlModeMessage = k_api::ActuatorConfig::ControlModeInformation();

  int i;
  cmd_pos.resize(num_full_dof);
  cmd_vel.resize(num_full_dof);
  cmd_eff.resize(num_full_dof);
  zero_velocity_command.resize(num_full_dof, 0.0);
  pos.resize(num_full_dof);
  vel.resize(num_full_dof);
  eff.resize(num_full_dof);
  pos_offsets.resize(num_full_dof);
  soft_limits.resize(num_full_dof);
  // cmd_cart_vel.resize(6); // SE(3)

  for (std::size_t i = 0; i < pos_offsets.size(); ++i)
    pos_offsets[i] = 0.0;

  for (std::size_t i = 0; i < cmd_vel.size(); ++i)
    cmd_vel[i] = 0.0;

  // connect and register the joint state interface.
  // this gives joint states (pos, vel, eff) back as an output.
  for (std::size_t i = 0; i < num_arm_dof; ++i)
  {
    std::string jnt_name = mPrefix + "joint_" + std::to_string(i + 1);

    // connect and register the joint state interface.
    // this gives joint states (pos, vel, eff) back as an output.
    hardware_interface::JointStateHandle state_handle(
        jnt_name, &pos[i], &vel[i], &eff[i]);
    jnt_state_interface.registerHandle(state_handle);

    // connect and register the joint position interface
    // this takes joint velocities in as a command.
    hardware_interface::JointHandle vel_handle(
        jnt_state_interface.getHandle(jnt_name), &cmd_vel[i]);
    jnt_vel_interface.registerHandle(vel_handle);

    // connect and register the joint position interface
    // this takes joint positions in as a command.
    hardware_interface::JointHandle pos_handle(
        jnt_state_interface.getHandle(jnt_name), &cmd_pos[i]);
    jnt_pos_interface.registerHandle(pos_handle);

    // connect and register the joint position interface
    // this takes joint effort in as a command.
    hardware_interface::JointHandle eff_handle(
        jnt_state_interface.getHandle(jnt_name), &cmd_eff[i]);
    jnt_eff_interface.registerHandle(eff_handle);
  }
  mode_service = nh.advertiseService(
      "set_control_mode", &Gen3Robot::setControlMode, this);

  // connect and register the joint state interface for gripper
  hardware_interface::JointStateHandle grp_state_handle(
      mPrefix +"finger_joint",
      &pos[num_full_dof - 1],
      &vel[num_full_dof - 1],
      &eff[num_full_dof - 1]);
  jnt_state_interface.registerHandle(grp_state_handle);

  hardware_interface::JointHandle grp_vel_handle(
      jnt_state_interface.getHandle(mPrefix + "finger_joint"),
      &cmd_vel[num_full_dof - 1]);
  jnt_vel_interface.registerHandle(grp_vel_handle);

  hardware_interface::JointHandle grp_pos_handle(
      jnt_state_interface.getHandle(mPrefix + "finger_joint"),
      &cmd_pos[num_full_dof - 1]);
  jnt_pos_interface.registerHandle(grp_pos_handle);

  hardware_interface::JointHandle grp_eff_handle(
      jnt_state_interface.getHandle(mPrefix + "finger_joint"),
      &cmd_eff[num_full_dof - 1]);
  jnt_eff_interface.registerHandle(grp_eff_handle);

  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_vel_interface);
  registerInterface(&jnt_pos_interface);

  ROS_INFO("Register Effort Interface...");
  registerInterface(&jnt_eff_interface);

  // connect and register the joint mode interface
  // this is needed to determine if velocity or position control is needed.
  hardware_interface::JointModeHandle arm_mode_handle("joint_mode", &arm_mode);
  hardware_interface::JointModeHandle gripper_mode_handle(
      "gripper_mode", &gripper_mode);
  jm_interface.registerHandle(arm_mode_handle);
  jm_interface.registerHandle(gripper_mode_handle);

  registerInterface(&jm_interface);

  // Create the sessions so we can start using the robot
  auto createSessionInfo = Kinova::Api::Session::CreateSessionInfo();
  createSessionInfo.set_username(m_username);
  createSessionInfo.set_password(m_password);
  createSessionInfo.set_session_inactivity_timeout(
      m_api_session_inactivity_timeout_ms);
  createSessionInfo.set_connection_inactivity_timeout(
      m_api_connection_inactivity_timeout_ms);
  try
  {
    m_tcp_session_manager->CreateSession(createSessionInfo);
    ROS_INFO("Session created successfully for TCP services");

    m_udp_session_manager->CreateSession(createSessionInfo);
    ROS_INFO("Session created successfully for UDP services");
  }
  catch (std::runtime_error& ex_runtime)
  {
    std::string error_string
        = "The node could not connect to the arm. Did you specify the right IP "
          "address and is the arm powered on?";
    ROS_ERROR("%s", error_string.c_str());
    throw ex_runtime;
  }

  // Clearing faults
  try
  {
    mBase->ClearFaults();
  }
  catch (...)
  {
    std::cout << "Unable to clear robot faults" << std::endl;
    return;
  }

  finger = gripper_command.mutable_gripper()->add_finger();
  finger->set_finger_identifier(1);

  arm_mode = hardware_interface::JointCommandModes::MODE_EFFORT;
  gripper_mode = hardware_interface::JointCommandModes::MODE_POSITION;
  last_arm_mode = hardware_interface::JointCommandModes::BEGIN;

  // Initialize the low pass filter.  Counter pointers feed the
  // diagnostics pipeline — if either LPF's sticky-NaN trap-break
  // fires, the corresponding counter increments and the failure
  // becomes a transient instead of "limp until restart".
  in_lpf  = new LowPassFilter(1000, 30,  num_full_dof, &m_nan_lpf_in_count);
  out_lpf = new LowPassFilter(1000, 100, num_full_dof, &m_nan_lpf_out_count);

  // Initialize current control parameters
  if (num_arm_dof == 6)
  {
    input_current_limit = {10.0, 10.0, 10.0, 6.0, 6.0, 6.0};
    gear_ratio = {11.0, 11.0, 11.0, 7.6, 7.6, 7.6};
  }
  else
  {
    // arm with 7 dof
    input_current_limit = {10.0, 10.0, 10.0, 10.0, 6.0, 6.0, 6.0};
    gear_ratio = {11.0, 11.0, 11.0, 11.0, 7.6, 7.6, 7.6};
  }

  // ---- Pinocchio initialization (gravity-compensation model) ----
  // buildModel throws std::invalid_argument / std::runtime_error if
  // the URDF path is empty, unreadable, or malformed.  Bare throw
  // here would propagate out of the constructor, out of main(), and
  // terminate the process via std::terminate — with a backtrace that
  // rarely names Pinocchio.  Catch it, log a useful diagnostic
  // identifying the bad path and parser message, then rethrow so the
  // top-level main() catch can shut down cleanly.
  try
  {
    pinocchio::urdf::buildModel(mURDFFile, model);
  }
  catch (const std::exception& ex)
  {
    ROS_FATAL(
        "[kortex_hardware] Pinocchio failed to parse URDF at '%s': %s. "
        "Check the ~urdf_file rosparam (currently '%s'); the path must "
        "exist, be readable by the container user, and contain a valid "
        "URDF/XACRO-output document.",
        mURDFFile.c_str(), ex.what(), mURDFFile.c_str());
    throw;
  }
  data = pinocchio::Data(model);
  q_ = pinocchio::neutral(model);
  gravity_ = pinocchio::computeGeneralizedGravity(model, data, q_);

#ifdef KORTEX_HARDWARE__THREAD_PRIORITY
  std::ifstream realtime_file {"/sys/kernel/realtime", std::ios::in};
  bool has_realtime {false};
  if (realtime_file.is_open()) {
    realtime_file >> has_realtime;
  }

  ::pthread_t const current_thread {pthread_self()};
  int cpu_core {-1};
  if (ros::param::get("~cpu_core", cpu_core)) {
    ::cpu_set_t cpuset {};
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core, &cpuset);
    if (::pthread_setaffinity_np(current_thread, sizeof(::cpu_set_t), &cpuset) == 0) {
      std::cout << "Set thread affinity to cpu '" << cpu_core << "'!" << std::endl;
    } else {
      std::cerr << "Failed to set thread affinity to cpu '" << cpu_core << "'!" << std::endl;
    }
  } else {
    std::cerr << "Failed to get cpu_core for setting thread affinity, not pinning process!" << std::endl;
  }
  int policy {};
  struct ::sched_param param {};
  ::pthread_getschedparam(current_thread, &policy, &param);
  if (has_realtime) {
    policy = SCHED_FIFO;
    std::cout << "Real-time system detected: Setting policy to 'SCHED_FIFO'..." << std::endl;
  }
  int const max_thread_priority {::sched_get_priority_max(policy)};
  if (max_thread_priority != -1) {
    param.sched_priority = max_thread_priority;
    if (::pthread_setschedparam(current_thread, policy, &param) == 0) {
      std::cout << "Set thread priority '" << param.sched_priority << "' and policy '" <<
       policy << "' to async thread!" << std::endl;
    } else {
      std::cerr << "Failed to set thread priority '" << param.sched_priority << "' and policy '" <<
       policy << "' to async thread!" << std::endl;
    }
  } else {
    std::cerr << "Could not set thread priority to async thread: Failed to get max priority!" << std::endl;
  }
#endif  // KORTEX_HARDWARE__THREAD_PRIORITY

  // ---- Soft-stop bridge subscribers --------------------------------
  // Relative names — resolve to /<arm_ns>/in/* via launch-file
  // namespacing.  oxf20_soft_stop/launch/relays.launch bridges the
  // global /in/* topics onto these.
  m_estop_sub        = nh.subscribe("in/emergency_stop", 1,
                                    &Gen3Robot::estopCallback, this);
  m_clear_faults_sub = nh.subscribe("in/clear_faults", 1,
                                    &Gen3Robot::clearFaultsCallback, this);
  m_stop_sub         = nh.subscribe("in/stop", 1,
                                    &Gen3Robot::stopCallback, this);
  ROS_INFO("[kortex_hardware] soft-stop subscribers attached on "
           "in/{emergency_stop,clear_faults,stop}");

  // ---- Layer 1 NaN diagnostics --------------------------------------
  // 1 Hz publisher of the counter array.  Subscribers can drive
  // rqt_plot / rosbag without per-cycle overhead in the control loop.
  // Counter atomics are read-only here; no lock needed.
  m_diag_pub = nh.advertise<std_msgs::UInt64MultiArray>(
      "diagnostics/nan_counts", 10);
  m_diag_timer = nh.createTimer(
      ros::Duration(1.0), &Gen3Robot::diagnosticsTimerCallback, this);
  ROS_INFO("[kortex_hardware] NaN diagnostics publisher attached on "
           "~diagnostics/nan_counts (1 Hz)");
}

void Gen3Robot::diagnosticsTimerCallback(const ros::TimerEvent&)
{
  // Snapshot the atomics.  Layout: [cmd, lpf_in, lpf_out, gravity, last_joint].
  // Matches the order used in docs/diagnosis_report.md.
  std_msgs::UInt64MultiArray msg;
  msg.data.resize(5);
  msg.data[0] = m_nan_cmd_count.load(std::memory_order_relaxed);
  msg.data[1] = m_nan_lpf_in_count.load(std::memory_order_relaxed);
  msg.data[2] = m_nan_lpf_out_count.load(std::memory_order_relaxed);
  msg.data[3] = m_nan_gravity_count.load(std::memory_order_relaxed);
  // last_joint is signed; cast to unsigned for transport — -1 (no
  // hit yet) becomes UINT64_MAX on the wire, easy to spot.
  int last = m_nan_last_joint.load(std::memory_order_relaxed);
  msg.data[4] = static_cast<std::uint64_t>(last);
  m_diag_pub.publish(msg);
}

void Gen3Robot::estopCallback(const std_msgs::Empty::ConstPtr&)
{
  // Latch BEFORE issuing the API call so the next write() in the
  // main loop is skipped even if the RPC is briefly delayed.
  m_estop_latched.store(true, std::memory_order_release);
  ROS_WARN("[kortex_hardware] EMERGENCY STOP received");
  try
  {
    // 0 = "stop all actuators".  The {false, 0, 100} options match
    // the destructor's call (Gen3Robot.cpp:325) — no expected-reply,
    // ID 0, 100 ms timeout.
    mBase->ApplyEmergencyStop(0, {false, 0, 100});
  }
  catch (k_api::KDetailedException& ex)
  {
    ROS_ERROR("[kortex_hardware] ApplyEmergencyStop failed: %s", ex.what());
    // Keep the latch on — better to be stopped-with-error than to
    // resume writing cyclic frames to an arm we can't confirm faulted.
  }
}

void Gen3Robot::clearFaultsCallback(const std_msgs::Empty::ConstPtr&)
{
  ROS_INFO("[kortex_hardware] CLEAR FAULTS received");
  try
  {
    mBase->ClearFaults();

    // ApplyEmergencyStop kicks the arm out of whatever servoing mode
    // it was in.  Restore based on which mode the main loop was using:
    //
    //   * If LOW_LEVEL_SERVOING (effort/torque on the compliant stack)
    //     was active, re-establish it AND re-apply each actuator's
    //     TORQUE/CURRENT control mode.  Without the per-actuator step
    //     the firmware silently demotes actuators to POSITION on fault,
    //     so the next BaseCyclic frame is accepted but produces no
    //     motion — the most confusing failure mode.
    //   * Otherwise (SINGLE_LEVEL position/velocity), just restore the
    //     single-level servoing mode and let the next one-shot RPC
    //     re-engage.
    //
    // The per-actuator step is also gated on mActuatorCount > 0 because
    // mActuatorCount has no in-class initializer and is only assigned
    // inside switchToEffortMode().  If clearFaultsCallback runs before
    // any effort mode was ever entered (e.g. spurious clear-faults at
    // startup) the count is undefined; the guard keeps us out of UB.
    if (mLowLevelServoing && mActuatorCount > 0)
    {
      mServoingMode.set_servoing_mode(
          k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
      mBase->SetServoingMode(mServoingMode);

      // Restore per-actuator control mode.  current_control selects
      // between TORQUE (default) and CURRENT, matching the convention
      // in switchToEffortMode().
      if (current_control)
        mControlModeMessage.set_control_mode(
            k_api::ActuatorConfig::ControlMode::CURRENT);
      else
        mControlModeMessage.set_control_mode(
            k_api::ActuatorConfig::ControlMode::TORQUE);
      for (unsigned int idx = 1; idx <= mActuatorCount; idx++)
        mActuatorConfig->SetControlMode(mControlModeMessage, idx);

      ROS_INFO("[kortex_hardware] restored LOW_LEVEL_SERVOING + %s "
               "control on %u actuators",
               current_control ? "CURRENT" : "TORQUE", mActuatorCount);
    }
    else
    {
      mServoingMode.set_servoing_mode(
          k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
      mBase->SetServoingMode(mServoingMode);
      ROS_INFO("[kortex_hardware] restored SINGLE_LEVEL_SERVOING");
    }
  }
  catch (k_api::KDetailedException& ex)
  {
    ROS_ERROR("[kortex_hardware] ClearFaults recovery failed: %s",
              ex.what());
    // Leave the latch on so we don't blindly resume cyclic writes
    // against a still-faulted or partially-recovered arm.
    return;
  }
  // Only release the latch once the full recovery sequence succeeds.
  m_estop_latched.store(false, std::memory_order_release);
  ROS_INFO("[kortex_hardware] faults cleared, control loop re-enabled");
}

void Gen3Robot::stopCallback(const std_msgs::Empty::ConstPtr&)
{
  // Soft stop — non-faulting decel.  Does NOT latch, because the
  // arm is not in a fault state after Base::Stop and the firmware
  // accepts cyclic commands again immediately.  If the operator
  // wanted a hard stop they should hit the e-stop topic.
  ROS_INFO("[kortex_hardware] SOFT STOP received");
  try
  {
    mBase->Stop();
  }
  catch (k_api::KDetailedException& ex)
  {
    ROS_ERROR("[kortex_hardware] Base::Stop failed: %s", ex.what());
  }
}

Gen3Robot::~Gen3Robot()
{
  try
  {
    mBase->ApplyEmergencyStop(0, {false, 0, 100});
    // clear faults
    mBase->ClearFaults();
  }
  catch (k_api::KDetailedException& ex)
  {
    std::cout << "Kortex exception: " << ex.what() << std::endl;

    std::cout << "Error sub-code: "
              << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(
                     (ex.getErrorInfo().getError().error_sub_code())))
              << std::endl;
  }

  mControlModeMessage.set_control_mode(
      k_api::ActuatorConfig::ControlMode::POSITION);
  for (int idx = 0; idx < mActuatorCount; idx++)
    mActuatorConfig->SetControlMode(mControlModeMessage, idx + 1);

  // Set the servoing mode back to Single Level
  mServoingMode.set_servoing_mode(
      k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  mBase->SetServoingMode(mServoingMode);

  // Wait for a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  m_tcp_session_manager->CloseSession();
  m_udp_session_manager->CloseSession();
  m_tcp_router->SetActivationStatus(false);
  m_udp_router->SetActivationStatus(false);
  m_tcp_transport->disconnect();
  m_udp_transport->disconnect();

  delete mBase;
  delete mBaseCyclic;

  delete m_tcp_session_manager;
  delete m_udp_session_manager;
  delete m_tcp_router;
  delete m_udp_router;
  delete m_tcp_transport;
  delete m_udp_transport;

  ros::Duration(0.10).sleep();
}

bool Gen3Robot::setControlMode(
    kortex_hardware::ModeService::Request& req,
    kortex_hardware::ModeService::Response& resp)
{
  if (req.mode == "position")
  {
    arm_mode = hardware_interface::JointCommandModes::MODE_POSITION;
    resp.success = true;
  }
  else if (req.mode == "velocity")
  {
    arm_mode = hardware_interface::JointCommandModes::MODE_VELOCITY;
    resp.success = true;
  }
  else if (req.mode == "effort")
  {
    arm_mode = hardware_interface::JointCommandModes::MODE_EFFORT;
    resp.success = true;
  }
  else if (req.mode == "stop")
  {
    arm_mode = hardware_interface::JointCommandModes::EMERGENCY_STOP;
    resp.success = true;
  }
  else
  {
    ROS_ERROR("Invalid control mode");
    resp.success = false;
  }
}

void Gen3Robot::initializeSoftLimits()
{
  ROS_INFO("Initializing soft limits for Gen3");
  // TODO: Instead convert to a init function that allows us to set soft limits
  // etc.
}

ros::Time Gen3Robot::get_time(void)
{
  return ros::Time::now();
}

ros::Duration Gen3Robot::get_period(void)
{
  return ros::Duration(0.01);
}

inline double Gen3Robot::degreesToRadians(double degrees)
{
  return (M_PI / 180.0) * degrees;
}

inline double Gen3Robot::radiansToDegrees(double radians)
{
  return (180.0 / M_PI) * radians;
}

void Gen3Robot::sendPositionCommand(const std::vector<double>& command)
{
  mLastFeedback = mBaseCyclic->RefreshFeedback();
  if (command == prev_cmd_pos)
    return;

  try
  {
    mBase->StopAction();
  }
  catch (k_api::KDetailedException& ex)
  {
    std::cout << "Kortex exception: " << ex.what() << std::endl;

    std::cout << "Error sub-code: "
              << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(
                     (ex.getErrorInfo().getError().error_sub_code())))
              << std::endl;
  }
  prev_cmd_pos = command;

  auto action = k_api::Base::Action();

  auto reach_joint_angles = action.mutable_reach_joint_angles();
  auto joint_angles = reach_joint_angles->mutable_joint_angles();

  auto actuator_count = mBase->GetActuatorCount();

  for (size_t i = 0; i < actuator_count.count(); ++i)
  {
    auto joint_angle = joint_angles->add_joint_angles();
    joint_angle->set_joint_identifier(i);
    joint_angle->set_value(radiansToDegrees(command.at(i)));
  }

  std::cout << "Executing action" << std::endl;
  try
  {
    mBase->ExecuteAction(action);
  }
  catch (k_api::KDetailedException& ex)
  {
    std::cout << "Kortex exception: " << ex.what() << std::endl;

    std::cout << "Error sub-code: "
              << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(
                     (ex.getErrorInfo().getError().error_sub_code())))
              << std::endl;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "runtime error: " << ex2.what() << std::endl;
  }
  catch (...)
  {
    std::cout << "Unknown error." << std::endl;
  }
}

void Gen3Robot::sendGripperPositionCommand(const float& command)
{
  std::cout << "Sending gripper position command: " << command << std::endl;
  finger->set_value(command);
  gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);
  try
  {
    mBase->SendGripperCommand(gripper_command);
  }
  catch (k_api::KDetailedException& ex)
  {
    std::cout << "Kortex exception: " << ex.what() << std::endl;

    std::cout << "Error sub-code: "
              << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(
                     (ex.getErrorInfo().getError().error_sub_code())))
              << std::endl;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "runtime error: " << ex2.what() << std::endl;
  }
  catch (...)
  {
    std::cout << "Unknown error." << std::endl;
  }
}

void Gen3Robot::sendVelocityCommand(const std::vector<double>& command)
{
  mLastFeedback = mBaseCyclic->RefreshFeedback();

  auto action = k_api::Base::Action();
  action.set_name("angular action movement");
  action.set_application_data("");

  auto joint_speeds = action.mutable_send_joint_speeds();

  for (std::size_t i = 0; i < command.size() - 1; ++i)
  {
    auto joint_speed = joint_speeds->add_joint_speeds();
    joint_speed->set_joint_identifier(i);
    joint_speed->set_value(radiansToDegrees(command.at(i)));
    joint_speed->set_duration(0.1); // TODO: magic number 0.1
  }

  try
  {
    mBase->SendJointSpeedsCommand(*joint_speeds);
  }
  catch (k_api::KDetailedException& ex)
  {
    std::cout << "Kortex exception: " << ex.what() << std::endl;

    std::cout << "Error sub-code: "
              << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(
                     (ex.getErrorInfo().getError().error_sub_code())))
              << std::endl;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "runtime error: " << ex2.what() << std::endl;
  }
  catch (...)
  {
    std::cout << "Unknown error." << std::endl;
  }
}

void Gen3Robot::sendGripperVelocityCommand(const float& command)
{
  std::cout << "Sending gripper velocity command: " << command << std::endl;
  gripper_command.set_mode(k_api::Base::GRIPPER_SPEED);
  finger->set_value(command);
  try
  {
    mBase->SendGripperCommand(gripper_command);
  }
  catch (k_api::KDetailedException& ex)
  {
    std::cout << "Kortex exception: " << ex.what() << std::endl;

    std::cout << "Error sub-code: "
              << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(
                     (ex.getErrorInfo().getError().error_sub_code())))
              << std::endl;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "runtime error: " << ex2.what() << std::endl;
  }
  catch (...)
  {
    std::cout << "Unknown error." << std::endl;
  }
}

void Gen3Robot::sendGripperEffortCommand(const float& command)
{ 
  gripper_command.set_mode(k_api::Base::GRIPPER_FORCE);
  finger->set_value(command);
  try
  {
    mBase->SendGripperCommand(gripper_command);
  }
  catch (k_api::KDetailedException& ex)
  {
    std::cout << "Kortex exception: " << ex.what() << std::endl;

    std::cout << "Error sub-code: "
              << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(
                     (ex.getErrorInfo().getError().error_sub_code())))
              << std::endl;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "runtime error: " << ex2.what() << std::endl;
  }
  catch (...)
  {
    std::cout << "Unknown error." << std::endl;
  }
}

void Gen3Robot::setBaseCommand()
{
  k_api::BaseCyclic::Command base_command;
  // Initialize each actuator to their current position
  for (unsigned int i = 0; i < mActuatorCount; i++)
  {
    base_command.add_actuators()->set_position(
        mLastFeedback.actuators(i).position());
  }
  mBaseCommand = base_command;

  // Initialize gripper low level command pointer
  gripper_low_level_cmd = mBaseCommand.mutable_interconnect()
                              ->mutable_gripper_command()
                              ->add_motor_cmd();
  // Set position to current gripper position
  gripper_low_level_cmd->set_position(pos[num_full_dof - 1] * 100);
  gripper_low_level_cmd->set_velocity(0.0);
  gripper_low_level_cmd->set_force(100.0);
}

void Gen3Robot::switchToEffortMode()
{
  bool return_status = true;

  // Get actuator count
  mActuatorCount = mBase->GetActuatorCount().count();

  // Clearing faults
  try
  {
    mBase->ClearFaults();
  }
  catch (...)
  {
    std::cout << "Unable to clear robot faults" << std::endl;
    return;
  }

  k_api::BaseCyclic::Feedback base_feedback;

  mServoingMode.set_servoing_mode(
      k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
  mBase->SetServoingMode(mServoingMode);
  mLowLevelServoing = true;
  setBaseCommand();

  // Pre-load gravity-compensated torque (or current) into mBaseCommand
  // BEFORE the SetControlMode loop below.  Without this pre-load, the
  // buffered cyclic frame's `torque_joint` field is 0 (the default
  // from setBaseCommand which only sets position).  As each actuator
  // is then transitioned from POSITION to TORQUE mode by the
  // SetControlMode RPC sequence below (~30 ms × N actuators ≈ 200 ms
  // total), it immediately starts using the 0 torque value from the
  // buffered frame and the joint sags until the next main-loop
  // iteration runs sendTorqueCommand.  By front-loading gravity comp
  // here, each actuator applies the correct holding torque the
  // instant it switches mode, eliminating the ~200 ms startup drop.
  //
  // `pos` is fresh because read() ran earlier in the same main-loop
  // iteration that called write() → switchToEffortMode.
  // `addGravityCompensation` already has the F2 guard against
  // Pinocchio failure (zeroes gravity on throw / non-finite output).
  std::vector<double> startup_gravity_cmd(num_full_dof, 0.0);
  addGravityCompensation(model, data, pos, startup_gravity_cmd);
  for (unsigned int i = 0; i < mActuatorCount; ++i)
  {
    if (current_control)
    {
      double c = startup_gravity_cmd[i] / gear_ratio[i];
      if (!std::isfinite(c)) c = 0.0;
      mBaseCommand.mutable_actuators(i)->set_current_motor(c);
    }
    else
    {
      double t = startup_gravity_cmd[i];
      if (!std::isfinite(t)) t = 0.0;
      mBaseCommand.mutable_actuators(i)->set_torque_joint(t);
    }
  }

  // Taken from Kinova API
  // Send a first frame — now carrying gravity-comp torques/currents.
  mLastFeedback = mBaseCyclic->Refresh(mBaseCommand);

  // Taken from Kinova API
  // Set all actuators to torque mode now that the command is equal to measure
  if (current_control)
    mControlModeMessage.set_control_mode(
        k_api::ActuatorConfig::ControlMode::CURRENT);
  else
    mControlModeMessage.set_control_mode(
        k_api::ActuatorConfig::ControlMode::TORQUE);
  for (int idx = 1; idx < mActuatorCount + 1; idx++)
    mActuatorConfig->SetControlMode(mControlModeMessage, idx);
  std::cout << "Switched successfully to effort mode" << std::endl;
}

void Gen3Robot::sendTorqueCommand(std::vector<double>& command)
{
  if (!mLowLevelServoing)
    return;
  addGravityCompensation(model, data, pos, command);

  // // Initialize each actuator to their current position
  for (unsigned int i = 0; i < mActuatorCount; i++)
  {
    // Taken from Kinova API
    // Position command to first actuator is set to measured one to avoid
    // following error to trigger Bonus: When doing this instead of disabling
    // the following error, if communication is lost and first
    //        actuator continues to move under torque command, resulting
    //        position error with command will trigger a following error and
    //        switch back the actuator in position command to hold its position
    mBaseCommand.mutable_actuators(i)->set_position(
        mLastFeedback.actuators(i).position());
  }
  now = GetTickUs();
  int rate = now - last;
  try
  {
    // Layer 1 boundary guard.  A non-finite torque command makes it
    // onto the wire as a NaN/Inf float; the Kortex firmware silently
    // rejects that actuator's command (no exception, no fault) and
    // the arm goes limp.  Detect at the very last opportunity,
    // substitute zero, and count the event for diagnostics.  Zero is
    // a deliberate choice: it loses gravity comp for one cycle but
    // prevents the silent-limp failure mode and (importantly) keeps
    // the firmware path clean.  See docs/diagnosis_report.md.
    for (unsigned int idx = 0; idx < mActuatorCount; idx++)
    {
      double t = command.at(idx);
      if (!std::isfinite(t))
      {
        m_nan_cmd_count.fetch_add(1, std::memory_order_relaxed);
        m_nan_last_joint.store(static_cast<int>(idx),
                               std::memory_order_relaxed);
        ROS_WARN_THROTTLE(
            1.0,
            "[kortex_hardware] non-finite torque command on joint %u "
            "(was %f) — substituting 0.0",
            idx, t);
        t = 0.0;
      }
      mBaseCommand.mutable_actuators(idx)->set_torque_joint(t);
    }

    // Incrementing identifier ensures actuators can reject out of time frames
    mBaseCommand.set_frame_id(mBaseCommand.frame_id() + 1);
    if (mBaseCommand.frame_id() > 65535)
      mBaseCommand.set_frame_id(0);

    for (unsigned int idx = 0; idx < mActuatorCount; idx++)
    {
      mBaseCommand.mutable_actuators(idx)->set_command_id(
          mBaseCommand.frame_id());
    }
    mLastFeedback = mBaseCyclic->Refresh(mBaseCommand, 0);
  }
  catch (k_api::KDetailedException& ex)
  {
    std::cout << "API error: " << ex.what() << std::endl;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "Error: " << ex2.what() << std::endl;
  }

  last = GetTickUs();
  prev_cmd_eff = command;
}

void Gen3Robot::sendCurrentCommand(std::vector<double>& command)
{
  if (!mLowLevelServoing)
    return;
  addGravityCompensation(model, data, pos, command);

  // Convert torque to current
  // motor coefficient 1~4 : 11 Nm/A, 5~7 : 7.6 Nm/A (consider gear ratio)
  for (int i = 0; i < num_arm_dof; i++)
  {
    command[i] = command[i] / gear_ratio[i];
  }
  if (is_out_lpf_initialized != true)
  {
    out_lpf->initLPF(command);
    is_out_lpf_initialized = true;
  }
  // lpf->getFilteredEffort(eff);
  std::vector<double> filteredCommand = out_lpf->getFilteredEffort(command);
  command = filteredCommand;

  // To avoid current warning
  for (int i = 0; i < mActuatorCount; i++)
  {
    if (std::fabs(command.at(i)) > input_current_limit.at(i))
    {
      command.at(i) = input_current_limit.at(i) * command.at(i)
                      / std::fabs(command.at(i));
    }
  }
  prev_cmd_eff = command;

  // TODO: Check this. Manually delay to match the rate of 1kHz; ROS rate can be
  // inaccurate sometimes
  while (now - last < 1000)
  {
    now = GetTickUs();
  }
  try
  {
    // Initialize each actuator to their current position and set current
    // command
    for (unsigned int i = 0; i < mActuatorCount; i++)
    {
      // Taken from Kinova API
      // Position command to first actuator is set to measured one to avoid
      // following error to trigger Bonus: When doing this instead of disabling
      // the following error, if communication is lost and first
      //        actuator continues to move under torque command, resulting
      //        position error with command will trigger a following error and
      //        switch back the actuator in position command to hold its
      //        position
      mBaseCommand.mutable_actuators(i)->set_position(
          mLastFeedback.actuators(i).position());
      // Layer 1 boundary guard — same rationale as sendTorqueCommand.
      // Current-mode is doubly exposed because `out_lpf` (applied
      // earlier in this function) can latch sticky NaN; the LPF's
      // internal trap-break + this final isfinite check together
      // mean we cannot ship a non-finite current to the wire.
      double c = command.at(i);
      if (!std::isfinite(c))
      {
        m_nan_cmd_count.fetch_add(1, std::memory_order_relaxed);
        m_nan_last_joint.store(static_cast<int>(i),
                               std::memory_order_relaxed);
        ROS_WARN_THROTTLE(
            1.0,
            "[kortex_hardware] non-finite current command on joint %u "
            "(was %f) — substituting 0.0",
            i, c);
        c = 0.0;
      }
      mBaseCommand.mutable_actuators(i)->set_current_motor(c);
    }

    // Incrementing identifier ensures actuators can reject out of time frames
    mBaseCommand.set_frame_id(mBaseCommand.frame_id() + 1);
    if (mBaseCommand.frame_id() > 65535)
      mBaseCommand.set_frame_id(0);

    for (unsigned int idx = 0; idx < mActuatorCount; idx++)
    {
      mBaseCommand.mutable_actuators(idx)->set_command_id(
          mBaseCommand.frame_id());
    }
    mLastFeedback = mBaseCyclic->Refresh(mBaseCommand, 0);
  }
  catch (k_api::KDetailedException& ex)
  {
    std::cout << "API error: " << ex.what() << std::endl;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "Error: " << ex2.what() << std::endl;
  } catch (...) {
    std::cout << "Unknown exception inside 'sendCurrentCommand' caught" << std::endl;
  }
  last = GetTickUs();
}

void Gen3Robot::sendGripperLowLevelCommand(const float& command)
{
  gripper_position_error = (command - pos[num_full_dof - 1]) * 100.0;

  if (fabs(gripper_position_error) < 1.5)
  {
    gripper_low_level_cmd->set_velocity(0.0);
    return;
  }

  gripper_velocity = gripper_proportional_gain * fabs(gripper_position_error);
  if (gripper_velocity > 100.0)
  {
    gripper_velocity = 100.0;
  }

  gripper_low_level_cmd->set_position(command * 100.0);
  gripper_low_level_cmd->set_velocity(gripper_velocity);
}

void Gen3Robot::write(void)
{
  // Ensures safe switching between modes and servoing levels
  if (last_arm_mode != arm_mode)
  {
    try
    {
      // clear faults if any
      mBase->ClearFaults();
      std::cout << "Switching control mode" << std::endl;
    }
    catch (k_api::KDetailedException& ex)
    {
      std::cout << "Kortex exception: " << ex.what() << std::endl;

      std::cout << "Error sub-code: "
                << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(
                       (ex.getErrorInfo().getError().error_sub_code())))
                << std::endl;
    }
    if (last_arm_mode == hardware_interface::JointCommandModes::MODE_POSITION)
      prev_cmd_pos.clear();
    if (last_arm_mode == hardware_interface::JointCommandModes::MODE_EFFORT)
    {
      prev_cmd_eff.clear();
      // Set first actuator back in position
      mControlModeMessage.set_control_mode(
          k_api::ActuatorConfig::ControlMode::POSITION);
      for (int idx = 0; idx < mActuatorCount; idx++)
        mActuatorConfig->SetControlMode(mControlModeMessage, idx + 1);

      // Set the servoing mode back to Single Level
      mServoingMode.set_servoing_mode(
          k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
      mBase->SetServoingMode(mServoingMode);
      mLowLevelServoing = false;

      // Wait for a bit
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    if (arm_mode == hardware_interface::JointCommandModes::MODE_EFFORT
        || arm_mode == hardware_interface::JointCommandModes::EMERGENCY_STOP)
    {
      switchToEffortMode();
    }
    last_arm_mode = arm_mode;
  }

  if (mUseGripper)
  {
    if (mLowLevelServoing)
    {
      // Use gripper in low level servoing mode if arm is in
      // low level servoing mode
      // TODO: Add support for input velocity and force limits
      // See: api_cpp/examples/01-gripper_low_level_command.cpp
      sendGripperLowLevelCommand(cmd_pos[num_full_dof - 1]);
    }
    else
    {
      switch (gripper_mode)
      {
        case hardware_interface::JointCommandModes::MODE_VELOCITY:
          sendGripperVelocityCommand(cmd_vel[num_full_dof - 1]);
          break;
        case hardware_interface::JointCommandModes::MODE_POSITION:
          sendGripperPositionCommand(cmd_pos[num_full_dof - 1]);
          break;
        case hardware_interface::JointCommandModes::MODE_EFFORT:
          sendGripperPositionCommand(cmd_pos[num_full_dof - 1]);
          break;
        default:
          // Stop Gripper
          sendGripperVelocityCommand(0);
      }
    }
  }
  switch (arm_mode)
  {
    case hardware_interface::JointCommandModes::MODE_VELOCITY:
      sendVelocityCommand(cmd_vel);
      break;
    case hardware_interface::JointCommandModes::MODE_POSITION:
      sendPositionCommand(cmd_pos);
      break;
    case hardware_interface::JointCommandModes::MODE_EFFORT:
      if (current_control)
        sendCurrentCommand(cmd_eff);
      else
        sendTorqueCommand(cmd_eff);
      break;
    case hardware_interface::JointCommandModes::EMERGENCY_STOP: {
      vector<double> zero_torque(num_full_dof, 0.0);
      if (current_control)
        sendCurrentCommand(zero_torque);
      else
        sendTorqueCommand(zero_torque);
      break;
    }
    default:
      // Stop Arm
      vector<double> zero(num_full_dof, 0.0);
      sendVelocityCommand(zero);
  }
}

void Gen3Robot::read(void)
{
  // Read the feedback
  if (!mFirstFeedbackReceived || mUseAdmittance)
  {
    mLastFeedback = mBaseCyclic->RefreshFeedback();
    mFirstFeedbackReceived = true;
  }
  for (std::size_t i = 0; i < num_arm_dof; ++i)
  {
    pos[i] = degreesToRadians(double(mLastFeedback.actuators(i).position()));
    if (pos[i] > M_PI)
      pos[i] -= 2 * M_PI;
    vel[i] = degreesToRadians(double(mLastFeedback.actuators(i).velocity()));
    eff[i] = double(mLastFeedback.actuators(i).torque());
  }

  if (is_in_lpf_initialized != true)
  {
    in_lpf->initLPF(eff);
    is_in_lpf_initialized = true;
  }
  in_lpf->getFilteredEffort(eff);
  std::vector<double> filteredEff = in_lpf->getFilteredEffort(eff);
  eff = filteredEff;

  // Read finger state. Note: position and velocity are percentage values
  // (0-100). Effort is set as current consumed by gripper motor (mA).
  pos[num_full_dof - 1]
      = mLastFeedback.interconnect().gripper_feedback().motor()[0].position()
        / 100.0;
  vel[num_full_dof - 1]
      = mLastFeedback.interconnect().gripper_feedback().motor()[0].velocity()
        / 100.0;
  eff[num_full_dof - 1] = mLastFeedback.interconnect()
                              .gripper_feedback()
                              .motor()[0]
                              .current_motor();
}