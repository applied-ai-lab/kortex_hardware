#include <iostream>
#include <memory>
#include <sstream>

#include <Gen3Robot.h>
#include <ros/rate.h>

#include "std_msgs/String.h"

int main(int argc, char* argv[])
{
  ROS_INFO_STREAM("Gen3 HARDWARE starting");
  ros::init(argc, argv, "kortex_hardware");
  ros::NodeHandle nh;

  // Construct the hardware interface.  The Gen3Robot constructor
  // performs all heavy initialization (Kortex session, URDF parse,
  // controller registration, real-time setup).  Any of those steps
  // can throw — wrap in a top-level handler so the process exits
  // cleanly with a non-zero status and a clear log line instead of
  // std::terminate'ing with an opaque backtrace.
  std::unique_ptr<Gen3Robot> robot;
  try
  {
    robot = std::unique_ptr<Gen3Robot>(new Gen3Robot(nh));
  }
  catch (const std::exception& ex)
  {
    ROS_FATAL("[kortex_hardware] startup failed: %s", ex.what());
    return 1;
  }

  controller_manager::ControllerManager cm(robot.get());
  bool use_admittance = false;
  ros::param::get("~use_admittance", use_admittance);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Ros control rate of 1100Hz
  ros::Rate controlRate(1100);
  while (ros::ok())
  {
    robot->read();
    cm.update(robot->get_time(), robot->get_period());

    // Skip the cyclic write while the soft-stop is latched.  The
    // firmware has already faulted the arm via ApplyEmergencyStop;
    // pushing further cyclic frames would just produce error spam
    // until ClearFaults runs.  See Gen3Robot.cpp::estopCallback.
    if (!use_admittance && !robot->estopLatched())
      robot->write();
    controlRate.sleep();
  }

  return 0;
}
