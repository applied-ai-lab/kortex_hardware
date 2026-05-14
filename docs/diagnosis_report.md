# `kortex_hardware` limp-arm diagnosis report

**Status:** investigation. Layer 1 instrumentation deployed in this
same change. Live-failure observations TBD.

## 1. Symptom profile

Failure mode reported by operator:

- Random Kinova Gen3 arm (left or right — both observed across runs)
  randomly **goes limp during operation**. Falls to whatever surface
  is under it under gravity alone.
- **No fault** on the arm base; on-arm LEDs stay green.
- **No erratic motion** preceding the failure — the arm doesn't lurch
  or oscillate, it just stops being held against gravity.
- **No error messages** printed by `kortex_hardware` to stdout/stderr
  (operator watches procman which captures both).
- `/joint_states` continues to publish accurate joint positions /
  velocities after the failure.
- Failure window: random within 2–20 minutes from startup. No obvious
  periodicity. No identified correlation with specific motions.
- The only known recovery: kill and restart the `kortex_hardware`
  node. After restart the arm comes back up cleanly.

What this profile rules out at the start:

| Hypothesis | Why not |
|---|---|
| Hardware fault | Status LEDs green; restart doesn't help if it's hardware |
| Firmware fault | Restart of the host-side process fixes it; firmware state would persist across host restart |
| Network drop | Joint states keep flowing — UDP frames are still landing |
| Total `kortex_hardware` crash | Process is still running, still publishing |
| Cleanly-handled SDK exception | All visible `catch` blocks `std::cout` something; operator saw nothing |

What this profile leaves on the table: **silent breakage of the
command path while the feedback path remains healthy**. The most
parsimonious explanation is that the cyclic frames we ship to the
firmware are accepted by the SDK (no exception), reach the firmware,
and are rejected per-actuator by the firmware's input validation
without an error reply — and we can't see it because we never check
the actuator status fields.

## 2. Hypothesis matrix

Hypotheses ranked by posterior probability after the symptom profile
constrains the space.

### H1 — Non-finite (NaN / Inf) propagation into `set_torque_joint` &nbsp;`P ≈ 0.55`

**Mechanism.** A `NaN` enters `command[i]` upstream of
`Gen3Robot.cpp:838` (`set_torque_joint(command.at(idx))`) or the
analogous `Gen3Robot.cpp:857` (`set_current_motor(command.at(i))`).
The Kortex firmware sees a non-finite float in the cyclic frame and
silently drops that actuator's command for the cycle. The actuator
goes to zero torque output (limp, not POSITION-held), and once the
NaN source persists, every cycle is rejected.

**Why no exception is raised.** NaN is data, not protocol error. The
SDK's `BaseCyclic::Refresh` ships the bytes; the firmware does
input-validation internally and per-actuator. The protocol has no
"reject" reply — the absent torque manifests only as the arm not
moving. Feedback frames return normally with valid position /
velocity / torque measurements (which is why `/joint_states` keeps
working).

**Why "restart fixes it."** Some in-process state has become wedged
in a non-finite value, and the restart wipes it. Specifically, the
`LowPassFilter::tau_J_prev` member (`LowPassFilter.cpp:31`) stores
the previous filtered value:

```cpp
filtered_tau_J[i] = tau_J_prev[i] * alpha_ + tau_J_raw[i] * (1 - alpha_);
tau_J_prev[i] = filtered_tau_J[i];
```

If `tau_J_prev[i]` ever becomes NaN, every subsequent call returns
NaN, regardless of how clean `tau_J_raw[i]` is. This is **sticky
NaN** — restart-only recovery is the signature.

**Plausible NaN sources, in descending likelihood:**

| Source | Sticky? | Notes |
|---|---|---|
| `out_lpf` (current-mode command filter) latches NaN | **Yes** | Best fit to "random within minutes, restart-only recovery." Only relevant in `current_control=true` mode. |
| `in_lpf` (effort feedback filter) latches NaN, controller reads NaN `eff[]`, writes NaN `cmd_eff` | Yes (LPF) + maybe (controller) | Affects both torque and current modes |
| Controller writes NaN to `cmd_eff` directly | Depends on controller — sticky if controller has stuck NaN state internally | Stock `joint_trajectory_controller` shouldn't; custom controllers might |
| `pos[i]` becomes NaN → `q_` has NaN → Pinocchio returns NaN → `command += gravity_` is NaN | No (single-cycle unless `pos` stays bad) | Would need Kinova to emit NaN in feedback — rare |
| `pinocchio::computeGeneralizedGravity` numerical breakdown | No | RNEA is well-conditioned; needs NaN-in to produce NaN-out |

The LPF sticky-NaN path is the strongest match to the user's symptom
profile because it's the only candidate that's both:

1. **Triggered by a transient event** (one bad UDP cycle, one
   numerical spike, one transient compute glitch) → would give random
   2–20 min timing, no obvious period.
2. **Unrecoverable in-process** (no logic anywhere in the code resets
   `tau_J_prev` once it becomes NaN) → matches restart-only fix.

### H2 — Firmware following-error demotion &nbsp;`P ≈ 0.05` *(user marked unlikely)*

**Mechanism.** The Kortex firmware monitors actuator following error
(commanded position vs measured). On overshoot it auto-demotes the
actuator to POSITION mode, holding position rather than continuing
to apply the over-large torque command. Comment at
`Gen3Robot.cpp:826-829` describes this:

```
// if communication is lost and first actuator continues to move
// under torque command, resulting position error with command will
// trigger a following error and switch back the actuator in
// position command to hold its position
```

**Why this doesn't match the symptom.** The described firmware
behaviour is "hold position," not "go limp." Limp implies zero
torque output, not a position-mode hold. So even if following-error
demotion is occurring, it wouldn't explain the falling arm.

Kept here for completeness; user confirmed they consider this
unlikely.

### H3 — Frame-ID rollover edge case &nbsp;`P ≈ 0.05`

**Mechanism.** `mBaseCommand.set_frame_id(mBaseCommand.frame_id()+1)`
at `Gen3Robot.cpp:841` increments per cycle. At 1100 Hz, wraps
65535→0 about every 59.6 s. If firmware treats 0 (after seeing
65535) as a "frame went backwards" event and rejects subsequent
frames silently, we'd see... silent rejection.

**Why this doesn't match cleanly.** Time pattern is "random within
2–20 min" — if rollover were the trigger, you'd see a strongly
bimodal distribution clustered around multiples of 60 s. Not what
operator reports. Still worth instrumenting (Layer 4 in the plan) —
the rollover counter is essentially free to track.

### H4 — Memory corruption / use-after-free &nbsp;`P ≈ 0.05`

**Mechanism.** Long-running C++ code with raw pointers, no smart-
pointer ownership, complex shared state. An off-by-one or use-after-
free could write into `command[]` storage producing NaN bit
patterns.

**Why it ranks low.** Both arms (separate processes, separate memory
spaces) fail. Memory corruption tends to be process-specific. If
both processes have the same bug, it'd be a deterministic issue at
the source-level — not a "random 2–20 min" timing pattern.

**How to surface.** Build with `AddressSanitizer` (`-fsanitize=address`)
and `UndefinedBehaviorSanitizer`. Cost is 2–3× slowdown — still
runs at >300 Hz which is sufficient to observe the bug.

### H5 — Soft-stop callback ↔ main thread race &nbsp;`P ≈ 0.03`

The new soft-stop subscribers in `Gen3Robot::estopCallback` and
`clearFaultsCallback` fire from the AsyncSpinner thread. They touch
`mBase` (TCP) and `mServoingMode`. The main thread touches
`mBaseCyclic` (UDP) and may concurrently read `mServoingMode` during
mode switches.

**Why low.** User confirms the bug pre-dates the soft-stop additions
(this is the bug they've been chasing). So the soft-stop code isn't
the cause. Listed for completeness because the *fix* lives in the
same file and any future regression on the cause could now bear
race-related symptoms.

### H6 — Kortex SDK internal state degradation &nbsp;`P ≈ 0.05`

**Mechanism.** The SDK is closed-source; a memory leak or queue
back-pressure bug inside the SDK could manifest as silent failure
after enough cycles.

**Why hard to diagnose.** We don't have SDK source. Best we can do
is observe via packet capture (Layer 5 in the plan) — if our
outgoing frames are clean but the firmware behaviour suggests we're
sending garbage, the SDK is the suspect.

### H7 — Controller-side numerical issue specific to your setup &nbsp;`P ≈ 0.12`

If a custom impedance or admittance controller is loaded, a state-
dependent singularity (Jacobian inverse, division by joint velocity
near zero, atan2 near 0/0) could produce NaN. Without knowing which
controllers are running, hard to rank higher.

**Surfaced by Layer 1 directly:** the `m_nan_cmd_count` counter
catches the boundary. If it increments while `m_nan_lpf_*_count`
stays at zero, the upstream controller is the culprit.

## 3. Pinocchio-side analysis (step 5)

`pinocchio::computeGeneralizedGravity` is called every cycle in
effort mode. Failure modes to consider:

**Dimension mismatch.** If `num_arm_dof` (from rosparam) doesn't
match the URDF parsed by Pinocchio, indices into `model.nqs[]` /
`model.idx_qs[]` can go out of range. **Ruled out for this bug**
because it would fail immediately at startup, not "after several
minutes." Worth a one-line assertion if not present, but not the
cause.

**Continuous-joint encoding.** Pinocchio uses (cos, sin) for the
continuous joints in `q_`. The code does:

```cpp
if (model.nqs[jidx] == 2) {
  q_[qidx]     = std::cos(config[i]);
  q_[qidx + 1] = std::sin(config[i]);
}
```

If `config[i]` (== `pos[i]`) is ever NaN, `cos(NaN)=NaN` and
`sin(NaN)=NaN`, contaminating `q_`. RNEA then propagates the NaN
through every gravity term. **This is the H1 propagation path** — the
fix is to either (a) prevent NaN getting into `pos[]` at the source
(can't easily — comes from feedback), or (b) catch the result at the
gravity-comp output boundary (what step 4 does).

**Singularity behaviour.** Unlike Jacobian-based IK, RNEA is
well-conditioned everywhere on SE(3) for revolute joints. An "unusual
arm pose" by itself doesn't produce NaN. NaN out almost always means
NaN in.

**`pinocchio::Data` reuse.** Each `computeGeneralizedGravity` call
rewrites the intermediate buffers in `data` (velocities,
accelerations, forces). There's no cross-cycle state accumulation
that could degrade silently over minutes. Not a source of slow
drift.

**Threading.** `model` and `data` are owned by `Gen3Robot`, accessed
only by the main thread inside `write()`. No spinner-side access.
Not a race source.

**Memory layout.** `q_` is `Eigen::VectorXd`, sized to `model.nq` in
the constructor (`q_ = pinocchio::neutral(model)`). No per-cycle
allocation; no resize.

**Conclusion.** Pinocchio is most likely a NaN *propagator*, not a
NaN *source*. The defensive measures in step 4 (try/catch on the
call, scan the output for non-finite) are sufficient.

## 4. Recommended fix order

The diagnostic measures in step 3 + 4 of the current change set serve
**both** as evidence-gathering and as partial mitigation. If H1 is
correct, the user-visible behaviour after these changes should shift
from "persistent limp until restart" to "brief transient + diagnostic
counter incrementing." That alone is a significant win.

| Layer | What it gives | Cost |
|---|---|---|
| **Layer 1** (this change set) | NaN guards at LPF + cyclic-cmd boundary + gravity output. Diagnostic counters published at 1 Hz. | ~50 ns/cycle |
| Layer 2 (optional next) | Actuator `status_flags` / `fault_bank_*` monitoring (per-actuator firmware status). Surfaces H2 + firmware-side anomalies our code currently ignores. | ~50 ns/cycle |
| Layer 3 (optional next) | Lock-free ring buffer of 2048 cycles dumped on service call. Per-cycle scope trace of the 2 seconds leading up to failure. | ~80 ns/cycle |
| Layer 4 (optional next) | 1100 Hz loop-period histogram. Surfaces CPU pressure / scheduling glitches that could precede failure. | <10 ns/cycle |
| Layer 5 (optional next) | `tcpdump` on UDP port 10001. Byte-level ground truth on what we ship to firmware. Heavyweight but offline. | Kernel-handled, near-zero impact on loop |

Defer Layers 2–5 until Layer 1 has had a chance to either confirm
or rule out H1.

## 5. Expected outcomes after this change

| Failure scenario | Behaviour after Layer 1 + gravity guards |
|---|---|
| **H1, sticky NaN in `out_lpf`** (best fit to user symptom) | LPF resets `tau_J_prev` to a finite value within 1 sample. Arm experiences a ~1–2 ms torque dropout, then resumes. `m_nan_lpf_out_count` increments. Diagnostic ROS_WARN fires once (throttled). **The user-visible bug is effectively fixed and we know the root cause.** |
| **H1, sticky NaN in `in_lpf`** | Same as above on the eff[] side. Any controller using eff feedback sees one bad sample, then clean values. `m_nan_lpf_in_count` increments. |
| **H1, transient NaN at the cmd boundary** | Boundary guard substitutes 0 torque for the bad joint. One-cycle dip. `m_nan_cmd_count` increments. Arm continues normally. |
| **H1, persistent NaN at the cmd boundary** (controller stuck) | Persistent zero-torque on the affected joint(s) — arm still limp. **But** `m_nan_cmd_count` runs away at 1 kHz, ROS_WARN spams, diagnostic topic shows the runaway. The bug is no longer silent — we know exactly which joint and that the controller is the culprit. |
| **H1, gravity comp NaN** | Output scan zeroes the gravity vector for that cycle. Combined with whatever `command[i]` is, the result is finite. `m_nan_gravity_count` increments. |
| **Pinocchio throws** | try/catch zeros gravity for the cycle, logs `ROS_ERROR_THROTTLE`. Arm continues with controller-only torque (no gravity comp) until the throw clears. |
| **H1 wrong (no NaN observed during failure)** | All four counters stay at 0 through the failure window. Confirms H1 is **not** the cause. Pivot to Layer 2 (status flags). Strongly informs next step. |

The asymmetric value of this experiment: even a null result is
informative. If we deploy and the bug recurs without any counter
incrementing, we've ruled out H1 with high confidence and the
remaining hypothesis space (H2 firmware-demotion, H4 memory, H6 SDK)
is much narrower.

## 6. Counter semantics reference

For the operator-side dashboard:

| Counter | Increments when | What it implicates |
|---|---|---|
| `m_nan_cmd_count` | `command[i]` non-finite right before `set_torque_joint` / `set_current_motor` | Upstream caller of `sendTorqueCommand` (controller, or `sendCurrentCommand`'s own preprocessing) wrote NaN. If `nan_lpf_*` is zero, **controller is the source**. |
| `m_nan_lpf_in_count` | Feedback LPF (`in_lpf`) produced a non-finite output | Bad torque feedback from Kinova firmware, OR sticky NaN in `tau_J_prev` |
| `m_nan_lpf_out_count` | Command LPF (`out_lpf`, current mode only) produced non-finite | Bad input to LPF (i.e. NaN command from gravity comp or controller) OR sticky NaN in `tau_J_prev` |
| `m_nan_gravity_count` | Pinocchio threw, OR gravity output had non-finite values | Bad `q_` going in (likely bad `pos[]` upstream), OR Pinocchio internal numerical issue |
| `m_nan_last_joint` | Index of the last joint where a non-finite command was caught | Identifies which actuator is the symptom origin |

Published at 1 Hz on `~diagnostics/nan_counts`
(`std_msgs/UInt64MultiArray`, `data` = [cmd, lpf_in, lpf_out,
gravity, last_joint]). Subscribe with rqt_plot for live monitoring;
record into rosbag for post-incident analysis.

## 7. What this report does NOT do

- Does not provide a packet-capture analysis. If the NaN-guard
  hypothesis is wrong, the next step is Layer 5 (UDP capture on
  port 10001) to get byte-level ground truth on what we ship vs
  what the firmware expects.
- Does not investigate the controller stack. If `m_nan_cmd_count`
  runs away during a failure (meaning a NaN is coming from a
  controller), the next debugging move is on the controller side
  — instrumenting the loaded effort controller, or `cm.update()`.
- Does not modify the soft-stop integration. The latch + callback
  are orthogonal to this bug and stay as deployed.
