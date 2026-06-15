# Why BridgeModule can't run against MuJoCo as-is (notes for the BridgeModule owner)

**Short version:** the *bridge logic* in BridgeModule is correct and already
interface-parameterised for sim — but the `BridgeModule.run()` **process**
hard-requires a physical RealSense camera, two Inspire hands on fixed IPs, and a
Unitree MotionSwitcher service. None of those exist when the robot is replaced by
`unitree_mujoco`, so the process aborts before the joint bridge ever starts. We
therefore can't use BridgeModule unmodified for the sim test, and the constraint
is that we don't edit it — hence a thin sim-only state bridge stands in. This
note is so you can decide whether to add a sim/headless mode to BridgeModule
itself.

## What already works (no change needed)

The low-level DDS bridge is exactly what sim needs:
- `getters/lococlient/get_joints_imu.py` subscribes `rt/lowstate`
  (`unitree_hg LowState_`), **domain 0**, **interface from `sys.argv[1]`**
  (`get_joints_imu.py:14-18,24`), and emits the 92-float
  `joints_imu` record. Point it at `lo` and it reads MuJoCo directly.
- `setter/joint_set.py` publishes `rt/lowcmd` the same way
  (`joint_set.py:60-64,77-81`).

So if the process could *start* on a camera-less box, the joint path would work
against MuJoCo with just `... <interface=lo>`.

## The three hard blockers in `BridgeModule.run()`

1. **RealSense is mandatory.** `run()` calls `rs.context().query_devices()` and
   **raises** `RuntimeError("No RealSense device found")` if none is attached
   (`BridgeModule/main/main.py:441-446`), then `capture_camera_calibration()`
   opens a pipeline and forks `camera_process` (`main.py:456,470-471`). On a
   dev/sim box with no camera, this is a hard stop at startup — before the joint
   workers are even forked.

2. **Inspire hands are forked unconditionally.** For each entry in
   `config.HANDS` (`config.py:14-17`) it forks `hand_manager`, which TCP-connects
   to `192.168.124.210` / `.211` (`main.py:475-483`). With no hands on the
   network these workers error/retry; harmless to the joint path but noisy, and
   they assume the robot LAN.

3. **MotionSwitcher has no MuJoCo equivalent.** `set_joints()` calls
   `_release_higher_level_mode()` → `MotionSwitcherClient.ReleaseMode()` and
   **loops until the mode is released** (`joint_set.py:46-54,66`). MuJoCo
   publishes `rt/lowstate` and accepts `rt/lowcmd` but runs **no** MotionSwitcher
   service, so this call blocks indefinitely and `rt/lowcmd` is never written.

## What a "sim mode" for BridgeModule would take (optional, your call)

If you'd like BridgeModule itself to run against MuJoCo (instead of our stand-in),
the minimal changes would be:
- A flag (e.g. `--no-camera` / `BRIDGE_NO_CAMERA=1`) that skips the RealSense
  check + `capture_camera_calibration()` + the `camera_process` fork, and the
  hand forks — so only `get_joints_imu` + `set_joints` + the joints/joint_set
  conductor plumbing run.
- A flag (e.g. `--sim` / `--no-motion-switch`) that skips
  `_release_higher_level_mode()` in `set_joints` (MuJoCo needs no release).
- Optionally honour an interface arg for `lo` (already supported via `sys.argv[1]`).

Until then, the sim loop uses `arch_b_sim/sim_state_bridge.py`, which replicates
**only** `get_joints_imu`'s 92-float record (byte-for-byte: `q27,dq27,tau27,
quat4 wxyz,gyro3,acc3,ts1`) and publishes `/BridgeModule/joints_imu` +
`/BridgeModule/conduct`, with no camera/hands/MotionSwitcher. The motor-write
side is handled by `arch_b_sim/sim_action_consumer.py` (see
`JOINTCOMMANDER_SPEC.md`), not by `set_joints`, because the sim consumer also has
to apply the action transform that `JointCommander` currently doesn't.
