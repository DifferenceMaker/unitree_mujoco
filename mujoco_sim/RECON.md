# ARCH_B_RECON.md — Architecture B vs. unitree_mujoco (Phase 0 findings)

**Goal of the effort:** run the ROS2 module stack
(BridgeModule → MovementModule → consumer → motors) end-to-end against
`unitree_mujoco` as a stand-in for the H1-2, to validate the Architecture B
balance pipeline in sim before it touches the real robot.

**Status of this document:** Phase 0 recon only. No code built yet. Every claim
below is cited to real source (`file:line`). Mismatches between the
session-log "contract" and the actual code are called out in
[§9 Mismatches](#9-mismatches-logged-contract-vs-actual-code).

**Scope note / module ownership:** Only `MovementModule` may be edited.
`BridgeModule`, `ActionModule` (incl. `JointCommander`) are a teammate's and
stay untouched — any glue lives here in the `unitree_mujoco` branch
`mujoco_sim`. `unitree_rl_lab` (the C++ reference controller + config) is
**not** a locked module and may be edited for the comparison.

---

## TL;DR — the one thing that changes the plan

The session logs assumed two things that turned out **false**, and they reshape
the build:

1. **BridgeModule already IS the low-level DDS↔ROS2 bridge.** Despite the
   directory name `getters/lococlient/`, it does **not** use the high-level
   Unitree `LocoClient`. It uses the **low-level** `unitree_sdk2py` channel API
   directly: subscribes `rt/lowstate` (`LowState_`, unitree_hg) and publishes
   `rt/lowcmd` (`LowCmd_`), on **domain 0**, with the **network interface taken
   from `sys.argv[1]`**. So pointing it at MuJoCo on `lo` is, in principle, a
   one-arg change. *(Confirmed: `BridgeModule/main/getters/lococlient/get_joints_imu.py:11-18`,
   `BridgeModule/main/setter/joint_set.py:26-32,60-64,77-81`.)*

2. **The action transform / SDK remap is applied NOWHERE in the live chain.**
   `ActionModule` *does* now subscribe to `/MovementModule/policy_action`
   (the 2026-06-10 log saying it did not is stale), **but** `JointCommander`
   forwards the raw action straight into joint slots `0..12` with only a
   joint-limit clamp — it does **not** apply `action*0.25 + offset`, the knee
   clip, or the `ACTION_SDK_IDS` slot remap. The 13 raw policy outputs are
   therefore (a) used as if they were radians and (b) routed to the wrong SDK
   joints. *(Confirmed: `ActionModule/main/conductor.py:88-90,153-169`,
   `ActionModule/main/joint_commander.py:127-197`.)*

Consequence: the as-built production chain **cannot balance** even on the real
robot, for two independent reasons (missing transform + wrong gains, see §9).
Both bugs are in locked modules. So for the sim test we follow the task's own
design: a **sim-only consumer** that reads `/MovementModule/policy_action`,
applies the verified transform, and writes `rt/lowcmd` directly to MuJoCo —
bypassing the (locked, sim-hostile) `ActionModule` + `BridgeModule`. The real
fixes are handed to the teammate as a written spec.

---

## 1. BridgeModule's role

**Yes — it is the DDS↔ROS2 boundary, at the low level, both directions.**

### State path (DDS → ROS2): `get_joints_imu.py`
- Imports `unitree_sdk2py.core.channel` + `unitree_hg ... LowState_`
  (`get_joints_imu.py:11-12`).
- `interface = sys.argv[1] if len(sys.argv) >= 2 else None;
  ChannelFactoryInitialize(0, interface)` — **domain 0, interface from argv**
  (`get_joints_imu.py:14-18`). This is the "lo interface arg" the logs mention.
- Subscribes `rt/lowstate` (`get_joints_imu.py:24`).
- Builds the **92-float** record and pushes it to the conductor, which
  publishes it on ROS2 as `joints_imu` (`get_joints_imu.py:44-50`,
  `main.py:383`). Layout (`get_joints_imu.py:36-47`):
  `[q(27), dq(27), tau(27), quat(4 wxyz), gyro(3), acc(3), ts(1)]` = 92.
  Matches `balance_contract` slices exactly (`balance_contract.py:62-67`).
- ROS2 topic: `/BridgeModule/joints_imu` (auto-prefixed by the connector,
  `Ros2_connector.py:290-291`).

### Command path (ROS2 → DDS): `setter/joint_set.py`
- Same low-level SDK, **interface from `sys.argv[1]`, domain 0**
  (`joint_set.py:60-64`).
- Subscribes `rt/lowstate`, publishes `rt/lowcmd` (`joint_set.py:77-81`).
- Consumes the inbound ROS2 topic `/BridgeModule/joint_set` (27 floats,
  radians) via the conductor (`main.py:411-417`), slew-limits it
  (`MAX_JOINT_RATE=1.0 rad/s`), and writes `rt/lowcmd` at 500 Hz with **flat
  gains `kp=50.0, kd=1.0, tau=0`** (`joint_set.py:122-132`, `config.py:34-37`).
- **Calls `MotionSwitcherClient.ReleaseMode()` first** to take low-level control
  (`joint_set.py:46-54,66`).

### Topics produced (confirmed)
- `/BridgeModule/joints_imu` — 92-float measured state (**yes**, the array the
  obs is built from).
- `/BridgeModule/joint_set` — this is **consumed**, not produced, by
  BridgeModule. It is **produced by ActionModule/JointCommander**
  (`joint_commander.py:106,184`). It is the 27-float commanded body pose
  (legs+torso+arms, radians), **not** a 27-float "IK-commanded arm pose"; the
  arm slots `13..26` carry the IK arm command, the leg/torso slots `0..12` carry
  whatever JointCommander put there. (The logs' "joint_set = 27-float
  IK-commanded arm pose" is imprecise: it's a full-body vector; only the arm
  slots are IK.)
- Plus camera/hand topics (`rgb`, `depth`, `hand`) irrelevant to balance.

### ⚠ Can BridgeModule's `main.py` actually run in sim?  **No (as-is).**
`BridgeModule.run()` hard-requires hardware unrelated to balance:
- Raises `RuntimeError("No RealSense device found")` if no camera
  (`main.py:441-446`), and forks a camera process + captures calibration
  (`main.py:456,470-471`).
- Forks Inspire-hand managers that TCP-connect to `192.168.124.210/211`
  (`config.py:14-17`, `main.py:475-483`).
- `set_joints` calls `MotionSwitcherClient.ReleaseMode()` — there is **no
  MotionSwitcher service in MuJoCo**, so it would block in
  `_release_higher_level_mode()` (`joint_set.py:46-54`).

So even though the *bridge logic* is correct and interface-parameterised, the
*process* that hosts it can't be brought up on a dev box against MuJoCo without
editing BridgeModule — which is forbidden. → we replicate just the two relevant
workers (state-out, cmd-in) as thin sim-only nodes (see INTEGRATION_DESIGN.md).

---

## 2. The action consumer

**Does anything subscribe to `/MovementModule/policy_action` and apply
`target = action*0.25 + offset (+ knee clip)`?**
→ Something subscribes; **nothing applies the transform or the remap.**

- `ActionModule/main/conductor.py:88-90` subscribes
  `Float32MultiArray` on `/MovementModule/policy_action`, callback
  `_on_legs_command`.
- `_on_legs_command` (`conductor.py:153-169`) caches the first 13 values into
  `self.legs_command` and sets `legs_received=True`. No transform.
- `JointCommander._loop` (`joint_commander.py:127-197`) seeds a 27-vector from
  the latest measured `current_body`, overlays `legs_command[0:13]` into slots
  `0..12` **verbatim** when `legs_received`, overlays arm IK into `13..26`,
  **clamps each joint to its mechanical limit** (`joint_commander.py:28-61,
  ~175-180`), and publishes to `/BridgeModule/joint_set`. **No `*0.25`, no
  `+offset`, no `[0.45,2.5]` knee clip, no `ACTION_SDK_IDS` remap.**

Where motor commands ultimately get written: `BridgeModule/setter/joint_set.py`
→ `rt/lowcmd` (§1). ActionModule itself touches **no** DDS and **no** `rt/lowcmd`
(pure ROS2; no `unitree_sdk2`/`cyclonedds` imports anywhere in ActionModule).

**Net:** the contract's required consumer math (`balance_contract.py:94-118`)
is unimplemented. The raw 13 actions are mis-scaled and mis-routed before they
reach the motors.

---

## 3. The arm-IK feed (`joint_set` for `arm_pose_command`)

- MovementModule reads `/BridgeModule/joint_set` and slices **only the arm
  slots** via `ARM_SDK_ORDER` for the `arm_pose_command` obs term
  (`MovementModule/main/main.py:80-88`, `balance_contract.py:78-79`). Leg/torso
  slots of joint_set are ignored by MovementModule (good — no feedback loop).
- `joint_set` is produced by `JointCommander` (§1). The arm slots come from the
  IK solver (`ActionModule/main/robot/arm.py` → `desired_state` → merged at
  `joint_commander.py`).
- **The IK is passive**: it only produces a pose when a sequence calls
  `arm.move(...)`. With no sequence running, JointCommander leaves the arm slots
  at the **measured** current pose (seeded from `joints_imu`), not a commanded
  pose. There is **no autonomous default-arm publisher**.
- For a first sim test: publish a **static default arm pose** on
  `/BridgeModule/joint_set` ourselves (arms at 0, the p7_1b default), so
  MovementModule has a valid, non-`None`, non-moving `arm_pose_command`.
  (MovementModule returns early if joint_set is `None` —
  `MovementModule/main/main.py:80-83`.)

---

## 4. Launch / ordering

- Each module has a `st.sh` that sources ROS2 Humble, activates a per-module
  venv `/workspace/.venv/<Module>/`, and runs `main/main.py $@`
  (`MovementModule/st.sh`, identical pattern in `BridgeModule/st.sh`,
  `ActionModule/st.sh`).
- The repo-level launcher is `.setup/st.sh` (builds/runs the
  `ros2-humble-dev` docker image, `--network host`, `--ipc=host`), delegating to
  `.setup/setup_scripts/module_startup.sh`, which takes `--one <Module>`,
  `--all`, `--add-pip <pkg>`, `--debug-*` etc.
- **`.setup/modules.sh` lists `BridgeModule, RvizModule, VisualModule,
  ActionModule` — `MovementModule` is NOT in `--all`** and is brought up
  separately (e.g. `bash .setup/st.sh --one MovementModule`). `onnxruntime` is
  already in `MovementModule/requirements.txt`.
- Required up-order: a producer's topics must exist before a consumer reads
  them, but the connector's cached-subscription pattern (`get_data` returns
  `None` until first msg, `Ros2_connector.py:104-120`) makes modules tolerant of
  late peers — they idle/early-return rather than crash. MovementModule
  early-returns until both `joints_imu` and `joint_set` are flowing
  (`main.py:72-87`). So the only hard requirement is: **state source + arm-pose
  source up before MovementModule produces useful actions.**
- Modules **can** be launched individually (`--one`), which is what the sim
  loop will do for MovementModule.

---

## 5. unitree_mujoco interface

**Use the C++ `simulate/` build, not `simulate_python/`.** (Why: the Python
bridge selects the message family by `if config.ROBOT=="g1"` else unitree_go
(`simulate_python/unitree_sdk2py_bridge.py:16-23`) — with `robot="h1_2"` it would
wrongly pick **unitree_go**. The C++ sim selects by motor count.)

- **Topics:** publishes `rt/lowstate`, subscribes `rt/lowcmd`
  (`simulate/src/unitree_sdk2_bridge.h` topic constants; PD law at
  `unitree_sdk2_bridge.h:181-186`).
- **Message family:** `m->nu > NUM_MOTOR_IDL_GO` (H1-2 has 27 actuators > Go2's
  20) → **`G1Bridge` = unitree_hg `LowState_`/`LowCmd_`**
  (`simulate/src/main.cc:669-674`, `unitree_sdk2_bridge.h:257-259`). **Matches
  BridgeModule and the C++ controller.** ✔
- **Interface + domain:** `config.yaml` `domain_id: 0`, `interface: "lo"`
  (`simulate/config.yaml:4-5`); overridable with `-i`/`-n`
  (`simulate/src/param.h:62-63`). ✔ domain 0 on lo.
- **Joint count/order:** 27 motors, actuator order =
  `left_hip_yaw, left_hip_pitch, left_hip_roll, left_knee, left_ankle_pitch,
  left_ankle_roll, [right leg ×6], torso, [left arm ×7], [right arm ×7]`
  (`unitree_robots/h1_2/h1_2.xml:206-232`). **This is exactly SDK
  BODY_JOINT_ORDER — no remap** — and matches the C++ controller's documented
  motor numbering (`unitree_rl_lab/.../State_RLBase.cpp:10-35`) and
  balance_contract's assumptions. ✔
- **LowState content:** per-motor `q/dq/tau_est`, IMU **quaternion wxyz**
  (`unitree_sdk2_bridge.h:198-206`), gyro, accel, `tick` (ms). Maps 1:1 to the
  92-float `joints_imu` record (§1). Quat order wxyz matches
  `quat_rotate_inverse_gravity` (`balance_contract.py:132-141`). ✔
- **LowCmd content:** per-motor `q, dq, kp, kd, tau` → PD + feedforward:
  `ctrl[i] = tau + kp*(q - q_meas) + kd*(dq - dq_meas)`
  (`unitree_sdk2_bridge.h:181-186`). The sim consumer must set `kp/kd` per motor
  (see §7). ✔
- **XML is the mass-corrected variant-D:** `scene.xml` includes `h1_2.xml`
  (`unitree_robots/h1_2/scene.xml:2`); `torso_link` mass **27.289 kg**, CoM
  **(0.030, −0.025, 0.17)** (`h1_2.xml:113-114`), ~76.5 kg total handless. The
  pre-correction model is preserved as `h1_2.xml.bak_premass`. ✔ Current model =
  the one we want.
- **Launch:** `simulate/build/unitree_mujoco` (prebuilt, dated Jun 12) reads
  `simulate/config.yaml`; `use_joystick: 1` and `enable_elastic_band: 1` are on
  by default (open question §10).

---

## 6. The reference path (the behaviour the ROS2 path must reproduce)

The C++ controller `unitree_rl_lab/deploy/robots/h1_2/build/h1_2_ctrl`
(prebuilt) is the known-good deploy path and the smoke test:

- `main.cpp:36` `ChannelFactory::Init(0, vm["network"])` — **domain 0,
  `--network` interface**. Against MuJoCo: `h1_2_ctrl --network lo`.
- It runs an Isaac-Lab-ported `ManagerBasedRLEnv`: the obs is assembled by the
  C++ observation manager and the action transform (`scale/offset/clip`) is
  applied inside `action_manager->processed_actions()`
  (`State_RLBase.cpp:62-64,105-113`). The `run()` loop then maps each processed
  action to its motor via `LEG_TORSO_URDF_IDS[i] → joint_ids_map → motor_idx`
  (`State_RLBase.cpp:18-35,105-113`).
- **Cross-check of the action→motor map** (decisive): computing
  `joint_ids_map[LEG_TORSO_URDF_IDS[i]]` for i=0..12 yields
  **`[0,6,12,1,7,2,8,3,9,4,10,5,11]`**, which is **identical to
  `balance_contract.ACTION_SDK_IDS`** (`balance_contract.py:105-107`). ✔ The
  contract's remap is verified against the reference, not just trusted.
- **Gains:** `State_RLBase::enter()` overwrites every motor's `kp/kd` from
  `env->robot->data.joint_stiffness / joint_damping`
  (`State_RLBase.h:15-24`) — i.e. the policy's **deploy.yaml
  `stiffness`/`damping`** arrays (motor order):
  `kp=[200,200,200,300,40,40, ×2 legs, 300 torso, 100,100,50,50,50,50,50 ×2
  arms]`, `kd=[2.5,2.5,2.5,4,2,2, …, 6 torso, 2…]`
  (`logs/milestones/p7_1b/params/deploy.yaml:4-9`). The FixStand kp/kd in
  `config/config.yaml:40-53` are used **only** during the stand-up ramp and are
  replaced on entry to balance. **→ the sim consumer's `rt/lowcmd` gains must be
  these deploy.yaml gains, NOT BridgeModule's flat 50/1.**
- **Arms:** written from `ArmPosePublisher` (Training held-pose mode)
  (`State_RLBase.cpp:59,123-131`), independent of the policy.
- **Policy selection caveat:** `config/config.yaml:69-72` points `BalancePush`
  at `logs/milestones/phase5_v3`, whose `deploy.yaml` default pose is the
  **p6mass_D era** (`-0.16/0.4/0.36/-0.2`) and whose onnx differs from p7_1b
  (md5 `fd5ead…` vs `4fafc0…`). MovementModule runs **p7_1b**
  (`MovementModule/policy/CURRENT` = `p7_1b`). **The p7_1b milestone exists at
  `unitree_rl_lab/logs/milestones/p7_1b` and its onnx is byte-identical to
  MovementModule's vendored p7_1b (both md5 `4fafc04b…`).** → For an
  apples-to-apples comparison, repoint `BalancePush.policy_dir` to
  `../../../logs/milestones/p7_1b` and rebuild/relaunch `h1_2_ctrl`.

---

## 7. The verified contract (single source of truth)

`MovementModule/main/balance_contract.py` is internally consistent and matches
both `policy/p7_1b/deploy.yaml` and the C++ controller:

| quantity | value | source / cross-check |
|---|---|---|
| obs dim | 87 | `balance_contract.py:46`; ONNX input width asserted live (`main.py:64-65`) |
| obs layout | ang_vel(3,×0.2)+proj_grav(3)+jpos_rel(27)+jvel_rel(27,×0.05)+last_action(13)+arm_cmd(14) | `balance_contract.py:53-60` ↔ deploy.yaml `observations` |
| action scale | 0.25 | `balance_contract.py:99` ↔ deploy.yaml `actions.*.scale` |
| action offset | `[0,0,0,-0.3,-0.3,0,0,0.6,0.6,-0.3,-0.3,0,0]` | `balance_contract.py:100-103` ↔ deploy.yaml `actions.*.offset` |
| action clip | knees (slots 7,8) `[0.45,2.5]`, else ±inf | `balance_contract.py:111-118` ↔ deploy.yaml `actions.*.clip` |
| **action→motor map** | `ACTION_SDK_IDS=[0,6,12,1,7,2,8,3,9,4,10,5,11]` | `balance_contract.py:105-107` ↔ **C++ `State_RLBase.cpp` computed map** |
| obs joint read order | `OBS_SDK_ORDER=[0,6,1,7,2,8,3,9,4,10,5,11,12,13,20,…]` | `balance_contract.py:72-76` |
| default pose (p7_1b) | `[…,-0.3,-0.3,0.4,0.4,…,0.6,0.6,…,-0.3,-0.3,0.3,0.3,…]` | `balance_contract.py:83-87` ↔ deploy.yaml `default_joint_pos` |
| step dt | 0.02 (50 Hz) | `balance_contract.py:92` |
| **PD gains (deploy)** | stiffness/damping arrays, motor order | deploy.yaml `stiffness`/`damping` (used by C++ `enter()`) |

`offline_check.py` already validates the obs builder against known-truth frames
**and** drift-guards the constants against the active policy's `deploy.yaml`
(`offline_check.py:84-132`). Reuse it as the offline contract test.

The sim consumer must **import** `ACTION_SCALE/OFFSET/CLIP/SDK_IDS` and
`ARM_SDK_ORDER` from `balance_contract.py` rather than re-hardcoding. (deploy.yaml
gains are read from the milestone's `params/deploy.yaml`.)

---

## 8. The actual end-to-end data flow (as built today)

```
            ┌─────────────── DDS (unitree_hg, domain 0, lo) ───────────────┐
 MuJoCo/robot ──rt/lowstate──▶ Bridge/get_joints_imu ──ROS2 /BridgeModule/joints_imu──▶ MovementModule
                                                                                            │ (87-obs → ONNX)
                                            (arm slots) /BridgeModule/joint_set ◀───────────┤ reads for arm_cmd
                                                              ▲                             │
                                                              │                  ROS2 /MovementModule/policy_action (13 raw)
                                          ActionModule/JointCommander ◀───────────────────┘
                                            merges legs[0:12]+armIK[13:26]  ⚠ NO transform/remap
                                                              │ 27-vec
 MuJoCo/robot ◀──rt/lowcmd── Bridge/set_joints ◀─ROS2 /BridgeModule/joint_set
                              ⚠ flat kp=50/kd=1, MotionSwitcher release (no sim svc)
```

⚠ = the two breakages that stop this from balancing (see §9). MovementModule
itself is correct; the breakages are in the locked consumer + the locked
command writer.

---

## 9. Mismatches: logged contract vs. actual code

| # | Session-log claim | Reality in code | Impact |
|---|---|---|---|
| M1 | BridgeModule may be a high-level LocoClient bridge | It's the **low-level** `rt/lowstate`/`rt/lowcmd` SDK bridge, interface from `argv` | **Good** — bridge work largely done; just can't host it in sim (camera/hands/MotionSwitcher) |
| M2 | "JointCommander did NOT subscribe to policy_action" | It **does** subscribe now (`conductor.py:88`) | Log stale |
| M3 | Consumer applies `action*0.25+offset+clip` + `ACTION_SDK_IDS` | **Not applied anywhere**; raw actions go to slots 0..12 with limit-clamp only | **Blocker** — must live in sim consumer; README spec for real JointCommander |
| M4 | (implicit) command writer uses policy gains | `set_joints` uses **flat kp=50/kd=1** (`config.py:36-37`); reference uses deploy.yaml stiffness/damping | **Blocker** for fidelity — sim consumer must use deploy.yaml gains |
| M5 | `joint_set` = 27-float IK-commanded **arm** pose | It's a **full-body** 27-vec; only arm slots are IK; legs are JointCommander's merge | Clarification; MovementModule already slices arm-only |
| M6 | C++ deploy + balance_metrics.py "proven working" reference + sidecar | `h1_2_ctrl` exists & prebuilt ✔; **`balance_metrics.py` does NOT exist in any repo** (referenced only in session notes) | Verdict instrument missing — see §10 OQ-1 |
| M7 | C++ controller runs the same policy | `config.yaml` runs **phase5_v3** (p6mass_D-era), not p7_1b | Must repoint to `logs/milestones/p7_1b` (onnx md5-identical to MovementModule) |
| M8 | MovementModule emits 13 raw actions | True **but** `CONN_TEST=True` makes it publish the 27-float default pose instead (`main.py:51,103-108`) | Flip to `False` (allowed edit) for the live test |

---

## 10. Open questions (resolve before/early in build)

- **OQ-1 (biggest):** `balance_metrics.py` — the designated verdict instrument
  (touchdowns / feet_dist / torso ang-vel RMS, labeled idle_quiet/trainingdist/
  push) — **does not exist in the checked-out repos.** It is named in
  `notes/sessions/2026-06-12/-14/-15`. Was it never committed, lives on another
  machine, or is it the Isaac-side headless-metrics tool? If unavailable, the
  validation step needs a small DDS sidecar built (subscribes `rt/lowstate`,
  computes the metrics) — confirm whether to build it.
- **OQ-2:** Confirm the sim consumer should **bypass** ActionModule +
  BridgeModule (writing `rt/lowcmd` itself), as the task's Phase-1 design implies
  — given both are locked and currently can't balance / can't run in sim. (My
  recommendation: yes.)
- **OQ-3:** Arm pose for the sim test — static p7_1b default (cleanest for
  idle_quiet) vs. mirror the C++ `ArmPosePublisher` Training held-pose for a
  tighter apples-to-apples. Recommend static default first, optionally match
  later.
- **OQ-4:** OK to edit `unitree_rl_lab` config (repoint `BalancePush.policy_dir`
  → `p7_1b`) and rebuild `h1_2_ctrl` for the comparison? (Not a locked module.)
- **OQ-5:** MuJoCo `config.yaml` has `enable_elastic_band: 1` (overhead support
  spring) and `use_joystick: 1`. For a free-standing balance test the band
  should likely be **off** (or at least documented), and the joystick may need
  disabling if no gamepad is attached. Confirm desired sim setup.
- **OQ-6:** Gains — confirmed the reference uses deploy.yaml stiffness/damping
  (motor order). The deploy.yaml arrays are length 35 (27 motors + 8 trailing
  zeros); confirm the articulation trims to 27 in motor order (assumed). The sim
  consumer will use the first-27 in motor order.

---

## 11. What this unblocks (hands-off to INTEGRATION_DESIGN.md)

- A thin **sim state bridge** (`rt/lowstate` → `/BridgeModule/joints_imu`,
  92-float) — replicates `get_joints_imu` minus camera/hands.
- A **static arm-pose publisher** on `/BridgeModule/joint_set`.
- A **sim action consumer** (`/MovementModule/policy_action` → transform+remap →
  `rt/lowcmd`, deploy.yaml gains) — the JointCommander+set_joints replacement for
  sim.
- MovementModule run **unchanged** except `CONN_TEST=False` (the one allowed
  edit).
- A single **launch script** (MuJoCo D-model on lo + the three nodes +
  MovementModule), mirroring `start_teleop_balance.sh`'s structure.
- A **README spec** for the real `JointCommander` (transform+remap) and the real
  `set_joints` gains fix — for the teammate.
- A **validation note** comparing the ROS2 path vs `h1_2_ctrl` on the same
  p7_1b policy + same MuJoCo.
