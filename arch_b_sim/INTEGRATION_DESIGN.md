# INTEGRATION_DESIGN.md — Architecture B × MuJoCo (Phase 1)

One-page integration design. Read after `ARCH_B_RECON.md`. **Nothing built
yet** — this is the proposal to approve before Phase 2.

All new code lives in this `unitree_mujoco` branch (`arch-b-mujoco-sim`) under
`arch_b_sim/`, clearly labeled sim-only. The teammate's modules
(`BridgeModule`, `ActionModule`) are **not** touched. The only change in
`Aspired_Robot_Project` is `MovementModule` `CONN_TEST → False` (the one allowed
edit). `unitree_rl_lab` config is edited only to align the reference policy.

---

## Architecture decision: bypass the locked modules for the sim loop

The recon showed the as-built chain can't balance (missing transform+remap in
the locked `JointCommander`; wrong flat gains in the locked `set_joints`) and
can't even start in sim (`BridgeModule.run()` demands a RealSense + Inspire
hands + a MotionSwitcher service). So, per the task's Phase-1 design, the sim
loop **replaces** the locked consumer/writer with a thin sim consumer that reads
`/MovementModule/policy_action` and writes `rt/lowcmd` directly. MovementModule —
the actual subject of validation — runs unchanged. The real-robot fixes are
delivered as `JOINTCOMMANDER_SPEC.md`.

This keeps the test honest about *what it validates*: **MovementModule's
obs→ONNX→action contract + the documented action transform reproduce the C++
deploy controller's balancing.** It does **not** validate the teammate's
JointCommander/set_joints (those are specced, not exercised).

```
 MuJoCo (h1_2 D-model, lo, domain0, unitree_hg)
   │  rt/lowstate                                   rt/lowcmd  ▲
   ▼                                                           │
 [1] sim_state_bridge ──ROS2 /BridgeModule/joints_imu──▶ MovementModule ──┐
 [2] sim_armpose_pub ──ROS2 /BridgeModule/joint_set────▶ (unchanged)      │
                                                       /MovementModule/policy_action (13 raw)
                                                                          │
 [3] sim_action_consumer ◀────────────────────────────────────────────────┘
       clip→×0.25→+offset (balance_contract) ; remap ACTION_SDK_IDS→motors 0..12
       arms 13..26 ← static commanded pose ; per-motor kp/kd ← deploy.yaml ; →rt/lowcmd
```

---

## Component 1 — `sim_state_bridge.py`  (DDS→ROS2, sim-only)

- Subscribe `rt/lowstate` (`unitree_hg LowState_`) on `ChannelFactoryInitialize(0, "lo")`.
- Build the **exact 92-float record** from `get_joints_imu.py:36-47`
  (`q27,dq27,tau27,quat4(wxyz),gyro3,acc3,ts1`) — same order, so the contract
  slices line up byte-for-byte.
- Publish ROS2 `Float32MultiArray` on `/BridgeModule/joints_imu` (via the
  project's `Ros2_connector`, or a plain rclpy publisher — TBD by what's
  importable in the chosen runtime; see "Runtime" below).
- Thin: ~40 lines, no camera, no hands, no MotionSwitcher.

## Component 2 — `sim_armpose_pub.py`  (static arm command, sim-only)

- Publish a constant 27-float `Float32MultiArray` on `/BridgeModule/joint_set`
  at ~50 Hz. Arm slots (`ARM_SDK_ORDER` indices) hold the chosen arm pose;
  leg/torso slots are arbitrary (MovementModule ignores them).
- v1 pose: p7_1b default arms (zeros) → `arm_pose_command` = default arm angles,
  matching `offline_check.py` Case 1. Later: optionally match the C++
  `ArmPosePublisher` Training held-pose for a tighter comparison (OQ-3).
- May be merged into Component 1 (one node, two publishers) to reduce moving
  parts.

## Component 3 — `sim_action_consumer.py`  (the JointCommander+set_joints stand-in)

The core deliverable. Per `rt/lowstate` tick (or fixed 50 Hz, matching
`step_dt`):

1. Read latest `/MovementModule/policy_action` (13 raw). If none yet, hold the
   default pose.
2. **Transform (imported from `balance_contract`, not re-hardcoded):**
   `a = clip(action, ACTION_CLIP_LOW, ACTION_CLIP_HIGH)` then
   `target_sdk[ACTION_SDK_IDS[k]] = a[k]*ACTION_SCALE + ACTION_OFFSET[k]` for
   k=0..12. (Knee clip `[0.45,2.5]` falls out of the imported CLIP arrays.)
3. Arms: set `target_sdk[ARM_SDK_ORDER] =` the static commanded arm pose
   (same source as Component 2), so legs/torso come from the policy and arms
   from the command — never feed our own leg output back.
4. Write `rt/lowcmd` (`unitree_hg LowCmd_`) on `lo`: per motor
   `q=target_sdk[i], dq=0, tau=0, kp=stiffness[i], kd=damping[i]` where
   `stiffness/damping` come from the **active milestone's `params/deploy.yaml`**
   (motor order, first 27) — the same gains the C++ `enter()` applies. CRC +
   `mode_machine` set as in `joint_set.py:122-131` (read `mode_machine` from an
   `rt/lowstate` sub; **no** MotionSwitcher call — MuJoCo needs none).
5. Optional gentle slew limit (as `set_joints` does) to avoid a step on the
   first command; off by default for a clean comparison with the C++ path
   (which writes targets directly).

**Single source of truth:** import `ACTION_SCALE, ACTION_OFFSET,
ACTION_CLIP_LOW/HIGH, ACTION_SDK_IDS, ARM_SDK_ORDER, DEFAULT_JOINT_POS` from
`MovementModule/main/balance_contract.py`; load gains from the milestone
deploy.yaml. Assert lengths at startup.

## Component 4 — launch script `run_arch_b_sim.sh`

Mirrors `start_teleop_balance.sh`'s structure (tabs/sections), but local + sim:
1. MuJoCo: `simulate/build/unitree_mujoco` with the D-model on `lo`, domain 0,
   elastic-band/joystick per OQ-5.
2. `sim_state_bridge` + `sim_armpose_pub` (state & arm-cmd flowing).
3. `MovementModule` via its `st.sh --one` (or directly, depending on Runtime),
   `CONN_TEST=False`.
4. `sim_action_consumer` (closes the loop).
5. (Reference mode) instead of 2–4: `h1_2_ctrl --network lo` for the C++ path.

## Component 5 — `JOINTCOMMANDER_SPEC.md` (handoff, no code)

Written spec for the teammate's **real** `JointCommander`: apply
`clip→×0.25→+offset` and the `ACTION_SDK_IDS` remap before publishing
`joint_set` (or before `rt/lowcmd`), and the **`set_joints` gains fix** (use
deploy.yaml stiffness/damping, not flat 50/1). Mirrors the 2026-06-10 precedent
(consumer change handed over as doc, not code).

## Component 6 — validation note

Run the ROS2 path and `h1_2_ctrl` on the **same p7_1b** (md5-identical onnx) +
**same MuJoCo D-model**, in labeled modes (idle_quiet / trainingdist / push),
and compare with the metrics sidecar (OQ-1). Divergence ⇒ glue bug, not policy.

---

## Runtime question (needs a decision in build)

MovementModule expects ROS2 Humble + its venv inside the `ros2-humble-dev`
docker (`st.sh`), and imports `Ros2_connector`/`Debug_tool` from `.global`. The
sim nodes need both **ROS2** (to talk to MovementModule) and **`unitree_sdk2py`
+ CycloneDDS** (to talk to MuJoCo). Two viable runtimes:
- **(A)** Run the sim nodes inside the same `ros2-humble-dev` container (so ROS2
  + the connector are available), and `pip install unitree_sdk2py` there
  (`--add-pip`). MuJoCo runs on the host; DDS crosses via `--network host` + lo.
- **(B)** Run the sim nodes on the host in a venv that has rclpy +
  unitree_sdk2py, sharing lo + the ROS2 graph with the container
  (`--network host`, matching `RMW`/`ROS_DOMAIN_ID`).

Recommend **(A)** for fewer DDS/RMW mismatches (everything ROS2 in one place;
unitree DDS is domain-0 CycloneDDS on lo regardless). Confirm in Phase 2 step 1.

---

## Build order (Phase 2, each step independently verifiable)

1. **MuJoCo up + lowstate flowing** — launch sim; prove `rt/lowstate` on lo
   (echo via a tiny `unitree_sdk2py` subscriber, or `h1_2_ctrl --network lo`
   connects). Smoke test.
2. **State→ROS2** — `sim_state_bridge`; `ros2 topic echo /BridgeModule/joints_imu`;
   at default pose assert `joint_pos_rel≈0`, `projected_gravity≈[0,0,-1]`
   (reuse `offline_check.py` math + a live frame).
3. **MovementModule infers** — with joints_imu + static joint_set, confirm 13
   actions on `/MovementModule/policy_action`; `offline_check.py` green.
4. **Consumer closes loop** — `sim_action_consumer`; robot balances in MuJoCo
   on the Architecture B path.
5. **Validate vs C++** — `h1_2_ctrl --network lo` on the same p7_1b + same
   MuJoCo; compare metrics in labeled modes.

---

## Risks / how each is caught early

- **Joint-order off-by-one** → step 2 known-pose cross-check (`joint_pos_rel≈0`)
  before any policy runs.
- **Gain mismatch** → step 5 divergence from C++ path; gains asserted against
  deploy.yaml at consumer startup.
- **Quat convention** → `offline_check.py` Case 4 + live `projected_gravity`
  print (`main.py:90-94`).
- **Wrong message family (go vs hg)** → eliminated by using C++ `simulate/`
  (motor-count dispatch → G1Bridge/unitree_hg).
- **Different policy in C++ vs ROS2** → repoint to `logs/milestones/p7_1b`
  (md5-verified identical onnx) before comparing.

---

# ADDENDUM — decisions after Phase-1 review (2026-06-15)

User feedback resolved the open questions and corrected one design assumption.
This addendum supersedes the static-arm-only framing above where they conflict.

## A1. ActionModule STAYS in the loop — the integration is the point

Correction: we bypass **only BridgeModule** (the camera-bound process) and the
motor-write — **not ActionModule**. The whole point is *our balance legs/torso +
the colleague's IK-resolved arms*. ActionModule is pure ROS2 (no camera, no DDS,
`ActionModule/main/main.py` confirms), so it runs in sim as-is. Two run modes:

- **Mode A — balance bring-up (no ActionModule).** `sim_state_bridge` +
  `sim_armpose_pub` (static default arm pose) + MovementModule + consumer.
  Tests legs/balance in isolation. Phase-2 steps 1–4.
- **Mode B — full integration (the goal).** `sim_state_bridge` +
  **colleague's ActionModule** (real IK arms) + MovementModule + consumer.
  No `sim_armpose_pub` (ActionModule owns `/BridgeModule/joint_set`).

**How the arm command reaches us (verified path):** ActionModule's
`JointCommander` publishes the full-body 27-vec on `/BridgeModule/joint_set`;
its **arm slots (SDK 13..26)** are the IK-resolved arm pose
(`joint_commander.py:106,184`). Both MovementModule (for the `arm_pose_command`
obs, via `ARM_SDK_ORDER`) and our consumer (for the arm **motor targets**) read
**only** those arm slots. Leg/torso slots of `joint_set` are ignored by both —
legs come from the policy. So no feedback loop.

**What ActionModule needs to run without BridgeModule** (so `sim_state_bridge`
must provide it, by impersonating BridgeModule on the ROS2 side):
1. `/BridgeModule/conduct` latched `True` — ActionModule blocks on this
   handshake (`ActionModule/main/main.py:52-53`). `sim_state_bridge` announces
   it (node name `BridgeModule`).
2. `/BridgeModule/joints_imu` — IK seed / current-body
   (`ActionModule/main/main.py:64`). Provided.
3. A sequence trigger: ActionModule auto-runs the `start` sequence on boot
   (`main.py:74`); arm motion otherwise comes from
   `ros2 topic pub --once /ActionModule/run std_msgs/String "data: <seq>"`.
4. MoveIt IK backend (bundled under `ActionModule/.moveit/`, no RvizModule
   dependency) — present in the colleague's `ros2-humble-dev` image.

## A2. Gains — three regimes; default to the sim2real harness config

`HARNESS.md` (aspired/deploy_mujoco_harness) documents the *intended* sim2real
gain split, which differs from both the stock controller and the real Bridge:

| regime | legs+torso | arms | where |
|---|---|---|---|
| stock C++ controller | deploy.yaml stiffness/damping | deploy.yaml (100/50…) | `State_RLBase::enter()` |
| **harness (sim2real, default)** | **deploy.yaml stiffness/damping** | **kp=50 / kd=1** | harness `config.yaml` `arm_kp/arm_kd` |
| real robot (BridgeModule) | flat 50/1 (**bug for legs**) | flat 50/1 | `BridgeModule/config.py` |

→ The sim consumer's gains are **configurable**, default = harness regime:
legs/torso from the milestone `deploy.yaml`, arms `kp=50/kd=1`. The validation
compares against the harness controller on the same regime.

## A3. Metrics — found; integrate headless

`balance_metrics.py` is at `unitree_rl_lab` `aspired/deploy:.../tools/`. It's a
self-contained `rt/lowstate` sidecar (MuJoCo-FK touchdowns/feet_dist/torso
ang-vel RMS/falls; `--iface lo --domain 0 --xml <auto>`; stdin
`zero|mode|note|quit`; **RUN SUMMARY on SIGINT/exit**; tolerant of a closed
stdin when backgrounded). Plan: vendor a copy under `arch_b_sim/tools/` (with a
provenance header), and have `run_arch_b_sim.sh` start it **in the background**
with stdout→`arch_b_sim/logs/balance_metrics_<ts>.log`, print that path on
start, and on launcher exit `SIGINT` it so the summary lands in the log, then
print the path again. A FIFO exposes its stdin so `mode <label>` / `zero` can be
sent without a second console. Works identically for the ROS2 path and the C++
reference path.

## A4. MuJoCo support — non-issue

Keep `use_joystick: 1` (joystick drives push/command tests — no alternative) and
leave `enable_elastic_band: 1` (toggled at runtime in the sim window). Same
setting for both paths. No `config.yaml` edit needed for these.

## A5. Reference path — reuse the colleague's harness, don't rebuild it

The C++ sim2sim reference already exists on `aspired/deploy_mujoco_harness`
(controller + metrics + pushes + arm modes). For the comparison we (a) align its
policy to `p7_1b` (md5-identical onnx) and (b) run its metrics headless via our
launcher. Switching the shared `unitree_rl_lab` checkout to that branch +
rebuilding `h1_2_ctrl` is a coordination step to confirm before doing (it's the
colleague's deploy repo, currently on `aspired/training`).

## A6. Runtime placement (resolved)

`unitree_mujoco` (C++ sim, needs X/display) runs on the **host**. The Python sim
nodes run in the `ros2-humble-dev` container (so ROS2 + the project's
`Ros2_connector` are available) launched by `run_arch_b_sim.sh` with **both**
repos mounted (`-v Aspired:/workspace -v unitree_mujoco:/unitree_mujoco`),
`--network host --ipc=host` so host-MuJoCo DDS (CycloneDDS, lo, domain 0) and the
container share the bus. `unitree_sdk2py` is pip-installed into a small harness
venv. MovementModule launches via its own `st.sh --one` in the same container.
