# Spec: production JointCommander + gains for the Architecture B balance path

For the **ActionModule / BridgeModule owner**. Delivered as a doc (not code) —
these files are not ours to edit (mirrors the 2026-06-10 consumer handoff). The
sim loop uses `arch_b_sim/sim_action_consumer.py` as the *executable reference*
for everything below; this doc says what the production code must do to match it.

## The two bugs that stop the real robot from balancing

Recon of the live chain (see `ARCH_B_RECON.md` §2, §9) found the balance path
broken in two independent places, both in modules we don't own:

**Bug 1 — the action transform + SDK remap is applied nowhere.**
`conductor._on_legs_command` caches the raw 13 policy actions
(`ActionModule/main/conductor.py:153-169`) and `JointCommander` copies
`legs_command[i]` into body slot `i` of `/BridgeModule/joint_set` with only a
joint-limit clamp (`joint_commander.py:127-197`). The policy's 13 outputs are
therefore (a) used as if they were radians (no `×0.25 + offset`, no knee clip)
and (b) routed to the wrong joints (no `ACTION_SDK_IDS` remap — e.g. action[2]
is the **torso** and must go to SDK motor 12, not body slot 2).

**Bug 2 — wrong PD gains.** `BridgeModule/setter/joint_set.py` writes `rt/lowcmd`
with flat `kp=50, kd=1` for **all** joints (`BridgeModule/config.py:36-37`). The
known-good C++ controller uses the policy's **deploy.yaml `stiffness`/`damping`**
for legs+torso (e.g. knee kp=300, hip kp=200, torso kp=300) and only 50/1 for the
arms. Flat 50/1 on the legs is far too soft for this crouch policy.

## Required transform (legs + torso) — the fix for Bug 1

Apply this to the 13 raw actions **before** they become joint targets. Import the
constants from the single source of truth — do **not** re-hardcode:

```python
# from MovementModule/main/balance_contract.py
from balance_contract import (ACTION_SCALE, ACTION_OFFSET,
                              ACTION_CLIP_LOW, ACTION_CLIP_HIGH, ACTION_SDK_IDS)
import numpy as np

def policy_action_to_sdk_targets(action13, sdk_targets27):
    a = np.clip(np.asarray(action13[:13], np.float32),
                ACTION_CLIP_LOW, ACTION_CLIP_HIGH)        # knee clip [0.45,2.5] @ slots 7,8
    legs_torso = a * ACTION_SCALE + ACTION_OFFSET         # scale 0.25 + per-joint offset
    sdk_targets27[ACTION_SDK_IDS] = legs_torso            # route action k → SDK motor ACTION_SDK_IDS[k]
    return sdk_targets27
```

`ACTION_SDK_IDS = [0,6,12,1,7,2,8,3,9,4,10,5,11]`. This equals
`joint_ids_map[LEG_TORSO_URDF_IDS]` in the C++ reference controller
(`unitree_rl_lab/.../State_RLBase.cpp`) — verified by
`arch_b_sim/selftest_transform.py` (run it; it asserts the equality against the
active policy's `deploy.yaml`).

**Where to put it:** logically it belongs where `policy_action` is ingested
(in `conductor._on_legs_command`, or in `JointCommander` just before the merge),
so that `/BridgeModule/joint_set` carries real radian targets — consistent with
the arm slots, which are already radian IK targets. MovementModule reads only the
**arm** slots of `joint_set`, so fixing the leg slots does not affect it.

Do **not** feed the leg targets back into the policy obs (it trains on the
arm-only command; legs come from its own output). Arms in `joint_set` stay as the
IK pose — unchanged.

## Required gains — the fix for Bug 2

Write `rt/lowcmd` per-motor `kp/kd` from the active policy's
`deploy.yaml stiffness/damping` (motor order, first 27) for **legs+torso**, and
`kp=50/kd=1` for the **14 arm motors** (the team's real-robot arm gains; matches
`HARNESS.md`'s `arm_kp/arm_kd`). Concretely, `set_joints` should stop using flat
`H1_2_KP/H1_2_KD` for all joints and instead load:

```
kp[0..12]  = deploy.yaml stiffness[0..12]      # 200/200/200/300/40/40 ×2, torso 300
kp[13..26] = 50.0 ;  kd[13..26] = 1.0          # arms
kd[0..12]  = deploy.yaml damping[0..12]         # 2.5/2.5/2.5/4/2/2 ×2, torso 6
```

(`arch_b_sim/sim_action_consumer.py:load_gains()` does exactly this; `--gains
deploy` uses deploy.yaml for arms too, matching the *stock* C++ controller, and
`--gains flat50` reproduces today's BridgeModule for A/B testing.)

## Sim-mode flags BridgeModule needs (separate, see BRIDGEMODULE_SIM_NOTES.md)

So BridgeModule itself can run against MuJoCo (instead of our stand-in): a
`--no-camera` path (skip RealSense check + calibration + camera/hand forks) and a
`--no-motion-switch` path (skip `MotionSwitcherClient.ReleaseMode()`, which has no
MuJoCo equivalent and blocks forever). The DDS bridge already takes the interface
from `sys.argv[1]` and is otherwise sim-ready.

## How to verify your production version matches

1. `python3 arch_b_sim/selftest_transform.py` (in the container, ROS sourced) —
   asserts your constants/remap against `deploy.yaml` + the C++ map.
2. Run your fixed stack against MuJoCo and compare `balance_metrics` (touchdowns
   / feet_dist / torso ang-vel RMS) to `arch_b_sim/run_arch_b_sim.sh --ref`
   (the C++ path) on the **same p7_1b policy + same XML**. They should balance
   equivalently; divergence is a glue bug, not the policy.
