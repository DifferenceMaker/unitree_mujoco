# arch_b_sim — Architecture B balance stack × unitree_mujoco (sim-only)

Sim-only scaffolding to run the ROS2 balance pipeline (MovementModule) against
`unitree_mujoco` as a stand-in for the H1-2, **without editing the teammate's
locked modules** (BridgeModule, ActionModule). Lives on the `arch-b-mujoco-sim`
branch of `unitree_mujoco`.

## Read in this order
1. **ARCH_B_RECON.md** — what's actually implemented vs the logged contract (Phase 0).
2. **INTEGRATION_DESIGN.md** (+ Addendum) — the integration design + decisions (Phase 1).
3. **VALIDATION_NOTE.md** — what's verified + the desktop runbook (Phase 2).
4. **JOINTCOMMANDER_SPEC.md** — handoff spec for the teammate's production fix.
5. **BRIDGEMODULE_SIM_NOTES.md** — why BridgeModule can't run in sim as-is (for its owner).

## Run
```bash
bash run_arch_b_sim.sh --mode-a    # our legs/torso + static default arms
bash run_arch_b_sim.sh --mode-b    # our legs/torso + colleague's ActionModule IK arms
bash run_arch_b_sim.sh --ref       # known-good C++ controller (comparison baseline)
```

## Files
| file | role |
|---|---|
| `sim_state_bridge.py` | rt/lowstate → `/BridgeModule/joints_imu` (92-float) + `/BridgeModule/conduct` |
| `sim_action_consumer.py` | `/MovementModule/policy_action` → transform+remap → `rt/lowcmd` (deploy.yaml + 50/1 arm gains) |
| `sim_armpose_pub.py` | static `/BridgeModule/joint_set` (Mode A only) |
| `selftest_transform.py` | offline assert of the transform/remap vs deploy.yaml + the C++ map |
| `run_arch_b_sim.sh` | host orchestrator: MuJoCo + headless metrics + ROS2 stack |
| `tools/_nodes_in_container.sh` | container-side node bring-up (Mode A/B) |
| `tools/chain_selftest.sh` | full Mode-A data-flow test against a fake robot (no physics/GUI) |
| `tools/fake_lowstate_pub.py` | MuJoCo stand-in: publishes rt/lowstate at the default pose |
| `tools/balance_metrics.py` | verdict sidecar (vendored from unitree_rl_lab `aspired/deploy`) |
| `tools/cyclonedds_lo.xml` | loopback-only DDS config for headless testing |

## Verified (headless): contract (offline_check) · transform (selftest) · DDS-on-lo · full Mode-A data-flow (chain_selftest). Physics/balance + C++ comparison are desktop tasks — see VALIDATION_NOTE.md.

The one edit outside this dir: `MovementModule/main/main.py` `CONN_TEST` is now
env-overridable and defaults to publishing real policy actions (was a hardcoded
default-pose stub). MovementModule is ours to edit; the teammate's modules are untouched.
