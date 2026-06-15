# VALIDATION_NOTE.md — Architecture B × MuJoCo: what's verified, what's left

Bottom-up, per the verification principles (verify each layer before the next).
"This box" = the headless dev environment where the recon was done; the GUI
MuJoCo can't open a window here (`libdecor-gtk`/Wayland init fails → the sim
exits), so **physics validation is a desktop task**. Everything *except physics*
is verified here.

## ✅ Verified on this box

| Layer | How | Result |
|---|---|---|
| **Contract / obs / ONNX** | `MovementModule/Utils/offline_check.py` in the ros2 container | ALL PASS — obs width 87, proj_grav/joint_pos_rel/arm_cmd correct, ONNX 87→13, calm home action (max 0.456) |
| **Action transform + SDK remap** | `arch_b_sim/selftest_transform.py` (container, ROS python) | ALL PASS — `ACTION_SDK_IDS == joint_ids_map[LEG_TORSO_URDF_IDS]` (the C++ map) read live from `deploy.yaml`; scale/offset/clip == deploy.yaml; knee clip [0.45,2.5]; slot coverage {0..26} disjoint |
| **DDS on `lo`** | `tools/fake_lowstate_pub.py` → subscriber, `tools/cyclonedds_lo.xml` | rt/lowstate flows; unicast loopback discovery works (multicast disabled) |
| **Full Mode-A data-flow (no physics)** | `tools/chain_selftest.sh` (one container, fake robot) | CHAIN OK — `/BridgeModule/joints_imu` → MovementModule → `/MovementModule/policy_action` (13) → consumer → `rt/lowcmd`; knee target 0.713 (=0.45 floor crouch), kp[knee]=300, kp[arm]=50 |

What this proves: the obs is built right, the policy infers live, the consumer
applies the *correct* transform + remap + gains and writes `rt/lowcmd` — i.e. the
entire Architecture B plumbing is wired correctly. The only thing the fake robot
can't show is whether the robot **balances** (it doesn't respond to `rt/lowcmd`).

## ⏳ Left for the desktop (needs the GUI MuJoCo + physics)

These are Phase-2 step 4 (closed-loop balance) and step 5 (C++ comparison), plus
Mode B (colleague's IK arms). Run on the machine with a working display:

```bash
cd ~/Projects/robot_projects/repos/unitree_mujoco/arch_b_sim

# A) balance bring-up: our legs/torso + a STATIC default arm pose
bash run_arch_b_sim.sh --mode-a
#   → MuJoCo window opens; disable the elastic band in the sim for free-standing.
#   → expect the robot to hold the p7_1b crouch. Watch balance_metrics log path
#     printed at startup (touchdowns ~0, torso ang-vel RMS low, feet_dist steady).

# B) FULL integration: our legs/torso + the colleague's ActionModule IK arms
bash run_arch_b_sim.sh --mode-b
#   → ActionModule publishes /BridgeModule/joint_set arm slots (real IK). Trigger
#     arm motion: ros2 topic pub --once /ActionModule/run std_msgs/String "data: <seq>"
#   → needs ActionModule's MoveIt deps in the image (the colleague's normal setup).

# C) reference: the known-good C++ controller on the SAME policy + same MuJoCo
#   First align the reference policy to p7_1b (one-time, in unitree_rl_lab):
#     set FSM.BalancePush.policy_dir → ../../../logs/milestones/p7_1b  in
#     deploy/robots/h1_2/config/config.yaml, then rebuild h1_2_ctrl.
#   (p7_1b onnx is md5-identical to MovementModule's, so this is a true A/B.)
bash run_arch_b_sim.sh --ref
#   → drive R3 FSM: FixStand → Balance. Compare its balance_metrics log to (A).
```

**Verdict rule:** the ROS2 path (A/B) and the C++ path (C) should balance
**equivalently** on the same p7_1b + same XML, in labeled modes
(`echo "mode idle_quiet|trainingdist|push" > /tmp/archb_metrics.stdin`). If they
diverge, it's a bug in our glue, not the policy — the most valuable single check.

### Known-pose cross-check to run first (catches a joint-order off-by-one)
With MuJoCo standing at the default pose, `ros2 topic echo --once
/BridgeModule/joints_imu` and confirm (via `offline_check` math) `joint_pos_rel
≈ 0` and `projected_gravity ≈ [0,0,-1]`. MovementModule already prints
`projected_gravity` once at startup (`main.py:90-94`) — it should read ~[0,0,-1].

## Caveats / risks still open

- **Joystick:** `simulate/config.yaml` has `use_joystick:1`; with no gamepad the
  launcher warns. If the sim refuses to start, set `use_joystick:0` (push/command
  tests then need a pad, per your note).
- **Mode B not run here** — the data path is verified wired (ActionModule is pure
  ROS2, handshake via our `sim_state_bridge`'s `/BridgeModule/conduct`), but
  bringing up ActionModule + MoveIt is a desktop step.
- **Gains regime** defaults to `harness` (legs deploy.yaml, arms 50/1). Use
  `--gains deploy` to match the *stock* C++ controller, or `--gains flat50` to
  reproduce today's BridgeModule. Pick the regime that matches the `--ref`
  controller you compare against.
- **`balance_metrics.py` is vendored** from `unitree_rl_lab` `aspired/deploy`
  (`tools/balance_metrics.py`, verbatim) so its metric definitions stay identical
  to the colleague's harness; the launcher runs it headless → logfile, SUMMARY on
  exit. Its `find_default_xml()` would point off in this new location, so the
  launcher always passes `--xml` explicitly.
