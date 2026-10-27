#!/usr/bin/env bash
# run_mujoco_homing.sh -- HOMING rig (2026-10-27): the loco-switch sim2sim rig with the homing commander instead of the keyboard.
#   bash mujoco_sim/run_mujoco_homing.sh                 # sim (desk scene) + Bridge + MovementModule [walk active, balance shadow] + homing_commander
#   bash mujoco_sim/run_mujoco_homing.sh stop
#   then:  bash mujoco_sim/tools/homing_trials.sh --n 10          # the 10-trial experiment (teleport -> goto -> arrival -> switch -> settle)
#   or by hand:  echo "goto 0.35 0 0" >> mujoco_sim/logs/.homing_cmd     (table edge centre x y + approach heading deg, world frame)
#                echo "teleport -4 0.5 20" > mujoco_sim/logs/.sim_stdin  (sim: place the base at x y yaw_deg, velocities zeroed)
# Policies: HOMING_WALK (default lm7f_kit_footbonus6_still, vendored in the homing worktree), HOMING_BALANCE (default p14h_stancewide);
# code = the MM_loco_switch_homing worktree (ASPIRED). Scene: the desk scene -- desk body at (0.75, 0, 0), top 0.80 x 1.20 m,
# so the NEAR EDGE CENTRE is (0.35, 0) and the approach heading is 0 deg (+x). Knobs: HOMING_VMAX (0.5), HOMING_STOP_DIST (0.35 torso
# front -> edge), HOMING_TORSO_FRONT (0.15), HOMING_POSE_NOISE_XY (0.01 m), HOMING_POSE_NOISE_YAW (0.1 deg), HOMING_POSE_BIAS_XY (0).
set -uo pipefail
REPOS="${REPOS:-$HOME/Projects/robot_projects/repos}"
export ASPIRED="${ASPIRED:-$REPOS/Aspired_Robot_Project-homing}"
export LOCOSW_MODE=h HOMING_POSE="${HOMING_POSE:-sim}"
# the module venvs live in the main checkout (gitignored): mount them into the worktree's /workspace/.venv
export ASPIRED_VENV="${ASPIRED_VENV:-$REPOS/Aspired_Robot_Project/.venv}"
SIM="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "stop" ]]; then exec bash "$SIM/run_mujoco_loco_switch.sh" stop; fi
rm -f "$SIM/logs/.homing_cmd"; : > "$SIM/logs/.homing_cmd"; touch "$SIM/logs/homing_trials.jsonl"   # host-owned: the container (root) must not create them
exec bash "$SIM/run_mujoco_loco_switch.sh" --desk --walk "${HOMING_WALK:-lm7f_kit_footbonus6_still}" --balance "${HOMING_BALANCE:-p14h_stancewide}" "$@"
