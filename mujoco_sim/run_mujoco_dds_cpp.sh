#!/usr/bin/env bash
# run_mujoco_dds_cpp.sh — test ANY balance/walk milestone in the dds_cpp (h1_2_ctrl)
# reference rig WITHOUT touching the hardware config.
#
#   bash run_mujoco_dds_cpp.sh --policy <milestone-slug> [--slot Balance_orient]
#
# h1_2_ctrl reads <bin>/../config/config.yaml (fixed, relative to the binary) and
# resolves relative policy_dir entries against that project dir. So: build a SHADOW
# project dir in /tmp with a COPY of the binary (RUNPATH is absolute -> libs resolve)
# and a COPY of config.yaml whose chosen slot's policy_dir points (absolute) at the
# requested milestone; then run the `ref` profile with CTRL_BIN overridden. The real
# deploy/robots/h1_2/config/config.yaml is never modified.
set -euo pipefail
POLICY=""; SLOT="Balance_orient"
while [[ $# -gt 0 ]]; do case "$1" in --policy) POLICY="$2"; shift 2;; --slot) SLOT="$2"; shift 2;; *) echo "unknown arg $1"; exit 2;; esac; done
[[ -n "$POLICY" ]] || { echo "usage: $0 --policy <slug> [--slot Balance_orient]"; exit 2; }
REPOS="${REPOS:-$HOME/Projects/robot_projects/repos}"
RLLAB="$REPOS/unitree_rl_lab"; PROJ="$RLLAB/deploy/robots/h1_2"
for root in "$RLLAB/logs/milestones" "$REPOS/aspired-isaac-lab/milestone_checkpoints"; do
  [[ -d "$root/$POLICY" ]] && MS="$root/$POLICY" && break
  m=$(ls -dt "$root"/*"$POLICY"* 2>/dev/null | head -1); [[ -n "$m" ]] && MS="$m" && break
done
[[ -n "${MS:-}" ]] || { echo "FATAL: no milestone matches '$POLICY'"; exit 2; }
for f in exported/policy.onnx params/deploy.yaml; do [[ -f "$MS/$f" ]] || { echo "FATAL: $MS missing $f"; exit 2; }; done
SHADOW="/tmp/fdk_dds/$(basename "$MS")"; rm -rf "$SHADOW"; mkdir -p "$SHADOW/build" "$SHADOW/config"
cp "$PROJ/build/h1_2_ctrl" "$SHADOW/build/h1_2_ctrl"
# point ONLY the chosen slot's policy_dir at the milestone (absolute path)
# chosen slot -> the milestone (absolute); EVERY OTHER slot's relative policy_dir is
# absolutized against the real project dir — h1_2_ctrl resolves them against the
# binary's location, which in the /tmp shadow points at nothing (2026-08-27 crash:
# State_Balance_orient iterated /tmp/fdk_dds/<slug>/../../../logs/... and aborted).
awk -v slot="$SLOT" -v ms="$MS" -v proj="$PROJ" '
  /^  [A-Za-z_]+:/ { in_slot = ($1 == slot":") }
  in_slot && /^    policy_dir:/ { sub(/policy_dir:.*/, "policy_dir: " ms "   # fleetdeck dds_cpp test"); }
  !in_slot && /^    policy_dir:[ ]*[^\/]/ { sub(/policy_dir:[ ]*/, "policy_dir: " proj "/"); }
  { print }' "$PROJ/config/config.yaml" > "$SHADOW/config/config.yaml"
grep -n "fleetdeck dds_cpp test" "$SHADOW/config/config.yaml" >/dev/null || { echo "FATAL: slot '$SLOT' has no policy_dir line in config.yaml"; exit 2; }
echo ">>> [dds_cpp] shadow project $SHADOW"
echo ">>> [dds_cpp] slot $SLOT -> $MS  (hardware config untouched)"
echo ">>> [dds_cpp] gamepad: LT+up = FixStand, then the $SLOT combo (see config.yaml) engages the policy"
export CTRL_BIN="$SHADOW/build/h1_2_ctrl"
bash "$(dirname "$0")/run_mujoco_sim.sh" ref   # no exec: the launcher process stays alive for fleetdeck's status check
