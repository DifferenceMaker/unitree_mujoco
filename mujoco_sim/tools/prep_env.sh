#!/usr/bin/env bash
# prep_env.sh — one-shot environment prep for the ARCH B sim stack.
#
# Mirrors the Aspired repo's OFFICIAL setup chain (.setup/st.sh) but STOPS
# before module startup: on this host .docker/cyclonedds.xml pins the ROBOT
# LAN (192.168.123.222), so a plain `st.sh` run would launch the real module
# stack onto the robot bus (2026-07-03 incident class). This script only:
#   1. builds the ros2-humble-dev image (docker_setup.sh, --clean = --no-cache)
#   2. creates the per-module venvs (.venv/<module>) from requirements.txt
#   3. vendors git dependencies (.dependencies/) into the venvs
# No ROS node is ever started.
#
#   bash prep_env.sh            # ensure image + venvs (cheap when present)
#   bash prep_env.sh --clean    # full rebuild (parity with `st.sh --clean`)
#
# NB: python_setup.sh's ensure_venv SKIPS venvs that already exist — it does
# not reconcile them against requirements.txt. After a requirements change
# (e.g. 2026-07-14: ActionModule grew pin+casadi for the Pinocchio IK), delete
# the module's venv first:  sudo rm -rf <Aspired>/.venv/ActionModule
set -euo pipefail

REPOS="${REPOS:-$HOME/Projects/robot_projects/repos}"
ASPIRED="$REPOS/Aspired_Robot_Project"

CLEAN_FLAG=""
[ "${1:-}" = "--clean" ] && CLEAN_FLAG="--clean"

echo ">>> [prep 1/2] docker image (ros2-humble-dev) ${CLEAN_FLAG:+— --no-cache rebuild}"
( cd "$ASPIRED" && bash .setup/setup_scripts/docker_setup.sh $CLEAN_FLAG )

# Venvs for the modules.sh roster (BridgeModule/RvizModule/VisualModule/
# ActionModule) + MovementModule, which is NOT in the roster — it is launched
# with `--one MovementModule` from this PC, so `--all` never creates its venv.
echo ">>> [prep 2/2] module venvs + vendored dependencies (no ROS nodes started)"
docker run --rm -v "$ASPIRED:/workspace" -w /workspace \
  --entrypoint bash ros2-humble-dev -c '
    set -e
    bash .setup/setup_scripts/python_setup.sh --all
    bash .setup/setup_scripts/python_setup.sh --one MovementModule
    bash .setup/setup_scripts/dependencies_setup.sh --all
  '

echo ">>> prep OK — venvs: $(ls "$ASPIRED/.venv" 2>/dev/null | tr '\n' ' ')"
