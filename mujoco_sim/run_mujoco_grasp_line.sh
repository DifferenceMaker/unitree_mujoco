#!/usr/bin/env bash
# ============================================================================
# run_mujoco_grasp_line.sh — GRASP RIG: a gr-line RIGHT-hand grasp policy tested
# THROUGH the arch-B stack in MuJoCo (2026-08-28).
#
#   bash run_mujoco_grasp_line.sh --policy gr7b_clean_smooth [--object cube] [--gap 0.03]
#   bash run_mujoco_grasp_line.sh stop
#
# What runs (torso + RIGHT arm + hand, pelvis WELDED, no balance policy):
#   sim        unitree_mujoco on scene_comx06_grasp_right.xml — built per launch by
#              aspired-isaac-lab/scripts/tools/build_grasp_scene.py from the policy's
#              env.yaml (trained arm pose -> palm FK -> table + object placement);
#              the right Inspire hand as built (6 force-limited position servos +
#              mechanically coupled followers); ARCHB_GRASP=1 turns on grasp_sim.h:
#              finger servos from rt/sim_hand/cmd, hand/object state to
#              rt/sim_hand/state (+ the .grasp_sim_state file mirror)
#   container  REAL BridgeModule (BRIDGE_SIM=1 + BRIDGE_SIM_HANDS=R: its real
#              finger_manager talks Modbus-TCP to the Inspire EMULATOR at 127.0.0.1)
#              + inspire_sim_emu.py + object_pose_relay.py (VisualModule stand-in:
#              cube in the torso frame, 1-2 s random hold + noise) + REAL ActionModule
#              (GraspPolicy runner -> arm/hand override -> JointCommander -> Bridge),
#              sequence cube_task.rl_grasp auto-dispatched after ARCHB_GRASP_SEQ_DELAY s.
#
# GATE: only RIGHT-hand Arm7Table CUBE policies (params/env.yaml: asset
# inspire_hand_arm7_right, obs pad_forces/joint_pos/last_action/arm_joint_pos/
# cube_pose, right_* arm action, 0.09x0.06x0.055 cuboid) — a left-hand or tube/ring
# policy is refused here AND by GraspPolicy itself (operator rule 2026-08-28).
#
# ISOLATION as the other rigs: ROS2 domain 77 + ROS_LOCALHOST_ONLY, unitree-SDK DDS
# domain 1 on lo. Modbus emulator on 127.0.0.1:6000 (inspire_sdkpy hardcodes 6000).
# ============================================================================
set -uo pipefail

REPOS="${REPOS:-$HOME/Projects/robot_projects/repos}"
ASPIRED="$REPOS/Aspired_Robot_Project"
MUJOCO="$REPOS/unitree_mujoco"
SDK="$REPOS/unitree_sdk2_python"
RLLAB="$REPOS/unitree_rl_lab"
ISAAC="$REPOS/aspired-isaac-lab"
SIM="$MUJOCO/mujoco_sim"
MJ_BIN="$MUJOCO/simulate/build/unitree_mujoco"
TV_PY="${TV_PY:-$HOME/miniconda3/envs/tv/bin/python}"
SCENE_NAME="scene_comx06_grasp_right.xml"
SCENE_XML="$MUJOCO/unitree_robots/h1_2/$SCENE_NAME"
SIM_DDS_DOMAIN=1

POLICY=""; OBJECT="auto"; GAP="0.03"; RECORD=0; RECORD_FILE_OPT=""; DEBUG=1; PROFILE="run"
VISION_HOLD_MIN="${VISION_HOLD_MIN:-1.0}"; VISION_HOLD_MAX="${VISION_HOLD_MAX:-2.0}"; SEQ_DELAY="${ARCHB_GRASP_SEQ_DELAY:-3}"
while [[ $# -gt 0 ]]; do case "$1" in
  stop)           PROFILE="stop"; shift;;
  --policy)       POLICY="$2"; shift 2;;
  --object)       OBJECT="$2"; shift 2;;          # cube (gated) | tube (asset staged, NOT gated yet)
  --gap)          GAP="$2"; shift 2;;             # palm-pad to object-top gap at spawn [m]
  --vision-hold)  VISION_HOLD_MIN="${2%,*}"; VISION_HOLD_MAX="${2#*,}"; shift 2;;   # "min,max" seconds
  --seq-delay)    SEQ_DELAY="$2"; shift 2;;
  --record)       RECORD=1; shift;;
  --record-file)  RECORD=1; RECORD_FILE_OPT="$2"; shift 2;;
  --quiet)        DEBUG=0; shift;;
  -h|--help) sed -n '2,32p' "$0"; exit 0;;
  *) echo "unknown arg: $1"; exit 1;;
esac; done

LOG_DIR="$SIM/logs"; mkdir -p "$LOG_DIR"
STAMP=$(date +%Y%m%d_%H%M%S)
MJ_LOG="$LOG_DIR/mujoco_grasp_$STAMP.log"
GRASP_FILE="$SIM/logs/.grasp_sim_state"
STAGE="$SIM/logs/.grasp_policy_stage"
RECORD_FILE="${RECORD_FILE_OPT:-$LOG_DIR/mujoco_grasp_rec_$STAMP.mp4}"

sweep_leftovers() {
  local strays; strays=$(docker ps -q --filter "name=archb_sim_")
  if [[ -n "$strays" ]]; then
    echo ">>> [sweep] removing archb containers: $(docker ps --format '{{.Names}}' --filter 'name=archb_sim_' | tr '\n' ' ')"
    docker rm -f $strays >/dev/null 2>&1
  fi
  pkill -f "$MJ_BIN" 2>/dev/null && echo ">>> [sweep] killed a leftover unitree_mujoco sim"
  rm -f "$GRASP_FILE" "$GRASP_FILE.tmp" "$SIM/logs/.grasp_place_object"
}
if [[ "$PROFILE" == "stop" ]]; then
  echo ">>> STOP: tearing down any running/leftover sim stack..."; sweep_leftovers; echo ">>> done — environment clean."; exit 0
fi
[[ -n "$POLICY" ]] || { echo "FATAL: --policy <milestone> is required"; exit 2; }
[[ -x "$MJ_BIN" ]] || { echo "FATAL: sim binary missing: $MJ_BIN (build simulate/)"; exit 2; }
[[ -x "$TV_PY" ]] || { echo "FATAL: tv python not found at $TV_PY (needed for build_grasp_scene.py)"; exit 2; }
sweep_leftovers

# ── policy resolve (same rule as the desk rig) ──────────────────────────────
MILESTONES="$RLLAB/logs/milestones"
resolve_ms() {
  [[ -d "$MILESTONES/$1" ]] && { echo "$1"; return; }
  local m
  m=$(ls -dt "$MILESTONES"/*"_${1}_"[0-9]* 2>/dev/null | head -1)
  [[ -z "$m" ]] && m=$(ls -dt "$MILESTONES"/"${1}_"[0-9]* 2>/dev/null | head -1)
  [[ -z "$m" ]] && m=$(ls -dt "$MILESTONES"/*"$1"* 2>/dev/null | head -1)
  [[ -n "$m" ]] && basename "$m"
}
MS="$(resolve_ms "$POLICY")"
if [[ -z "$MS" || ! -d "$MILESTONES/$MS" ]]; then
  MILESTONES="$ISAAC/milestone_checkpoints"; MS="$(resolve_ms "$POLICY")"
  [[ -n "$MS" ]] && echo ">>> [policy] resolved from the repo archive: $MILESTONES/$MS"
fi
[[ -n "$MS" && -d "$MILESTONES/$MS" ]] || { echo "FATAL: no milestone matches --policy '$POLICY'"; exit 2; }
for f in exported/policy.onnx params/deploy.yaml params/env.yaml; do
  [[ -f "$MILESTONES/$MS/$f" ]] || { echo "FATAL: $MS missing $f"; exit 2; }
done
MSDIR="$MILESTONES/$MS"

# ── OBJECT auto-detect (2026-09-03, tube rig): read the training object from the
# milestone's own env.yaml — the same one-button flow as the cube; --object wins.
if [[ "$OBJECT" == "auto" ]]; then
  OBJECT=$(python3 - "$MSDIR/params/env.yaml" <<'PY'
import sys, re
s = open(sys.argv[1]).read()
if "tube_d180" in s: print("tube")
elif "ring_d180" in s: print("ring")
else: print("cube")
PY
)
  echo ">>> [object] auto-detected from env.yaml: $OBJECT"
fi
# per-object container knobs: object height (hover z + table math), the palm->
# object-center forward offset (the tube trains with the palm at the NEAR RIM —
# object_xy_offset (0.09, 0)), planner collision box, hover gap default inside
# the trained band (cube U[0.02,0.08] -> 0.06; tube U[0.02,0.04] -> 0.04).
case "$OBJECT" in
  tube) OBJ_H="0.130"; OBJ_FWD="0.09"; OBJ_BOX="0.19 0.19 0.13"; HOVER_GAP_DEF="0.04";;
  ring) OBJ_H="0.020"; OBJ_FWD="0.071"; OBJ_BOX="0.20 0.20 0.02"; HOVER_GAP_DEF="0.065";;
  *)    OBJ_H="0.055"; OBJ_FWD="0";    OBJ_BOX="0.09 0.06 0.055"; HOVER_GAP_DEF="0.06";;
esac

# ── CONTRACT GATE + deploy.yaml staging (trained default pose patched in) ───
rm -rf "$STAGE"; mkdir -p "$STAGE/$MS"
python3 - "$MSDIR" "$STAGE/$MS" "$OBJECT" <<'PY' || exit 4
import sys, yaml, shutil, json, os
ms, stage, obj = sys.argv[1:4]
env = yaml.unsafe_load(open(os.path.join(ms, "params/env.yaml")))
dep = yaml.unsafe_load(open(os.path.join(ms, "params/deploy.yaml")))
bad = []
asset = str(env["scene"]["robot"]["spawn"].get("asset_path", ""))
if "inspire_hand_arm7_right" not in asset:
    bad.append(f"robot asset is not inspire_hand_arm7_right ({asset.split('/')[-1] or '?'}) — RIGHT-hand rig only")
obs = [k for k, v in env["observations"]["policy"].items() if isinstance(v, dict) and "func" in v]
want = ["pad_forces", "joint_pos", "last_action", "arm_joint_pos", "cube_pose"]
if obs != want:
    bad.append(f"obs {obs} != {want}")
acts = env["actions"]
arm = acts.get("arm") if isinstance(acts.get("arm"), dict) else None
if arm is None or "fingers" not in acts:
    bad.append("actions must be fingers + arm")
else:
    names = list(arm.get("joint_names") or [])
    if len(names) != 7 or not all(n.startswith("right_") for n in names):
        bad.append(f"arm action joints are not the 7 right_* joints: {names}")
cube = env["scene"].get("cube", {})
sp = cube.get("spawn", {}) if isinstance(cube, dict) else {}
size = tuple(round(float(v), 4) for v in (sp.get("size") or ())) if sp.get("size") else None
if obj == "cube":
    if "spawn_cuboid" not in str(sp.get("func", "")) or size != (0.09, 0.06, 0.055):
        bad.append(f"training object is not the 0.09x0.06x0.055 cuboid (func {str(sp.get('func','')).split(':')[-1]}, size {size}) — tube/ring rigs come later")
elif obj == "tube":
    if "tube_d180" not in str(sp.get("asset_path", "")):
        bad.append("--object tube but the policy did not train on tube_d180_h130")
if bad:
    print("GATE REFUSED — this launcher serves RIGHT-hand Arm7Table cube policies only:")
    for b in bad: print("   -", b)
    sys.exit(1)
# stage: onnx + deploy.yaml with the TRAINED default arm pose as actions.arm.offset
# (the exporter writes 0.0 for use_default_offset actions; GraspPolicy adds the offset)
jp = env["scene"]["robot"]["init_state"]["joint_pos"]
names = list(dep["actions"]["arm"]["joint_names"])
off = [float(jp[n]) for n in names]
dep["actions"]["arm"]["offset"] = off
shutil.copy(os.path.join(ms, "exported/policy.onnx"), os.path.join(stage, "policy.onnx"))
for f in ("overrides.json", "MILESTONE.md"):
    if os.path.exists(os.path.join(ms, f)): shutil.copy(os.path.join(ms, f), stage)
shutil.copy(os.path.join(ms, "params/env.yaml"), os.path.join(stage, "env.yaml"))
class D(yaml.SafeDumper): pass
def _repr_default(dumper, data): return dumper.represent_scalar("tag:yaml.org,2002:null", "null")
D.add_multi_representer(object, _repr_default)
with open(os.path.join(stage, "deploy.yaml"), "w") as f:
    yaml.dump(json.loads(json.dumps(dep, default=lambda o: None)), f, Dumper=D, sort_keys=False)
print(f"GATE OK: right-hand Arm7Table {obj} policy; trained default arm pose {[round(v,3) for v in off]} patched into deploy.yaml")
PY
printf '%s\n' "$MS" > "$STAGE/CURRENT"
echo ">>> [policy] staged $MS -> $STAGE (container sees it as ActionModule/policy/CURRENT)"
# ARM PD GAINS = the TRAINING gains (arm kp 40 / kd 3, grasp_env_cfg_arm.py "deploy arm gains
# 40/3"): BridgeModule resolves ALL 27 gains from MovementModule/policy/CURRENT deploy.yaml
# (policies are gain-adapted — 2026-07-06 hard requirement) and otherwise falls back to
# config.py (arms kp 30/kd 3 — the first rig runs). Stage a 27-wide gains-only deploy.yaml
# (legs/torso = config.py values, arms = training) as the Bridge's gain source. On the REAL
# robot the same thing must happen: the active MM policy's arm gains, or a grasp-line CURRENT.
STAGE_MM="$SIM/logs/.grasp_gains_stage"; rm -rf "$STAGE_MM"; mkdir -p "$STAGE_MM/$MS"
ARM_KP="${ARCHB_GRASP_ARM_KP:-40}"; ARM_KD="${ARCHB_GRASP_ARM_KD:-3}"
python3 - "$STAGE_MM/$MS/deploy.yaml" "$ARM_KP" "$ARM_KD" <<'PY'
import sys
out, kp, kd = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
legs_kp = [200.0, 200.0, 200.0, 300.0, 40.0, 40.0, 200.0, 200.0, 200.0, 300.0, 40.0, 40.0, 300.0]
legs_kd = [2.5, 2.5, 2.5, 4.0, 2.0, 2.0, 2.5, 2.5, 2.5, 4.0, 2.0, 2.0, 6.0]
with open(out, "w") as f:
    f.write("# gains-only stand-in for BridgeModule's gains-from-policy resolver (grasp rig)\n")
    f.write("stiffness: [" + ", ".join(str(v) for v in legs_kp + [kp] * 14) + "]\n")
    f.write("damping: [" + ", ".join(str(v) for v in legs_kd + [kd] * 14) + "]\n")
PY
printf '%s\n' "$MS" > "$STAGE_MM/CURRENT"
echo ">>> [gains] Bridge arm PD from the staged gains file: kp $ARM_KP / kd $ARM_KD (training values; ARCHB_GRASP_ARM_KP/KD override)"

# ── scene: built from THIS policy's trained arm pose ────────────────────────
echo ">>> [scene] building $SCENE_NAME (object=$OBJECT, gap=$GAP) from $MS/params/env.yaml"
"$TV_PY" "$ISAAC/scripts/tools/build_grasp_scene.py" --env-yaml "$MSDIR/params/env.yaml" --object "$OBJECT" --gap "$GAP" --out "$SCENE_XML" 2>&1 | sed 's/^/    /' | grep -v "^    $"
[[ -f "$SCENE_XML" ]] || { echo "FATAL: scene build failed"; exit 5; }

# ── preflight venvs ─────────────────────────────────────────────────────────
for _m in BridgeModule ActionModule; do
  [[ -f "$ASPIRED/.venv/$_m/bin/activate" ]] || { echo "FATAL: $ASPIRED/.venv/$_m missing (bash $SIM/tools/prep_env.sh)"; exit 3; }
done

CONTAINER="archb_sim_$$"; MJ_PID=""; WATCHDOG_PID=""
cleanup() {
  [[ -n "${_CLEANED:-}" ]] && return; _CLEANED=1
  echo ""; echo ">>> cleaning up..."
  [[ -n "$WATCHDOG_PID" ]] && kill "$WATCHDOG_PID" 2>/dev/null
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  [[ -n "$MJ_PID" ]] && kill "$MJ_PID" 2>/dev/null || true
  rm -f "$GRASP_FILE" "$GRASP_FILE.tmp"
  echo ">>> sim log (mujoco tab):      $MJ_LOG"
  echo ">>> stack log (FULL console — Bridge/GraspPolicy/rl_grasp/emulator/relay): $LOG_DIR/stack_grasp_$STAMP.log"
  echo ">>> done."
}
trap cleanup EXIT INT TERM

# ── 1. sim ──────────────────────────────────────────────────────────────────
echo ">>> [1] launching unitree_mujoco (scene=$SCENE_NAME via -s, band OFF, ARCHB_GRASP=1) on lo, domain $SIM_DDS_DOMAIN..."
start_sim() {
  ( cd "$MUJOCO/simulate" && env -u WAYLAND_DISPLAY GLFW_PLATFORM=x11 \
      ARCHB_GRASP=1 ARCHB_NO_BAND=1 ARCHB_GRASP_FILE="$GRASP_FILE" \
    ARCHB_GRASP_PLACE_FILE="$SIM/logs/.grasp_place_object" \
      ARCHB_RECORD_FILE="$([[ ${RECORD:-0} = 1 ]] && echo "$RECORD_FILE")" \
      "$MJ_BIN" -r h1_2 -i "$SIM_DDS_DOMAIN" -n lo -s "$SCENE_NAME" ) >>"$MJ_LOG" 2>&1 &
  MJ_PID=$!
}
# the GLFW/X11 startup occasionally segfaults right after a previous teardown (2026-08-28,
# twice in a row; standalone the same binary+scene runs fine) — retry a few times
for attempt in 1 2 3; do
  start_sim
  echo "    pid $MJ_PID (attempt $attempt), log $MJ_LOG"
  sleep 3
  kill -0 "$MJ_PID" 2>/dev/null && break
  echo ">>> [1] MuJoCo died during startup (attempt $attempt) — $(tail -1 "$MJ_LOG" | cut -c1-80)"; sleep 2
done
if ! kill -0 "$MJ_PID" 2>/dev/null; then
  echo "ERROR: MuJoCo exited during startup 3x — see $MJ_LOG"; tail -5 "$MJ_LOG" | sed 's/^/    /'; exit 1
fi
[[ "${RECORD:-0}" = "1" ]] && echo ">>> [rec] in-sim recording -> $RECORD_FILE (stops with the sim)"

# ── 2. container: Bridge (+ real finger_manager) + emulator + relay + ActionModule
( while kill -0 "$MJ_PID" 2>/dev/null; do sleep 2; done
  echo ""; echo ">>> [watchdog] sim process died — tearing down the stack"
  docker rm -f "$CONTAINER" >/dev/null 2>&1 ) &
WATCHDOG_PID=$!
echo ">>> [2] arch-B grasp stack in container (MODE=g): Bridge+finger_manager, Inspire emulator, object_pose_relay, ActionModule"
echo "    sequence cube_task.rl_grasp dispatches ${SEQ_DELAY}s after ActionModule reports ready; watch: [GraspPolicy R] / [rl_grasp R] / [inspire_emu r] / [GRASP] (sim log)"
DOCKER_CMD=(docker run --rm --name "$CONTAINER" --network host --ipc=host
  -e ROS_DOMAIN_ID="${ARCHB_ROS_DOMAIN:-77}" -e ROS_LOCALHOST_ONLY=1
  -e BRIDGE_DDS_DOMAIN="$SIM_DDS_DOMAIN" -e SIM_DDS_DOMAIN="$SIM_DDS_DOMAIN"
  -e MODE=g -e ARCHB_DEBUG="$DEBUG"
  -e BRIDGE_SIM_HANDS=R -e BRIDGE_HAND_IP_R=127.0.0.1 -e BRIDGE_DEBUG_FINGERS="${BRIDGE_DEBUG_FINGERS:-1}"
  -e BRIDGE_GETTER_MIN_DT="${BRIDGE_GETTER_MIN_DT:-0.002}" -e BRIDGE_IMU_PERIOD="${BRIDGE_IMU_PERIOD:-0.002}"
  -e BRIDGE_TAU_CAP_FRAC="${BRIDGE_TAU_CAP_FRAC:-0.6}" -e EMERGENCY_SRV="${EMERGENCY_SRV:-0}"
  -e AM_GRASP_POLICY="$MS" -e AM_ARM_OVERRIDE=1
  -e AM_GRASP_HOVER="${AM_GRASP_HOVER:-ik}" -e AM_GRASP_CUBE_HEIGHT="${AM_GRASP_CUBE_HEIGHT:-$OBJ_H}" -e AM_GRASP_OBJ_FWD_OFFSET="${AM_GRASP_OBJ_FWD_OFFSET:-$OBJ_FWD}" -e AM_GRASP_HOVER_GAP="${AM_GRASP_HOVER_GAP:-$HOVER_GAP_DEF}" -e GRASP_OBJ_BOX="${GRASP_OBJ_BOX:-$OBJ_BOX}" -e AM_GRASP_ARM_DECODE="${AM_GRASP_ARM_DECODE:-handover}" -e AM_GRASP_GRAV_FF="${AM_GRASP_GRAV_FF:-1}" -e AM_GRASP_GRAV_FF_SCALE="${AM_GRASP_GRAV_FF_SCALE:-1.4}" -e AM_GRASP_GRAV_FF_FADE="${AM_GRASP_GRAV_FF_FADE:-1.0}" -e GRASP_PLACE_FILE=/unitree_mujoco/mujoco_sim/logs/.grasp_place_object
  -e GRASP_SIM_FILE=/unitree_mujoco/mujoco_sim/logs/.grasp_sim_state
  -e VISION_HOLD_MIN="$VISION_HOLD_MIN" -e VISION_HOLD_MAX="$VISION_HOLD_MAX"
  -e ARCHB_GRASP_SEQ_DELAY="$SEQ_DELAY"
  -v "$ASPIRED:/workspace" -v "$STAGE:/workspace/ActionModule/policy" -v "$STAGE_MM:/workspace/MovementModule/policy"
  -v "$MUJOCO:/unitree_mujoco" -v "$SDK:/unitree_sdk2_python"
  --entrypoint bash ros2-humble-dev /unitree_mujoco/mujoco_sim/tools/_nodes_in_container.sh)
echo ">>> stack console log: $LOG_DIR/stack_grasp_$STAMP.log"
"${DOCKER_CMD[@]}" 2>&1 | tee "$LOG_DIR/stack_grasp_$STAMP.log"
