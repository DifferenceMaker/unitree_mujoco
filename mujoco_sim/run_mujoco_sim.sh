#!/usr/bin/env bash
# ============================================================================
# run_mujoco_sim.sh — one-command Architecture B × MuJoCo balance sim.
#
# ONE positional PROFILE picks the whole configuration (no flag combinatorics):
#
#   balance    (default) REAL Arch B v2 stack, balance only — BridgeModule
#              (BRIDGE_SIM=1, joints-only) + MovementModule (FixStand→hold→
#              policy; arms hold via the measured-arms fallback)
#   arms       balance + arm_ik_commander: red/blue-dot Cartesian arm targets
#              from the command file  mujoco_sim/logs/.arm_targets
#              ('l x y z' | 'r x y z' | 'default')
#   arms-demo  arms, with auto-cycling demo targets (hands-free eval)
#   teleop     DEPLOYMENT REHEARSAL: the REAL ActionModule (colleague's IK
#              stack; sequences pin ERNEST, container has MoveIt — real-robot parity) + teleop keys
#              while the balance policy stands. Type in THIS terminal:
#              w/s a/d q/e = left hand x/y/z, i/k j/l u/o = roll/pitch/yaw,
#              p = print pose, ESC = quit teleop. (sim `push` unavailable here —
#              the terminal belongs to teleop; use balance/arms for pushes.)
#   ref        the known-good C++ h1_2_ctrl (NO ROS2) — apples-to-apples only.
#              WARNING: runs on DDS domain 0 (h1_2_ctrl hardcodes it) — do NOT
#              use while the real-robot stack is up on this PC.
#
# Options: --quiet (turn off the ARCHB_DEBUG diagnostics; default ON)
#          --no-metrics | --metrics-mode <idle_quiet|push|trainingdist>
#
# ISOLATION (both DDS planes, do not weaken — 2026-07-03 incidents #1 & #2):
#   ROS2 plane:        ROS_DOMAIN_ID=77 + ROS_LOCALHOST_ONLY=1 (container)
#   unitree-SDK plane: DDS domain 1 on lo for sim+metrics+Bridge
#                      (the real robot bus is domain 0; the real stack's
#                      cyclonedds binds 127.0.0.1 too, so domain separation —
#                      not interface separation — is what actually isolates)
#
# Examples:
#   bash run_mujoco_sim.sh                    # balance
#   bash run_mujoco_sim.sh arms-demo          # balance + auto-cycling arm dots
#   echo "l 0.35 0.25 0.10" >> mujoco_sim/logs/.arm_targets   # steer the red dot
#   echo "mode push" > /tmp/archb_metrics.stdin   # label a disturbance mode live
# ============================================================================
set -uo pipefail

# ── paths (edit REPOS if your layout differs) ───────────────────────────────
REPOS="${REPOS:-$HOME/Projects/robot_projects/repos}"
ASPIRED="$REPOS/Aspired_Robot_Project"
MUJOCO="$REPOS/unitree_mujoco"
SDK="$REPOS/unitree_sdk2_python"
RLLAB="$REPOS/unitree_rl_lab"
SIM="$MUJOCO/mujoco_sim"
MJ_BIN="$MUJOCO/simulate/build/unitree_mujoco"
CTRL_BIN="${CTRL_BIN:-$RLLAB/deploy/robots/h1_2/build/h1_2_ctrl}"   # env override: fleetdeck dds_cpp shadow (run_mujoco_dds_cpp.sh)
XML="$MUJOCO/unitree_robots/h1_2/h1_2_comx06_armature.xml"   # comx06 body + Unitree per-joint armature (2026-09-08) — matches config.yaml scene_comx06_armature*.xml
HAND790=0; HAND790_EXPLICIT=0   # --hand790: the same body with the real 790 g Inspire hands (h1_2_comx06_hand790-trained policies, p15+)
TV_PY="${TV_PY:-$HOME/miniconda3/envs/tv/bin/python}"
DDS_LO="$SIM/tools/cyclonedds_lo.xml"

# ── profile + options ────────────────────────────────────────────────────────
PROFILE="balance"; METRICS=1; METRICS_MODE="idle_quiet"; DEBUG=1
while [[ $# -gt 0 ]]; do case "$1" in
  balance|arms|arms-demo|teleop|ref|stop|keys) PROFILE="$1"; shift;;
  --mode-a) echo "NOTE: --mode-a is now the 'balance' profile"; PROFILE="balance"; shift;;
  --mode-b) echo "NOTE: --mode-b is now the 'arms' profile"; PROFILE="arms"; shift;;
  --ref)    PROFILE="ref"; shift;;
  --quiet)        DEBUG=0; shift;;
  --metrics-mode) METRICS_MODE="$2"; shift 2;;
  --no-metrics)   METRICS=0; shift;;
  --hand790)      HAND790=1; HAND790_EXPLICIT=1; shift;;
  -h|--help) sed -n '2,42p' "$0"; exit 0;;
  *) echo "unknown arg: $1 (profiles: balance | arms | arms-demo | teleop | ref | stop)"; exit 1;;
esac; done

# ── BODY AUTO-DETECT (2026-09-17) ────────────────────────────────────────────
# Derive the MuJoCo body from the policies the controller is configured to load,
# instead of relying on someone remembering --hand790. WHY: on 2026-09-17 the whole
# p14b wave (trained on h1_2_comx06_hand790.urdf) was sim2sim'd on
# scene_comx06_armature_desk.xml -- the OLD 0.19 kg hand body, AND a desk scene --
# because this launcher only touched robot_scene when --hand790 was passed and
# otherwise inherited whatever run_mujoco_desk_line.sh had left in config.yaml
# (last written 2026-09-08). Same class as the 2026-08-28 desk-rig body trap.
# The config is a BOARD (many states, one policy each), so a mixed board cannot be
# satisfied by one scene -- that case is reported instead of silently guessed.
# 2026-09-22: for the Architecture B profiles the policy that actually runs is the MovementModule's
# (MM_POLICY, else policy/CURRENT_<MM_LINE>), NOT the h1_2_ctrl board below -- that board is the `ref`
# profile's. Resolving the body from the board while the MM ran a p14e policy would have picked the
# 0.19 kg-hand body for a hand790 policy again. Resolve the MM policy -> its MILESTONE.md slug -> the
# milestone's params/env.yaml first; fall back to the board only for `ref` or when that fails.
_MM_POLICY_RESOLVED=""
if [[ $HAND790_EXPLICIT -eq 0 && "$PROFILE" != "ref" ]]; then
  _mmpol="${MM_POLICY:-$(head -n1 "$ASPIRED/MovementModule/policy/CURRENT_${MM_LINE:-BALANCE}" 2>/dev/null | tr -d '[:space:]')}"
  _mmms=$(grep -m1 -oP '^# Milestone:\s*\K\S+' "$ASPIRED/MovementModule/policy/$_mmpol/MILESTONE.md" 2>/dev/null || true)
  for _env in "$RLLAB/logs/milestones/$_mmms/params/env.yaml" "$REPOS/aspired-isaac-lab/milestone_checkpoints/$_mmms/params/env.yaml"; do
    [[ -n "$_mmms" && -f "$_env" ]] || continue
    if grep -q "h1_2_comx06_hand790" "$_env"; then
      HAND790=1; echo ">>> [body] MovementModule policy '$_mmpol' (milestone $_mmms) names h1_2_comx06_hand790 -> --hand790 AUTO"
    else
      echo ">>> [body] MovementModule policy '$_mmpol' (milestone $_mmms) uses the plain comx06 body -> armature body AUTO"
    fi
    _MM_POLICY_RESOLVED=1; break
  done
  [[ -n "$_MM_POLICY_RESOLVED" ]] || echo ">>> [body] WARNING: could not resolve the MovementModule policy '$_mmpol' to a milestone env.yaml (MILESTONE.md slug '$_mmms') -- falling back to the h1_2_ctrl board scan below, which describes the REF profile's policies, not this one. Pass --hand790 for a real-hand policy."
fi
if [[ $HAND790_EXPLICIT -eq 0 && -z "$_MM_POLICY_RESOLVED" ]]; then
  _cfg="$RLLAB/deploy/robots/h1_2/config/config.yaml"
  _n790=0; _nplain=0; _seen=0; _list790=""
  while read -r _pd; do
    [[ -z "$_pd" ]] && continue
    _md=$(cd "$(dirname "$_cfg")" && readlink -f "$_pd" 2>/dev/null)
    _env="$_md/params/env.yaml"
    # milestones live machine-local; fall back to the git-tracked harvest dir
    [[ -f "$_env" ]] || _env="$REPOS/aspired-isaac-lab/milestone_checkpoints/$(basename "${_md:-$_pd}")/params/env.yaml"
    [[ -f "$_env" ]] || continue
    _seen=$((_seen+1))
    if grep -q "h1_2_comx06_hand790" "$_env"; then
      _n790=$((_n790+1)); _list790="$_list790 $(basename "${_md:-$_pd}")"
    else
      _nplain=$((_nplain+1))
    fi
  done < <(grep -oP 'policy_dir:\s*\K\S+' "$_cfg" 2>/dev/null)
  if   [[ $_seen -eq 0 ]]; then
    echo ">>> [body] WARNING: no policy env.yaml resolved from $_cfg -- body NOT verified. Pass --hand790 if this policy was trained on the real-hand body."
  elif [[ $_n790 -gt 0 && $_nplain -eq 0 ]]; then
    HAND790=1
    echo ">>> [body] all $_seen configured policies name h1_2_comx06_hand790 -> --hand790 AUTO"
  elif [[ $_n790 -eq 0 ]]; then
    echo ">>> [body] all $_seen configured policies use the plain comx06 body -> armature body AUTO"
  else
    echo ">>> [body] WARNING: MIXED board -- $_n790 of $_seen policies are real-hand ($_list790 ), the rest are 0.19 kg-hand."
    echo ">>> [body]          One scene cannot serve both. Defaulting to the PLAIN armature body;"
    echo ">>> [body]          pass --hand790 when you are testing the real-hand policies."
  fi
fi

if [[ $HAND790 -eq 1 ]]; then
  # 2026-09-16: policies trained on h1_2_comx06_hand790.urdf must be evaluated on the matching
  # MuJoCo body (hand mass folded into the wrist bodies, torso re-solved; 77.2676 kg, CoM within
  # 0.08 mm of the URDF). Every p12-p14 policy keeps the plain armature body -- do NOT pass this
  # for them.
  XML="$MUJOCO/unitree_robots/h1_2/h1_2_comx06_armature_hand790.xml"
  if ! grep -qE 'robot_scene: "scene_comx06_armature_hand790\.xml"' "$MUJOCO/simulate/config.yaml"; then
    sed -i 's/robot_scene: "[^"]*"/robot_scene: "scene_comx06_armature_hand790.xml"/' "$MUJOCO/simulate/config.yaml"
    echo ">>> [scene] robot_scene -> scene_comx06_armature_hand790.xml (--hand790)"
  fi
else
  # ALWAYS pin the scene on this path too. Leaving it alone inherited the desk-line's
  # scene (scene_comx06_armature_desk.xml, 2026-09-08) into every balance run.
  if ! grep -qE 'robot_scene: "scene_comx06_armature\.xml"' "$MUJOCO/simulate/config.yaml"; then
    sed -i 's/robot_scene: "[^"]*"/robot_scene: "scene_comx06_armature.xml"/' "$MUJOCO/simulate/config.yaml"
    echo ">>> [scene] robot_scene -> scene_comx06_armature.xml (comx06 + Unitree armature, 0.19 kg hands)"
  fi
fi

# Everything a profile implies, derived in ONE place:
MODE="a"; ARM_DEMO=0; SIM_DDS_DOMAIN=1; DOCKER_TTY=""
case "$PROFILE" in
  balance)   MODE="a";;
  arms)      MODE="b";;
  arms-demo) MODE="b"; ARM_DEMO=1;;
  teleop)    MODE="c"; DOCKER_TTY="-it";;     # keys arrive via the FIFO ('keys' terminal)
  keys)
    # Dedicated teleop key terminal: raw single-key reads forwarded to the
    # FIFO the in-container teleop dispatcher reads (TELEOP_INPUT). Run this
    # in a SECOND clean terminal while the teleop profile is up.
    KFIFO="$SIM/logs/.teleop_keys"
    [[ -p "$KFIFO" ]] || mkfifo "$KFIFO"
    echo "TELEOP KEYS — this terminal now drives the arms (logs stay in the other one)."
    echo "  w/s a/d q/e = left hand x/y/z   i/k j/l u/o = rot   p = pose   ESC = quit teleop"
    echo "  Ctrl+C here just exits this reader (teleop keeps running; rerun 'keys' to reattach)."
    # tools/teleop_keys_feed.py probes for a reader, KEEPS that write fd, and
    # streams stdin through it — see its docstring for the two FIFO gotchas
    # (probe-close EOF, heredoc-eats-stdin) that force this exact shape.
    # min 1: without it many stty impls leave MIN=0 and the first raw read
    # returns EOF instantly (the keys terminal 'exits immediately' bug).
    stty -icanon min 1 time 0 -echo; trap 'stty sane' EXIT INT TERM
    python3 "$SIM/tools/teleop_keys_feed.py" "$KFIFO"
    exit 0;;
  ref)       MODE="ref"; SIM_DDS_DOMAIN=0;;   # h1_2_ctrl hardcodes domain 0
esac
if [[ "$PROFILE" == "teleop" && ! -t 0 ]]; then
  echo "ERROR: the teleop profile needs an interactive terminal (keyboard input)."; exit 1
fi

# ── architecture banner — make it obvious which path is running ──────────────
echo "============================================================"
case "$PROFILE" in
  balance)   echo " ARCH B v2 × MuJoCo  —  profile: balance (REAL stack, no shims)"
             echo "   BridgeModule --sim + MovementModule (arms-hold fallback)" ;;
  arms)      echo " ARCH B v2 × MuJoCo  —  profile: arms (file-driven dot targets)"
             echo "   BridgeModule --sim + MovementModule + arm_ik_commander" ;;
  arms-demo) echo " ARCH B v2 × MuJoCo  —  profile: arms-demo (auto-cycling dots)"
             echo "   BridgeModule --sim + MovementModule + arm_ik_commander" ;;
  teleop)    echo " ARCH B v2 × MuJoCo  —  profile: teleop (DEPLOYMENT REHEARSAL)"
             echo "   BridgeModule --sim + MovementModule + REAL ActionModule (teleop keys)"
             echo "   NOTE: teleop's key reader eats Ctrl+C — press ESC first (quit teleop),"
             echo "         THEN Ctrl+C to stop the run." ;;
  ref)       echo " REFERENCE PATH (NOT Architecture B)  —  C++ h1_2_ctrl"
             echo "   !! DDS domain 0 — do NOT run while the real stack is up on this PC" ;;
esac
echo "   model: $XML"
echo "   sim DDS: lo, domain $SIM_DDS_DOMAIN   |   ROS2: domain ${ARCHB_ROS_DOMAIN:-77}, localhost-only"
echo "============================================================"

LOG_DIR="$SIM/logs"; mkdir -p "$LOG_DIR"
STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
MJ_LOG="$LOG_DIR/mujoco_$STAMP.log"
METRICS_LOG="$LOG_DIR/balance_metrics_${MODE}_$STAMP.log"
METRICS_FIFO="/tmp/archb_metrics.stdin"
BAND_FLAG="$SIM/logs/.band_release"                            # host path (shared mount, gitignored)
BAND_FLAG_CTR="/unitree_mujoco/mujoco_sim/logs/.band_release"  # same file, container path

[[ -x "$MJ_BIN" ]] || { echo "ERROR: MuJoCo binary not built: $MJ_BIN"; exit 1; }
# Architecture B is JOYSTICKLESS by design (config.yaml use_joystick:0) — the
# whole flow runs from the PC: bring-up + engage are automatic (MovementModule),
# band release is the engage flag, disturbances via the sim's stdin `push <vx> <vy>`
# and sim-window keys (9 = band toggle, 7/8 = band height).

# ── 0. sweep: kill anything left from crashed/aborted runs ──────────────────
# A run that died mid-way (Ctrl+C eaten by teleop's raw key reader, a frozen/
# killed sim window) leaves its container + nodes LIVE on the sim DDS bus — the
# next run then joins a bus with an already-engaged controller ("phantom" arms /
# instant policy). Also clears stale command/flag files: .arm_targets is
# replayed in full by arm_ik_commander at startup, and a leftover .band_release
# would drop the band instantly. Runs at every launch AND as the guaranteed
# exit: `bash run_mujoco_sim.sh stop`.
sweep_leftovers() {
  local strays; strays=$(docker ps -q --filter "name=archb_sim_")
  if [[ -n "$strays" ]]; then
    echo ">>> [sweep] removing archb containers: $(docker ps --format '{{.Names}}' --filter 'name=archb_sim_' | tr '\n' ' ')"
    docker rm -f $strays >/dev/null 2>&1
  fi
  pkill -f "$MJ_BIN" 2>/dev/null && echo ">>> [sweep] killed a leftover unitree_mujoco sim"
  pkill -f "balance_metrics.py" 2>/dev/null && echo ">>> [sweep] killed a leftover balance_metrics"
  rm -f "$BAND_FLAG" "$SIM/logs/.arm_targets" "$SIM/logs/.teleop_keys" "$METRICS_FIFO"
}

if [[ "$PROFILE" == "stop" ]]; then
  echo ">>> STOP: tearing down any running/leftover sim stack..."
  sweep_leftovers
  echo ">>> done — environment clean."
  exit 0
fi
sweep_leftovers

# ── preflight: module venvs must exist (official chain, see prep_env.sh) ────
# The stack runs from .venv/<module> inside the container (requirements.txt
# parity with the real robot — no ad-hoc boot pip since 2026-07-14). Fail here,
# on the host, instead of 40 lines deep inside the container.
_need=()
case "$MODE" in
  a)   _need=(MovementModule);;              # balance: no arm source
  b|c) _need=(MovementModule ActionModule);;
esac                                          # ref: dds_cpp binary, no Aspired stack
for _m in "${_need[@]}"; do
  if [[ ! -f "$ASPIRED/.venv/$_m/bin/activate" ]]; then
    echo "FATAL: $ASPIRED/.venv/$_m missing."
    echo "       Create it (no ROS nodes started):  bash $SIM/tools/prep_env.sh"
    exit 3
  fi
done

CONTAINER="archb_sim_$$"; MJ_PID=""; METRICS_PID=""; CTRL_PID=""; WATCHDOG_PID=""
cleanup() {
  [[ -n "${_CLEANED:-}" ]] && return; _CLEANED=1   # run once: Ctrl+C fires INT then EXIT
  echo ""; echo ">>> cleaning up..."
  [[ -n "$WATCHDOG_PID" ]] && kill "$WATCHDOG_PID" 2>/dev/null
  [[ -n "$METRICS_PID" ]] && { kill -INT "$METRICS_PID" 2>/dev/null; sleep 1; }  # → RUN SUMMARY into log
  [[ -n "${FIFO_HOLD_PID:-}" ]] && kill "$FIFO_HOLD_PID" 2>/dev/null
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  [[ -n "$CTRL_PID" ]] && kill "$CTRL_PID" 2>/dev/null || true
  [[ -n "$MJ_PID"  ]] && kill "$MJ_PID"  2>/dev/null || true
  pkill -f "balance_metrics.py" 2>/dev/null || true
  rm -f "$METRICS_FIFO" "$BAND_FLAG"
  [[ "$METRICS" = "1" ]] && echo ">>> metrics log + RUN SUMMARY: $METRICS_LOG"
  echo ">>> done."
}
trap cleanup EXIT INT TERM

# ── 1. MuJoCo (host) ────────────────────────────────────────────────────────
SCENE=$(grep -oP 'robot_scene:\s*"\K[^"]+' "$MUJOCO/simulate/config.yaml" 2>/dev/null || echo "?")
echo ">>> [1] launching unitree_mujoco (h1_2, scene=$SCENE) on lo, domain $SIM_DDS_DOMAIN..."
rm -f "$BAND_FLAG"   # clean slate so a stale flag can't pre-release the band
if [[ "$PROFILE" == "teleop" ]]; then
  # The teleop key FIFO must EXIST before the container starts (the dispatcher
  # opens it read-side and blocks until the 'keys' terminal attaches).
  rm -f "$SIM/logs/.teleop_keys"; mkfifo "$SIM/logs/.teleop_keys"
fi
if [[ "$PROFILE" == "teleop" ]]; then
  # teleop owns the terminal keys — detach the sim's stdin so its `push`
  # reader can't steal keystrokes from the teleop dispatcher.
  ( cd "$MUJOCO/simulate" && ARCHB_BAND_RELEASE_FILE="$BAND_FLAG" "$MJ_BIN" -r h1_2 -i "$SIM_DDS_DOMAIN" -n lo < /dev/null ) >"$MJ_LOG" 2>&1 &
else
  ( cd "$MUJOCO/simulate" && ARCHB_BAND_RELEASE_FILE="$BAND_FLAG" "$MJ_BIN" -r h1_2 -i "$SIM_DDS_DOMAIN" -n lo ) >"$MJ_LOG" 2>&1 &
fi
MJ_PID=$!
echo "    pid $MJ_PID, log $MJ_LOG  (disable the elastic band in the sim window for free-standing balance)"

# SAFETY GATE: if MuJoCo died during startup (e.g. "Joystick open failed."), do
# NOT bring up the ROS2 stack. Without a sim robot, MovementModule's
# /BridgeModule/* topics can discover a REAL BridgeModule on the network and
# command the REAL robot (2026-07-03 incident).
sleep 3
if ! kill -0 "$MJ_PID" 2>/dev/null; then
  echo "ERROR: MuJoCo exited during startup — see $MJ_LOG"
  tail -3 "$MJ_LOG" | sed 's/^/    /'
  echo "       Aborting: refusing to start the ROS2 stack without a live sim."
  exit 1
fi

# ── 2. metrics sidecar (host, tv env, headless → logfile) ───────────────────
if [[ "$METRICS" = "1" ]]; then
  if [[ -x "$TV_PY" ]]; then
    rm -f "$METRICS_FIFO"; mkfifo "$METRICS_FIFO"
    # FIFO write-end holder: stderr detached + killed in cleanup, else the fleetdeck console never sees EOF (2026-08-28)
    sleep infinity > "$METRICS_FIFO" 2>/dev/null & FIFO_HOLD_PID=$!
    HOLD_PID=$!
    "$TV_PY" "$SIM/tools/balance_metrics.py" --iface lo --domain "$SIM_DDS_DOMAIN" --xml "$XML" \
        --mode "$METRICS_MODE" < "$METRICS_FIFO" > "$METRICS_LOG" 2>&1 &
    METRICS_PID=$!
    echo ">>> [2] balance_metrics headless (pid $METRICS_PID)"
    echo "    log:   $METRICS_LOG"
    echo "    label: echo \"mode <push|trainingdist|idle_quiet>\" > $METRICS_FIFO   (also: zero | note <txt>)"
  else
    echo ">>> [2] SKIP metrics: tv python not found at $TV_PY (set TV_PY=...)"
  fi
fi

# ── 3. controller / ROS2 stack ──────────────────────────────────────────────
if [[ "$MODE" = "ref" ]]; then
  [[ -x "$CTRL_BIN" ]] || { echo "ERROR: h1_2_ctrl not built: $CTRL_BIN"; exit 1; }
  echo ">>> [3] REFERENCE path: $CTRL_BIN --network lo"
  echo "    (ensure BalancePush.policy_dir → logs/milestones/p7_1b for an apples-to-apples"
  echo "     comparison with MovementModule; rebuild h1_2_ctrl if you changed it.)"
  echo "    R3 FSM on the controller terminal: FixStand → Balance. Ctrl+C here to stop all."
  "$CTRL_BIN" --network lo & CTRL_PID=$!
  wait "$CTRL_PID"
else
  # WATCHDOG: if the sim dies mid-run (frozen window killed, crash), tear the
  # container down so the ROS2 stack can't outlive its robot (the "phantom
  # controller" orphan). Removing the container unblocks the foreground
  # docker run below -> the EXIT trap finishes the rest of the cleanup.
  ( while kill -0 "$MJ_PID" 2>/dev/null; do sleep 2; done
    echo ""; echo ">>> [watchdog] sim process died — tearing down the stack"
    docker rm -f "$CONTAINER" >/dev/null 2>&1 ) &
  WATCHDOG_PID=$!

  echo ">>> [3] REAL Architecture B v2 stack in container (MODE=$MODE)..."
  # ROS2 ISOLATION (do not remove): the sim stack publishes the same
  # /BridgeModule/* topics the REAL robot stack uses. Unpinned, ROS2/FastDDS
  # discovers peers on ALL interfaces incl. the robot LAN (enp6s0,
  # 192.168.123.x) on the default domain 0 — a sim run WILL drive the real
  # robot if the real BridgeModule is up (2026-07-03 incident). Pin the sim to
  # its own domain + loopback-only discovery; the CYCLONEDDS_URI lo-config
  # only covers the unitree-SDK plane, not ROS2.
  DOCKER_CMD=(docker run --rm $DOCKER_TTY --name "$CONTAINER" --network host --ipc=host
    -e ROS_DOMAIN_ID="${ARCHB_ROS_DOMAIN:-77}" -e ROS_LOCALHOST_ONLY=1
    -e BRIDGE_DDS_DOMAIN="$SIM_DDS_DOMAIN"
    -e MODE="$MODE" -e ARCHB_DEBUG="$DEBUG"
    -e MM_POLICY="${MM_POLICY:-}" -e MM_LINE="${MM_LINE:-BALANCE}"
    -e ARCHB_FIXSTAND_SEC="${ARCHB_FIXSTAND_SEC:-1.0}" -e ARCHB_HOLD_SEC="${ARCHB_HOLD_SEC:-3.5}" -e ARCHB_ACTION_CLIP="${ARCHB_ACTION_CLIP:-100.0}"
    -e ARCHB_ENGAGE_BLEND_SEC="${ARCHB_ENGAGE_BLEND_SEC:-0.3}" -e ARCHB_LOAD_STEPS="${ARCHB_LOAD_STEPS:-3}"
    -e BRIDGE_GETTER_MIN_DT="${BRIDGE_GETTER_MIN_DT:-0.002}" -e BRIDGE_LEG_SLEW_SCALE="${BRIDGE_LEG_SLEW_SCALE:-4.0}"
    -e BRIDGE_IMU_PERIOD="${BRIDGE_IMU_PERIOD:-0.002}" -e EMERGENCY_SRV="${EMERGENCY_SRV:-0}"
    -e ARM_IK_DEMO="$ARM_DEMO"
    -e ARCHB_BAND_RELEASE_FILE="$BAND_FLAG_CTR"
    -v "$ASPIRED:/workspace" -v "$MUJOCO:/unitree_mujoco" -v "$SDK:/unitree_sdk2_python"
    --entrypoint bash ros2-humble-dev /unitree_mujoco/mujoco_sim/tools/_nodes_in_container.sh)
  if [[ -n "$DOCKER_TTY" ]]; then
    # teleop: -it needs a real tty; piping through tee would break it.
    "${DOCKER_CMD[@]}"
  else
    # Archive the stack console: the [Bridge set] latency lines and [MM obs]
    # breakdowns previously existed ONLY on this console and were lost on
    # scroll — the A/B evidence (obs/action/loop ages) now lands next to the
    # metrics log.
    echo ">>> stack console log: $LOG_DIR/stack_$STAMP.log"
    "${DOCKER_CMD[@]}" 2>&1 | tee "$LOG_DIR/stack_$STAMP.log"
  fi
fi
