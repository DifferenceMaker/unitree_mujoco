#!/usr/bin/env bash
# ============================================================================
# run_mujoco_desk_line.sh — DESK-LINE testing twin of run_mujoco_sim.sh.
#
# Adds ONE thing over the parent script: --policy <name> picks which balance
# policy MovementModule runs, WITHOUT touching Aspired_Robot_Project. The
# chosen milestone (from unitree_rl_lab/logs/milestones) is staged into
# mujoco_sim/logs/.desk_policy_stage/ and OVERLAY-MOUNTED over
# /workspace/MovementModule/policy inside the container — the host repo's
# policy/CURRENT stays untouched.
#
#   bash run_mujoco_desk_line.sh --policy desk_fz6          # arms profile default
#   bash run_mujoco_desk_line.sh --policy desk_boxing
#   bash run_mujoco_desk_line.sh --policy desk_tilt arms-demo
#   bash run_mujoco_desk_line.sh --policy dp3_anchor --comx06 balance   # comx06-era desk line
#
# BODY flags (default = SYM): --stock (dp2b batch), --comx06 (dp3_anchor and
# every desk policy off the p12g/comx06 general line — rigid-floor desk scene).
# 90-obs anchor policies work out of the box: the sim plants the training
# anchor on band release (yellow ball, rt/anchor_point), Ctrl+Alt-click moves
# it, --anchor-wander adds vision-style jitter.
#
# <name> matches a milestone dir (exact, or substring — newest wins):
#   desk -> p12j_desk_2026-07-23        desk_fz6 -> p12j_desk_fz6_2026-07-23
#   desk_tilt -> p12j_desk_tilt_...     desk_boxing -> p12k_desk_boxing_...
# Also auto-sets robot_scene to scene_sym_soft07_desk.xml (announced) so the
# desk + alt-click reach targets are present.
#
# Parent docs (profiles, isolation, teleop keys) still apply:
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
CTRL_BIN="$RLLAB/deploy/robots/h1_2/build/h1_2_ctrl"
XML="$MUJOCO/unitree_robots/h1_2/h1_2_sym.xml"   # SYM body — matches scene_sym_soft07_desk + the desk-line policies
                                                 # (--stock rebinds XML + scene to the stock-CoM body below)
TV_PY="${TV_PY:-$HOME/miniconda3/envs/tv/bin/python}"
DDS_LO="$SIM/tools/cyclonedds_lo.xml"

# ── profile + options ────────────────────────────────────────────────────────
PROFILE="arms"; METRICS=1; POLICY="desk_fz6"; ARM_READY=1; ARM_READY_SEC=0; ANCHOR_WANDER=0; METRICS_MODE="idle_quiet"; DEBUG=1; BODY="sym"; RECORD=0; LEDGER=1
while [[ $# -gt 0 ]]; do case "$1" in
  balance|arms|arms-demo|teleop|ref|stop|keys) PROFILE="$1"; shift;;
  --mode-a) echo "NOTE: --mode-a is now the 'balance' profile"; PROFILE="balance"; shift;;
  --mode-b) echo "NOTE: --mode-b is now the 'arms' profile"; PROFILE="arms"; shift;;
  --ref)    PROFILE="ref"; shift;;
  --policy)       POLICY="$2"; shift 2;;
  --stock)        BODY="stock"; shift;;   # STOCK Unitree torso CoM (x=+0.0155, y=+0.0028) —
                                          # for policies trained WITHOUT the SYM tree, e.g. the
                                          # entire dp2b batch (see session 2026-07-28 two-urdf trap)
  --comx06)       BODY="comx06"; shift;;  # comx06 body + RIGID floor (new-soles era, 2026-07-30+):
                                          # dp3_anchor and every desk policy warmstarted off the
                                          # p12g/comx06 general line trains on the Isaac rigid plane
  --no-arm-ready) ARM_READY=0; shift;;
  --arm-ready-sec) ARM_READY_SEC="$2"; shift 2;;
  --anchor-wander) ANCHOR_WANDER=1; shift;;
  --quiet)        DEBUG=0; shift;;
  --metrics-mode) METRICS_MODE="$2"; shift 2;;
  --no-metrics)   METRICS=0; shift;;
  --record)       RECORD=1; shift;;   # x11grab the sim window -> LOG_DIR/mujoco_rec_<stamp>.mp4, auto-stop on exit
  --no-ledger)    LEDGER=0; shift;;   # disable the live reward-ledger overlay + tape
  -h|--help) sed -n '2,42p' "$0"; exit 0;;
  *) echo "unknown arg: $1 (profiles: balance | arms | arms-demo | teleop | ref | stop)"; exit 1;;
esac; done

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

# ── desk-line policy staging (overlay-mounted over MovementModule/policy) ────
MS=""
if [[ "$PROFILE" != "stop" ]]; then
MILESTONES="$RLLAB/logs/milestones"
resolve_ms() {  # exact dir > exact job-name boundary (*_<name>_<date>) > newest substring
  [[ -d "$MILESTONES/$1" ]] && { echo "$1"; return; }
  local m
  m=$(ls -dt "$MILESTONES"/*"_${1}_"[0-9]* 2>/dev/null | head -1)
  [[ -z "$m" ]] && m=$(ls -dt "$MILESTONES"/"${1}_"[0-9]* 2>/dev/null | head -1)
  [[ -z "$m" ]] && m=$(ls -dt "$MILESTONES"/*"$1"* 2>/dev/null | head -1)
  [[ -n "$m" ]] && basename "$m"
}
MS="$(resolve_ms "$POLICY")"
# FALLBACK ROOT (2026-08-25): pod-harvested milestones exist only in the repo
# archive (aspired-isaac-lab/milestone_checkpoints carries exported/ + params/
# via git) — not under this machine's logs/milestones. Same layout, so retry there.
if [[ -z "$MS" || ! -d "$MILESTONES/$MS" ]]; then
  MILESTONES="$REPOS/aspired-isaac-lab/milestone_checkpoints"
  MS="$(resolve_ms "$POLICY")"
  [[ -n "$MS" ]] && echo ">>> [policy] resolved from the repo archive: $MILESTONES/$MS"
fi
[[ -n "$MS" && -d "$MILESTONES/$MS" ]] || { echo "FATAL: no milestone matches --policy '$POLICY' under $MILESTONES"; exit 2; }
for f in exported/policy.onnx params/deploy.yaml; do
  [[ -f "$MILESTONES/$MS/$f" ]] || { echo "FATAL: $MS missing $f (harvest/export incomplete)"; exit 2; }
done
STAGE="$SIM/logs/.desk_policy_stage"
rm -rf "$STAGE"; mkdir -p "$STAGE/$MS"
cp "$MILESTONES/$MS/exported/policy.onnx" "$STAGE/$MS/policy.onnx"
cp "$MILESTONES/$MS/params/deploy.yaml"   "$STAGE/$MS/deploy.yaml"
cp "$MILESTONES/$MS/overrides.json"       "$STAGE/$MS/" 2>/dev/null || true
cp "$MILESTONES/$MS/MILESTONE.md"         "$STAGE/$MS/" 2>/dev/null || true
printf '%s\n' "$MS" > "$STAGE/CURRENT"
echo ">>> [policy] staged $MS -> $STAGE (container sees it as MovementModule/policy/CURRENT)"

# desk scene: the desk + click-to-reach markers must be in the sim
# any *_desk scene OF THE REQUESTED BODY is respected (the softness ladder:
# soft07/soft05/soft04/rigid); a NON-desk or wrong-body scene gets replaced by
# that body's default desk variant. --stock selects the stock-CoM body scenes.
if [[ "$BODY" == "stock" ]]; then
  XML="$MUJOCO/unitree_robots/h1_2/h1_2_stock.xml"
  if ! grep -qE 'robot_scene: "scene_stock_[a-z0-9]+_desk.xml"' "$MUJOCO/simulate/config.yaml"; then
    sed -i 's/robot_scene: "[^"]*"/robot_scene: "scene_stock_cush75_desk.xml"/' "$MUJOCO/simulate/config.yaml"
    echo ">>> [scene] robot_scene -> scene_stock_cush75_desk.xml (STOCK body, cush75-equivalent floor)"
  fi
elif [[ "$BODY" == "comx06" ]]; then
  XML="$MUJOCO/unitree_robots/h1_2/h1_2_comx06.xml"
  if ! grep -qE 'robot_scene: "scene_comx06[_a-z0-9]*desk.xml"' "$MUJOCO/simulate/config.yaml"; then
    sed -i 's/robot_scene: "[^"]*"/robot_scene: "scene_comx06_desk.xml"/' "$MUJOCO/simulate/config.yaml"
    echo ">>> [scene] robot_scene -> scene_comx06_desk.xml (comx06 body, RIGID floor = Isaac parity)"
  fi
elif ! grep -qE 'robot_scene: "scene_sym_[a-z0-9]+_desk.xml"' "$MUJOCO/simulate/config.yaml"; then
  sed -i 's/robot_scene: "[^"]*"/robot_scene: "scene_sym_soft07_desk.xml"/' "$MUJOCO/simulate/config.yaml"
  echo ">>> [scene] robot_scene -> scene_sym_soft07_desk.xml (desk + target balls)"
fi
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
echo "   POLICY: $MS  (staged; host MovementModule/policy untouched)"
echo "   sim DDS: lo, domain $SIM_DDS_DOMAIN   |   ROS2: domain ${ARCHB_ROS_DOMAIN:-77}, localhost-only"
echo "============================================================"

LOG_DIR="$SIM/logs"; mkdir -p "$LOG_DIR"
STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
MJ_LOG="$LOG_DIR/mujoco_$STAMP.log"
RECORD_FILE="$LOG_DIR/mujoco_rec_$STAMP.mp4"
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
ulimit -c unlimited 2>/dev/null   # capture a core if the sim segfaults (lands in simulate/)
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
  ( cd "$MUJOCO/simulate" && env -u WAYLAND_DISPLAY GLFW_PLATFORM=x11 ARCHB_RECORD_FILE="$([[ ${RECORD:-0} = 1 ]] && echo "$RECORD_FILE")" ARCHB_BAND_RELEASE_FILE="$BAND_FLAG" ANCHOR_WANDER="$ANCHOR_WANDER" "$MJ_BIN" -r h1_2 -i "$SIM_DDS_DOMAIN" -n lo < /dev/null ) >"$MJ_LOG" 2>&1 &
else
  # env -u WAYLAND_DISPLAY: force GLFW onto X11/XWayland — the native Wayland
# backend SEGFAULTS in libwayland-client (3x on 2026-08-21, incl. 12:11 BEFORE
# any sim changes: kernel "segfault ... in libwayland-client.so" at identical
# offset) AND a native-Wayland window is invisible to wmctrl/x11grab (--record).
( cd "$MUJOCO/simulate" && env -u WAYLAND_DISPLAY GLFW_PLATFORM=x11 ARCHB_RECORD_FILE="$([[ ${RECORD:-0} = 1 ]] && echo "$RECORD_FILE")" ARCHB_BAND_RELEASE_FILE="$BAND_FLAG" ANCHOR_WANDER="$ANCHOR_WANDER" "$MJ_BIN" -r h1_2 -i "$SIM_DDS_DOMAIN" -n lo ) >"$MJ_LOG" 2>&1 &
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

# ── 1b. recording (--record): IN-SIM capture (sim_record.h via
# ARCHB_RECORD_FILE) — x11grab records BLACK under a Wayland compositor;
# readPixels in the render loop captures window+HUD, stops when the sim exits.
if [[ "${RECORD:-0}" = "1" ]]; then
  echo ">>> [rec] in-sim recording -> $RECORD_FILE (stops with the sim)"
fi

# ── 2. metrics sidecar (host, tv env, headless → logfile) ───────────────────
if [[ "$METRICS" = "1" ]]; then
  if [[ -x "$TV_PY" ]]; then
    rm -f "$METRICS_FIFO"; mkfifo "$METRICS_FIFO"
    sleep infinity > "$METRICS_FIFO" &   # hold the write end open so stdin doesn't EOF
    HOLD_PID=$!
    # LIVE reward ledger (2026-08-21): auto-on when the staged policy carries
    # params/env.yaml (weights parsed from it); disable with --no-ledger.
    LEDGER_ARGS=()
    if [[ "${LEDGER:-1}" = "1" && -f "$MILESTONES/$MS/params/env.yaml" ]]; then
      LEDGER_ARGS=(--ledger "$MILESTONES/$MS/params/env.yaml")
      echo ">>> [2] reward LEDGER on ($MS/params/env.yaml); tape lands in $LOG_DIR"
    fi
    LEDGER_TAPE_DIR="$LOG_DIR" \
    "$TV_PY" "$SIM/tools/balance_metrics.py" --iface lo --domain "$SIM_DDS_DOMAIN" --xml "$XML" \
        --mode "$METRICS_MODE" "${LEDGER_ARGS[@]}" < "$METRICS_FIFO" > "$METRICS_LOG" 2>&1 &
    METRICS_PID=$!
    echo ">>> [2] balance_metrics headless (pid $METRICS_PID)"
    echo "    log:   $METRICS_LOG"
    echo "    label: echo \"mode <push|trainingdist|idle_quiet>\" > $METRICS_FIFO   (also: zero | note <txt>)"
  else
    echo ">>> [2] SKIP metrics: tv python not found at $TV_PY (set TV_PY=...)"
  fi
fi

# ── 3. controller / ROS2 stack ──────────────────────────────────────────────
# ── arm-ready: raise the arms above the desk shortly after engage ───────────
# The commander's joint-space default pose hangs the hands BELOW the 1m desk
# top, and ikpy has no collision awareness (colleague-confirmed limitation) —
# so reaching from under the desk drags arms through it. Seed raised Cartesian
# targets (hands ~ desk + 10 cm) once the stack is up; disable: --no-arm-ready.
# SEEDED BEFORE THE STACK (2026-08-10): the old 25 s sleep raised the arms
# AFTER MovementModule had already released the band and started gating, so a
# 96-obs wish policy hung (no wish -> no engage) while the robot was already
# free — it fell before engaging. The commander reads pre-existing lines on its
# first poll (_file_lines_done starts at 0), so writing the targets NOW means
# the arms are raised and the wish exists before the release cue.
# --arm-ready-sec <n> restores a delayed seed (0 = immediate, the default).
if [[ "$ARM_READY" == "1" && "$MODE" == "b" ]]; then
  _seed_arm_ready() {
    echo "l 0.30 0.25 0.10" >> "$SIM/logs/.arm_targets"
    echo "r 0.30 -0.25 0.10" >> "$SIM/logs/.arm_targets"
    echo ">>> [arm-ready] raised start pose seeded (hands ~desk+10cm)"
  }
  if [[ "${ARM_READY_SEC:-0}" -le 0 ]]; then
    _seed_arm_ready
  else
    ( sleep "$ARM_READY_SEC"; _seed_arm_ready ) &
  fi
fi

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
    -e ARCHB_FIXSTAND_SEC="${ARCHB_FIXSTAND_SEC:-1.0}" -e ARCHB_HOLD_SEC="${ARCHB_HOLD_SEC:-3.5}" -e ARCHB_ACTION_CLIP="${ARCHB_ACTION_CLIP:-100.0}"
    -e ARCHB_ENGAGE_BLEND_SEC="${ARCHB_ENGAGE_BLEND_SEC:-0.3}" -e ARCHB_LOAD_STEPS="${ARCHB_LOAD_STEPS:-3}"
    -e BRIDGE_GETTER_MIN_DT="${BRIDGE_GETTER_MIN_DT:-0.002}" -e BRIDGE_LEG_SLEW_SCALE="${BRIDGE_LEG_SLEW_SCALE:-4.0}"
    -e BRIDGE_IMU_PERIOD="${BRIDGE_IMU_PERIOD:-0.002}" -e EMERGENCY_SRV="${EMERGENCY_SRV:-0}"
    -e ARM_IK_DEMO="$ARM_DEMO"
    -e ARCHB_BAND_RELEASE_FILE="$BAND_FLAG_CTR"
    -v "$ASPIRED:/workspace" -v "$STAGE:/workspace/MovementModule/policy"
    -v "$MUJOCO:/unitree_mujoco" -v "$SDK:/unitree_sdk2_python"
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
