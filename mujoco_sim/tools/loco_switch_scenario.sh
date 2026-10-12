#!/usr/bin/env bash
# loco_switch_scenario.sh — the scripted walk<->balance switching test against a
# running run_mujoco_loco_switch.sh rig (2026-10-12).
#
#   FixStand on the band -> WALK engages at release -> drive forward -> stop
#   -> switch to BALANCE -> push (x), push (y) -> switch to WALK -> drive back
#   -> stop -> switch to BALANCE -> hold.
#
# Every step is time-stamped; the verdict at the end is read from the stack log
# (switch lines, warnings, lean) and the metrics log (falls). Exit 0 = both
# switches happened both ways and nothing fell.
#
#   bash tools/loco_switch_scenario.sh [--walk lm6b_sharp] [--balance p14h_stancewide]
#                                      [--vx 0.3] [--drive 6] [--push 0.5]
set -uo pipefail
SIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WALK="lm6b_sharp"; BALANCE="p14h_stancewide"; VX=0.3; DRIVE=6; PUSH=0.5; CONTAINER="archb_locosw"
while [[ $# -gt 0 ]]; do case "$1" in
  --walk) WALK="$2"; WALK_SET=1; shift 2;; --balance) BALANCE="$2"; WALK_SET=1; shift 2;;
  --vx) VX="$2"; shift 2;; --drive) DRIVE="$2"; shift 2;; --push) PUSH="$2"; shift 2;;
  *) echo "unknown arg: $1"; exit 1;;
esac; done
KFIFO="$SIM/logs/.teleop_keys"; SFIFO="$SIM/logs/.sim_stdin"
LOG="$SIM/logs/stack_locosw_latest.log"; MET="$SIM/logs/locosw_metrics_latest.log"
for _i in $(seq 1 120); do [[ -f "$LOG" ]] && break; sleep 0.5; done   # the rig creates it a few s after launch
[[ -f "$LOG" ]] || { echo "FATAL: no stack log at $LOG — is run_mujoco_loco_switch.sh running?"; exit 2; }
# Policy names: --walk/--balance override; otherwise read what MovementModule loaded.
if [[ -z "${WALK_SET:-}" ]]; then
  for _i in $(seq 1 120); do grep -q "SHADOW POLICY:" "$LOG" && break; sleep 0.5; done
  _w="$(grep -m1 -o 'ACTIVE POLICY: [^:]*' "$LOG" | sed 's/ACTIVE POLICY: //')"
  _b="$(grep -m1 -o 'SHADOW POLICY: [^:]*' "$LOG" | sed 's/SHADOW POLICY: //')"
  [[ -n "$_w" ]] && WALK="$_w"; [[ -n "$_b" ]] && BALANCE="$_b"
fi
echo "policies: walk=$WALK  balance=$BALANCE"
T0=$EPOCHREALTIME
ts() { printf '[%6.1fs] ' "$(awk "BEGIN{print $EPOCHREALTIME - $T0}")"; }
say() { ts; echo "$*"; }
wait_log() {   # wait_log <regex> <timeout_s> [start_line]
  local pat="$1" to="$2" from="${3:-1}" i
  for i in $(seq 1 $((to * 2))); do
    if tail -n +"$from" "$LOG" | grep -qE "$pat"; then return 0; fi
    sleep 0.5
  done
  return 1
}
lines() { wc -l < "$LOG"; }
pol() { docker exec "$CONTAINER" bash -c "source /opt/ros/humble/setup.bash; ros2 topic pub --once /MovementModule/policy std_msgs/String \"data: $1\"" >/dev/null 2>&1; }
keys() { printf "$1" > "$KFIFO"; }
push() { echo "push $1 $2" > "$SFIFO"; }
# walk_teleop step mode: one Up press = +WALK_KEYS_STEP (0.05 m/s). N presses -> N*0.05.
drive() {   # drive <vx>  (signed; Up or Down presses)
  local n; n=$(python3 -c "import math; print(int(round(abs($1)/0.05)))")
  local k; [[ "$(python3 -c "print(1 if $1 >= 0 else 0)")" = "1" ]] && k='\033[A' || k='\033[B'
  local i; for i in $(seq 1 "$n"); do keys "$k"; sleep 0.08; done
}
zero() { keys ' '; }
switch_to() {   # switch_to <policy> <from> <timeout>
  local from_line; from_line=$(lines)
  say "SWITCH -> $1 (requested)"; pol "$1"
  if wait_log "POLICY SWITCH: $2 -> $1" "$3" "$from_line"; then
    say "   switched: $(tail -n +"$from_line" "$LOG" | grep -E "POLICY SWITCH: $2 -> $1" | tail -1 | sed 's/.*POLICY SWITCH: //')"
    return 0
  fi
  say "   !! switch to $1 did NOT happen within $3 s. Last gate lines:"
  tail -n +"$from_line" "$LOG" | grep -E "policy switch" | tail -3 | sed 's/^/      /'
  return 1
}
lean_now() { grep -E "LEAN pitch" "$LOG" | tail -1 | sed -E 's/.*LEAN pitch ([^ ]+) deg.*roll ([^ ]+) deg.*/pitch \1 roll \2/'; }
fails=0

say "waiting for the WALK policy to engage ($WALK)…"
wait_log "policy ENGAGED: $WALK" 90 || { say "FATAL: $WALK never engaged"; exit 3; }
say "walk ENGAGED. settling 4 s"; sleep 4

say "DRIVE forward vx=$VX for $DRIVE s"; drive "$VX"; sleep "$DRIVE"
say "ZERO the stick, 3 s to come to rest"; zero; sleep 3

switch_to "$BALANCE" "$WALK" 25 || fails=$((fails+1))
sleep 3; say "balance settled: $(lean_now)"
say "PUSH +x $PUSH m/s"; push "$PUSH" 0; sleep 4; say "   after push x: $(lean_now)"
say "PUSH +y $PUSH m/s"; push 0 "$PUSH"; sleep 4; say "   after push y: $(lean_now)"

switch_to "$WALK" "$BALANCE" 25 || fails=$((fails+1))
sleep 2
say "DRIVE back vx=-$VX for $DRIVE s"; drive "-$VX"; sleep "$DRIVE"
say "ZERO the stick, 3 s"; zero; sleep 3

switch_to "$BALANCE" "$WALK" 25 || fails=$((fails+1))
say "holding balance 8 s"; sleep 8; say "final: $(lean_now)"

echo; echo "================ VERDICT ================"
grep -E "POLICY SWITCH|HANDOFF|WARNING|policy ENGAGED|BALANCE STABLE" "$LOG" | grep -v "no /BridgeModule/joint_set_arms" | tail -20
falls="?"
if [[ -f "$MET" ]]; then
  falls=$(grep -c "^\[FALL\]" "$MET")   # balance_metrics prints one [FALL] #n line per fall (tilt > 1 rad)
  echo "metrics: $(grep -E "^\[METRICS\]" "$MET" | tail -1)"
fi
n_sw=$(grep -c "POLICY SWITCH:" "$LOG")
echo "switches: $n_sw (expected 3)   scenario failures: $fails   falls (metrics): $falls"
if [[ "$fails" -eq 0 && "$n_sw" -ge 3 && ( "$falls" = "0" || "$falls" = "?" ) ]]; then echo "RESULT: PASS"; exit 0; fi
echo "RESULT: FAIL"; exit 1
