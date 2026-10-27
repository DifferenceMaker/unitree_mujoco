#!/usr/bin/env bash
# homing_trials.sh -- N homing trials on the running homing rig (run_mujoco_homing.sh): per trial the robot is TELEPORTED to a random
# start 3-5 m from the table (random heading), the commander is told the goal (table edge centre + approach heading), the trial ends
# ARRIVED (balance switched, settled SETTLE_S) or FAIL_*; then the walk policy is requested back for the next trial.
#   bash mujoco_sim/tools/homing_trials.sh [--n 10] [--goal "0.35 0 0"] [--dmin 3] [--dmax 5] [--seed 1] [--settle 25]
# Results: mujoco_sim/logs/homing_trials.jsonl (one JSON line per trial, written by the commander) + the summary table printed here.
set -uo pipefail
SIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
N=10; GOAL="0.35 0 0"; DMIN=3.0; DMAX=5.0; SEED=1; SETTLE=25; CONTAINER=archb_locosw
while [[ $# -gt 0 ]]; do case "$1" in
  --n) N="$2"; shift 2;; --goal) GOAL="$2"; shift 2;; --dmin) DMIN="$2"; shift 2;; --dmax) DMAX="$2"; shift 2;;
  --seed) SEED="$2"; shift 2;; --settle) SETTLE="$2"; shift 2;; *) echo "unknown arg: $1"; exit 1;; esac; done
SFIFO="$SIM/logs/.sim_stdin"; CMD="$SIM/logs/.homing_cmd"; LOG="$SIM/logs/stack_locosw_latest.log"; JL="$SIM/logs/homing_trials.jsonl"
[[ -p "$SFIFO" ]] || { echo "FATAL: sim stdin FIFO $SFIFO missing -- is run_mujoco_homing.sh running?"; exit 2; }
for _i in $(seq 1 120); do grep -q "\[homing\] pose=" "$LOG" 2>/dev/null && break; sleep 0.5; done
grep -q "\[homing\] pose=" "$LOG" || { echo "FATAL: homing_commander not up in $LOG"; exit 2; }
WALK="$(grep -m1 -o 'ACTIVE POLICY: [^:]*' "$LOG" | sed 's/ACTIVE POLICY: //')"; BAL="$(grep -m1 -o 'SHADOW POLICY: [^:]*' "$LOG" | sed 's/SHADOW POLICY: //')"
echo "policies: walk=$WALK balance=$BAL  goal=($GOAL)  trials=$N  starts $DMIN-$DMAX m  seed $SEED"
read -r GX GY GPSI <<< "$GOAL"
mark() { wc -l < "$JL" 2>/dev/null || echo 0; }
wait_result() {  # wait_result <lines_before> <timeout>
  local n0="$1" to="$2" i; for i in $(seq 1 $((to * 2))); do [[ "$(mark)" -gt "$n0" ]] && return 0; sleep 0.5; done; return 1; }
wait_active() {  # wait_active <policy> <timeout>
  local i; for i in $(seq 1 $((${2} * 2))); do
    a=$(docker exec "$CONTAINER" bash -c "source /opt/ros/humble/setup.bash; timeout 2 ros2 topic echo --once /MovementModule/policy_active std_msgs/String 2>/dev/null | grep -o 'data: .*' | cut -d' ' -f2" 2>/dev/null)
    [[ "$a" == "$1" ]] && return 0; sleep 0.5; done; return 1; }
touch "$JL"
for t in $(seq 1 "$N"); do
  # start pose: distance d in [DMIN, DMAX] from the goal, direction spread +-60 deg around the approach line (behind the table edge), random heading
  read -r SX SY SYAW <<< "$(python3 -c "
import math,random; random.seed($SEED*1000+$t)
d=random.uniform($DMIN,$DMAX); th=math.radians($GPSI)+math.pi+math.radians(random.uniform(-60,60))
print(f'{$GX+d*math.cos(th):.3f} {$GY+d*math.sin(th):.3f} {random.uniform(-180,180):.1f}')")"
  echo "=== trial $t: teleport to ($SX, $SY, $SYAW deg), goal ($GOAL)"
  # every trial starts under the WALK policy (the previous trial, or a hand run, may have ended in balance)
  if ! wait_active "$WALK" 1; then echo "walk" >> "$CMD"; wait_active "$WALK" 20 || echo "   !! walk policy not active after 20 s (continuing)"; sleep 1; fi
  echo "teleport $SX $SY $SYAW" > "$SFIFO"; sleep 2.0
  n0=$(mark); echo "goto $GOAL" >> "$CMD"
  if wait_result "$n0" $((120 + SETTLE)); then tail -1 "$JL" | python3 -c "
import sys,json; r=json.loads(sys.stdin.readline()); e=r.get('final_err') or {}; s=r.get('err_at_switch') or {}
print(f\"   {r['result']:22s} t={r.get('t_total','?')}s switch@{r.get('t_switch','?')}s  at switch: planar {s.get('planar_m','?')} m head {s.get('heading_deg','?')} deg | settled: planar {e.get('planar_m','?')} m (along {e.get('along_m','?')}, lat {e.get('lateral_m','?')}) head {e.get('heading_deg','?')} deg\")"
  else echo "   !! no result within $((120 + SETTLE)) s"; echo "stop" >> "$CMD"; fi
done
echo; echo "=== SUMMARY ($JL)"; python3 - "$JL" "$N" <<'PY'
import sys,json,statistics as st
rows=[json.loads(l) for l in open(sys.argv[1]) if l.strip()][-int(sys.argv[2]):]
ok=[r for r in rows if r.get("result")=="ARRIVED"]
print(f"trials {len(rows)}  ARRIVED {len(ok)}  other: {[r['result'] for r in rows if r.get('result')!='ARRIVED']}")
print(f"{'trial':>5} {'start (x y yaw)':>24} {'result':22} {'t_tot':>6} {'t_sw':>6} {'planar@sw':>9} {'head@sw':>8} {'planar':>7} {'along':>7} {'lat':>7} {'head':>7}")
for r in rows:
    s=r.get("err_at_switch") or {}; e=r.get("final_err") or {}; st_=r.get("start") or ["?","?","?"]
    print(f"{r['trial']:>5} {str(st_):>24} {r.get('result',''):22} {r.get('t_total','?'):>6} {r.get('t_switch','?'):>6} {s.get('planar_m','?'):>9} {s.get('heading_deg','?'):>8} {e.get('planar_m','?'):>7} {e.get('along_m','?'):>7} {e.get('lateral_m','?'):>7} {e.get('heading_deg','?'):>7}")
if ok:
    P=[r["final_err"]["planar_m"] for r in ok]; H=[abs(r["final_err"]["heading_deg"]) for r in ok]; T=[r["t_total"] for r in ok]
    print(f"settled over ARRIVED: planar mean {st.mean(P):.3f} max {max(P):.3f} m | |heading| mean {st.mean(H):.2f} max {max(H):.2f} deg | time mean {st.mean(T):.1f} s | pass (<=0.05 m, <=2 deg): {sum(1 for p,h in zip(P,H) if p<=0.05 and h<=2.0)}/{len(ok)}")
PY
