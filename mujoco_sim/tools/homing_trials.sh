#!/usr/bin/env bash
# homing_trials.sh -- N homing trials on the running homing rig (run_mujoco_homing.sh): per trial the robot is TELEPORTED to a random
# start 3-5 m from the table (random heading), the commander is told the goal (table edge centre + approach heading), the trial ends
# ARRIVED (balance switched, settled SETTLE_S) or FAIL_*; then the walk policy is requested back for the next trial.
#   bash mujoco_sim/tools/homing_trials.sh [--n 10] [--goal "0.35 0 0"] [--dmin 3] [--dmax 5] [--seed 1] [--settle 25] [--tag NAME] [--no-plot]
# The same --seed gives the same starts: run it per walk policy (HOMING_WALK=... bash run_mujoco_homing.sh) and the batches compare.
# Results: mujoco_sim/logs/homing_trials.jsonl (one JSON line per trial, written by the commander), the trajectory CSVs in
# mujoco_sim/logs/homing_traj/ (10 Hz: pose, commanded / measured / sim-truth velocity, errors), two tables printed here
# (placement; trajectory + command tracking), one benchmark row appended to mujoco_sim/results/HOMING_BENCH.md, and the plots
# (map, velocities, errors) in mujoco_sim/logs/homing_plots/ (needs a python with matplotlib: python3 or the isaacsim conda env,
# override with HOMING_PLOT_PY).
# Sign convention of the printed offsets: the ROBOT's offset from the stop pose -- fwd + = past the stop (towards the table),
# left + = left of the approach line, head + = turned left (CCW). (The jsonl keeps the commander's to-go errors: the same numbers
# with the opposite sign -- along + = still short, lateral + = must go left, heading + = must turn left.)
set -uo pipefail
SIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
N=10; GOAL="0.35 0 0"; DMIN=3.0; DMAX=5.0; SEED=1; SETTLE=25; CONTAINER=archb_locosw; TAG=""; PLOT=1
while [[ $# -gt 0 ]]; do case "$1" in
  --n) N="$2"; shift 2;; --goal) GOAL="$2"; shift 2;; --dmin) DMIN="$2"; shift 2;; --dmax) DMAX="$2"; shift 2;;
  --seed) SEED="$2"; shift 2;; --settle) SETTLE="$2"; shift 2;; --tag) TAG="$2"; shift 2;; --no-plot) PLOT=0; shift;;
  *) echo "unknown arg: $1"; exit 1;; esac; done
SFIFO="$SIM/logs/.sim_stdin"; CMD="$SIM/logs/.homing_cmd"; LOG="$SIM/logs/stack_locosw_latest.log"; JL="$SIM/logs/homing_trials.jsonl"
TRAJ="$SIM/logs/homing_traj"; BENCH="$SIM/results/HOMING_BENCH.md"; STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
[[ -p "$SFIFO" ]] || { echo "FATAL: sim stdin FIFO $SFIFO missing -- is run_mujoco_homing.sh running?"; exit 2; }
for _i in $(seq 1 120); do grep -q "\[homing\] pose=" "$LOG" 2>/dev/null && break; sleep 0.5; done
grep -q "\[homing\] pose=" "$LOG" || { echo "FATAL: homing_commander not up in $LOG"; exit 2; }
WALK="$(grep -m1 -o 'ACTIVE POLICY: [^:]*' "$LOG" | sed 's/ACTIVE POLICY: //')"; BAL="$(grep -m1 -o 'SHADOW POLICY[^:]*: [^:]*' "$LOG" | sed 's/.*: //')"   # "SHADOW POLICY (MM_SWITCH_POLICIES env): <name>: kind=..."
[[ -n "$TAG" ]] || TAG="${WALK:-walk}_seed${SEED}"
echo "policies: walk=$WALK balance=$BAL  goal=($GOAL)  trials=$N  starts $DMIN-$DMAX m  seed $SEED  tag $TAG"
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
  echo "teleport $SX $SY $SYAW" > "$SFIFO"; echo "teleport $SX $SY $SYAW" >> "$CMD"; sleep 2.0
  n0=$(mark); echo "goto $GOAL" >> "$CMD"
  if wait_result "$n0" $((120 + SETTLE)); then tail -1 "$JL" | python3 -c "
import sys,json
r=json.loads(sys.stdin.readline()); e=r.get('final_err') or {}; s=r.get('err_at_switch') or {}
def off(d): return (-d['along_m'], -d['lateral_m'], -d['heading_deg']) if d else (None,)*3   # to-go error -> the robot's offset (fwd, left, CCW)
fa,fl,fh=off(e); sa,sl,sh=off(s); f=lambda v,p=3: '?' if v is None else f'{v:+.{p}f}'
print(f\"   {r['result']:22s} t={r.get('t_total','?')}s switch@{r.get('t_switch','?')}s | at switch: {s.get('planar_m','?')} m (fwd {f(sa)} left {f(sl)}) head {f(sh,2)} deg | settled: {e.get('planar_m','?')} m (fwd {f(fa)} left {f(fl)}) head {f(fh,2)} deg\")
ph=r.get('phase_s') or {}; vf=r.get('v_far') or {}; tr=r.get('track') or {}; sd=r.get('settle_drift') or {}; tp=r.get('start_vs_teleport') or {}
g=lambda k: '?' if not tr.get(k) else f'{tr[k][0]:.2f}'
print(f\"   path {r.get('path_m','?')} m (straight {r.get('straight_m','?')}, eff {r.get('path_eff','?')}) | FAR {ph.get('FAR','?')} s ALIGN {ph.get('ALIGN','?')} s APPROACH {ph.get('APPROACH','?')} s PULSE {ph.get('PULSE',0)} s HOLD {ph.get('HOLD','?')} s\"
      f\" | cruise cmd {vf.get('cmd_mean','?')} meas {vf.get('meas_mean','?')} max {vf.get('meas_max','?')} m/s | gain vx {g('vx')} vy L {g('vy_left')} R {g('vy_right')} wz in-place L {g('wz_inplace_left')} R {g('wz_inplace_right')} walking {g('wz_walking')}\"
      f\" | turns {r.get('n_turns','?')} ({r.get('t_turn_s','?')} s) pulses {r.get('n_pulses','?')} realign {r.get('n_realign','?')}\"
      + (f\" | stand after the switch: moved fwd {-sd['along_m']:+.3f} left {-sd['lateral_m']:+.3f} m, turned {-sd['heading_deg']:+.2f} deg\" if sd else '')
      + (f\" | drop->goto: {tp['planar']} m, {tp['dyaw_deg']:+.1f} deg\" if tp else ''))"
  else echo "   !! no result within $((120 + SETTLE)) s"; echo "stop" >> "$CMD"; fi
done
echo; echo "=== SUMMARY ($JL)"; python3 - "$JL" "$N" "$BENCH" "$WALK" "$BAL" "$SEED" "$TAG" "$STAMP" <<'PY'
import sys,json,statistics as st,os,datetime
jl,n,bench,walk,bal,seed,tag,stamp=sys.argv[1:9]
rows=[json.loads(l) for l in open(jl) if l.strip()][-int(n):]
ok=[r for r in rows if r.get("result")=="ARRIVED"]
print(f"trials {len(rows)}  ARRIVED {len(ok)}  other: {[r['result'] for r in rows if r.get('result')!='ARRIVED']}")
def off(d): return (-d['along_m'], -d['lateral_m'], -d['heading_deg']) if d else (None,)*3
f=lambda v,p=3: '?' if v is None else f'{v:+.{p}f}'
print("--- placement: the ROBOT's offset from the stop pose (fwd + = towards the table, left +, head + = turned left); @sw = at the switch, settled = after the balance policy's stand")
print(f"{'trial':>5} {'start (x y yaw)':>24} {'result':22} {'t_tot':>6} {'t_sw':>6} {'planar@sw':>9} {'fwd@sw':>7} {'left@sw':>7} {'head@sw':>8} {'planar':>7} {'fwd':>7} {'left':>7} {'head':>7} {'drop->goto':>12}")
for r in rows:
    s=r.get("err_at_switch") or {}; e=r.get("final_err") or {}; st_=r.get("start") or ["?","?","?"]; tp=r.get("start_vs_teleport") or {}
    sa,sl,sh=off(s); fa,fl,fh=off(e)
    print(f"{r['trial']:>5} {str(st_):>24} {r.get('result',''):22} {r.get('t_total','?'):>6} {r.get('t_switch','?'):>6} {s.get('planar_m','?'):>9} {f(sa):>7} {f(sl):>7} {f(sh,2):>8} {e.get('planar_m','?'):>7} {f(fa):>7} {f(fl):>7} {f(fh,2):>7} {(str(tp.get('planar','?'))+' m '+f(tp.get('dyaw_deg'),1)+'deg') if tp else '?':>12}")
print("--- trajectory + command tracking: gains = measured/commanded over steady-command ticks (vx: cmd >= 0.1 m/s; vy L/R: |cmd| >= 0.08; wz in place: |cmd| = 0.8; wz walking: vx >= 0.1 & |wz| >= 0.2; ? = too few ticks); stand = what the hold+switch+settle added")
print(f"{'trial':>5} {'path':>6} {'strt':>6} {'eff':>5} {'t_far':>6} {'t_aln':>6} {'t_app':>6} {'t_pls':>6} {'cruise cmd/meas/max':>20} {'g_vx':>5} {'g_vyL':>6} {'g_vyR':>6} {'g_wzL':>6} {'g_wzR':>6} {'g_wzW':>6} {'turns':>5} {'t_trn':>6} {'pls':>4} {'realn':>5} {'stand fwd/left/head':>22}")
def g(tr,k): return '?' if not (tr or {}).get(k) else f"{tr[k][0]:.2f}"
for r in rows:
    ph=r.get("phase_s") or {}; vf=r.get("v_far") or {}; tr=r.get("track") or {}; sd=r.get("settle_drift") or {}
    cru=f"{vf.get('cmd_mean','?')}/{vf.get('meas_mean','?')}/{vf.get('meas_max','?')}"
    stand=f"{-sd['along_m']:+.3f}/{-sd['lateral_m']:+.3f}/{-sd['heading_deg']:+.1f}" if sd else '?'
    print(f"{r['trial']:>5} {r.get('path_m','?'):>6} {r.get('straight_m','?'):>6} {r.get('path_eff','?'):>5} {ph.get('FAR','?'):>6} {ph.get('ALIGN','?'):>6} {ph.get('APPROACH','?'):>6} {ph.get('PULSE',0):>6} {cru:>20} {g(tr,'vx'):>5} {g(tr,'vy_left'):>6} {g(tr,'vy_right'):>6} {g(tr,'wz_inplace_left'):>6} {g(tr,'wz_inplace_right'):>6} {g(tr,'wz_walking'):>6} {r.get('n_turns','?'):>5} {r.get('t_turn_s','?'):>6} {r.get('n_pulses','?'):>4} {r.get('n_realign','?'):>5} {stand:>22}")
if ok:
    P=[r["final_err"]["planar_m"] for r in ok]; H=[abs(r["final_err"]["heading_deg"]) for r in ok]; T=[r["t_total"] for r in ok]
    SP=[r["err_at_switch"]["planar_m"] for r in ok if r.get("err_at_switch")]; SH=[abs(r["err_at_switch"]["heading_deg"]) for r in ok if r.get("err_at_switch")]
    L=[-r["final_err"]["lateral_m"] for r in ok]; HD=[-r["final_err"]["heading_deg"] for r in ok]
    EFF=[r["path_eff"] for r in ok if r.get("path_eff")]; VC=[r["v_far"]["meas_mean"] for r in ok if r.get("v_far")]
    GX=[r["track"]["vx"][0] for r in ok if (r.get("track") or {}).get("vx")]; GL=[r["track"]["wz_inplace_left"][0] for r in ok if (r.get("track") or {}).get("wz_inplace_left")]
    GR=[r["track"]["wz_inplace_right"][0] for r in ok if (r.get("track") or {}).get("wz_inplace_right")]
    DL=[-r["settle_drift"]["lateral_m"] for r in ok if r.get("settle_drift")]; DH=[-r["settle_drift"]["heading_deg"] for r in ok if r.get("settle_drift")]
    m=lambda v,p=3: '?' if not v else f"{st.mean(v):.{p}f}"; mx=lambda v,p=3: '?' if not v else f"{max(v):.{p}f}"; md=lambda v,p=3: '?' if not v else f"{st.median(v):.{p}f}"
    print(f"at the switch: planar median {md(SP)} max {mx(SP)} m | |head| median {md(SH,2)} max {mx(SH,2)} deg")
    print(f"settled over ARRIVED: planar mean {m(P)} max {mx(P)} m | |heading| mean {m(H,2)} max {mx(H,2)} deg | left offset mean {m(L)} (sign = side) | heading mean {m(HD,2)} | time mean {m(T,1)} s | pass (<=0.05 m, <=2 deg): {sum(1 for p,h in zip(P,H) if p<=0.05 and h<=2.0)}/{len(ok)}")
    print(f"trajectory: path eff mean {m(EFF)} | cruise measured mean {m(VC)} m/s | gain vx {m(GX,2)} | wz in-place gain L {m(GL,2)} R {m(GR,2)} | stand after the switch: left {m(DL)} m, turned {m(DH,2)} deg (means, signed)")
    hdr="| date | walk | balance | seed | n | arrived | @switch planar med/max m | @switch head med/max deg | settled planar mean/max m | settled head mean/max deg | settled left mean m | t mean s | path eff | cruise m/s | gain vx | gain wz L/R | stand left / turned | tag |\n|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n"
    row=f"| {stamp} | {walk} | {bal or '(none)'} | {seed} | {len(rows)} | {len(ok)} | {md(SP)}/{mx(SP)} | {md(SH,2)}/{mx(SH,2)} | {m(P)}/{mx(P)} | {m(H,2)}/{mx(H,2)} | {m(L)} | {m(T,1)} | {m(EFF)} | {m(VC)} | {m(GX,2)} | {m(GL,2)}/{m(GR,2)} | {m(DL)} m / {m(DH,2)} deg | {tag} |\n"
    new=not os.path.exists(bench)
    with open(bench,"a") as fh:
        if new: fh.write("# HOMING benchmark -- one row per tools/homing_trials.sh batch (same seed = same starts; compare walk policies row by row)\n\n"+hdr)
        fh.write(row)
    print(f"benchmark row appended -> {bench}")
PY
if [[ "$PLOT" == 1 ]]; then
  PY="${HOMING_PLOT_PY:-}"
  [[ -n "$PY" ]] || for c in python3 "$HOME/miniconda3/envs/isaacsim/bin/python"; do "$c" -c 'import matplotlib' 2>/dev/null && { PY="$c"; break; }; done
  if [[ -n "$PY" ]]; then mkdir -p "$SIM/logs/homing_plots"; "$PY" "$SIM/tools/homing_plot.py" --jsonl "$JL" --n "$N" --traj "$TRAJ" --out "$SIM/logs/homing_plots/homing_${STAMP}_${TAG}" --title "$TAG  ($STAMP, walk $WALK, balance ${BAL:-none})"
  else echo "(no python with matplotlib found: plots skipped; set HOMING_PLOT_PY=<python>)"; fi
fi
