#!/usr/bin/env bash
# exam2_test.sh -- exam #2 Tests 1 and 2 (2026-10-30, results/EXAM2_SPEC.md) on the running homing rig.
#   bash mujoco_sim/tools/exam2_test.sh ladders [--start "-3 0 180"] [--timeout 900]
#   bash mujoco_sim/tools/exam2_test.sh pushes  [--start "-3 0 180"] [--timeout 600]
#   bash mujoco_sim/tools/exam2_test.sh stumble [--start "x y yaw"] [--timeout 300]      # Test 5 (2026-11-02): needs a rig launched with LOCOSW_STUMBLE_SEED
#   bash mujoco_sim/tools/exam2_test.sh enc     [--start "x y yaw"] [--timeout 500]      # Test 6 (2026-11-13, Exam #3): needs a sim built from 2026-11-13 on ("enc_at")
# ladders: teleport, a 20 s stand (stance + the joint state for the stance drawing), then per axis a continuous ladder of 0.05 rungs
#   (4 s each, the last 2.5 s measured) up to the top, a 4 s stop, the same ladder backwards, a 4 s stop: vx 0.05..1.00 / -0.05..-0.50,
#   vy +-0.05..0.30, wz +-0.05..0.80 (340 s of sim). Record -> logs/staircase[$SFX].jsonl (kind "ladders").
# pushes: teleport, 3 s settle, 12 pushes standing (2 s before each) then 12 walking at vx 0.3 (3 s before each): velocity kicks of
#   0.5 / 1.0 / 1.5 m/s front / back / left / right, 5 s measured after each (183 s). Record -> logs/pushes[$SFX].jsonl.
# A fall: recorded, the robot reset on the spot (standing snapshot), the script continues (ladders: the next half; pushes: the next push).
# stumble: teleport to the rough field's start (tools/stumble_field.py START, 45 cm before the field, on the plane), 3 s settle, vx 0.15 for
#   15 s, vx 0.30 for 15 s, a 3 s stop (36 s): per rung the ground-corrected swing apex per foot, scuffs / catches / trips, tilt, falls
#   (bumps 0.4-1 cm since 2026-11-02 evening; the same swing engine runs on EVERY ladder segment -> "sw" in the ladders print below).
#   Record -> logs/stumble[$SFX].jsonl. Without LOCOSW_STUMBLE_SEED the rig has no field: the test runs on the flat (field: null).
# enc: Test 6 encoder offsets (EXAM2_SPEC section 12): the commander teleports to its ENC_START (the ladders' lane) under the balance policy,
#   switches the walk in, walks 8 s, then per FIXED pattern of tools/enc_patterns.py (P0 zero .. P5) tells the sim "enc_at <sim_t> <offsets>"
#   (reported joint angle = true + offset, PD on the reading) and runs a 4 s settle, a 15 s stand and a 19 s walk at vx 0.30 (the last 15 s
#   measured); then, under the stress pattern P5, 8 OFFSET PUSHES (Test 2's 1.0 m/s kick from the 4 sides, standing and walking at 0.3: the
#   same "pre" / "push:..." segments, printed like Test 2's); offsets back to zero at the end (311 s of sim). Record -> logs/enc[$SFX].jsonl;
#   per segment "enc" = the raw bias numbers.
# Start default "-3 0 180": free floor west of the desk, facing away from it (the vx ladder walks ~31 m net to -x).
set -uo pipefail
SIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WHAT="${1:-}"; shift || true
[[ "$WHAT" == ladders || "$WHAT" == pushes || "$WHAT" == stumble || "$WHAT" == enc ]] || { sed -n '2,23p' "$0"; exit 1; }
START="-3 0 180"; TO=$([[ "$WHAT" == ladders ]] && echo 900 || echo 600)
[[ "$WHAT" == stumble ]] && { START=""; TO=300; }   # no coordinates = the commander uses the field's own start
[[ "$WHAT" == enc ]] && { START=""; TO=500; }       # no coordinates = the commander's ENC_START
while [[ $# -gt 0 ]]; do case "$1" in --start) START="$2"; shift 2;; --timeout) TO="$2"; shift 2;; *) echo "unknown arg: $1"; exit 1;; esac; done
SFX="${LOCOSW_INSTANCE:+_$LOCOSW_INSTANCE}"
CMD="$SIM/logs/.homing_cmd$SFX"; LOG="$SIM/logs/stack_locosw_latest$SFX.log"
JL="$SIM/logs/$([[ "$WHAT" == ladders ]] && echo staircase || echo "$WHAT")$SFX.jsonl"
# the commander of THIS run must be up (a stale "latest" symlink fooled the hand-run smokes on 2026-10-29)
for _i in $(seq 1 240); do L2="$(readlink -f "$LOG" 2>/dev/null)"; [[ -n "$L2" && "$L2" -nt "$CMD" ]] && grep -q "\[homing\] pose=" "$L2" 2>/dev/null && break; sleep 0.5; done
grep -q "\[homing\] pose=" "$LOG" || { echo "FATAL: homing_commander not up in $LOG"; exit 2; }
touch "$JL"; n0=$(wc -l < "$JL")
# EXAM_PLAN_BASE (Exam #3 rev 2, 2026-11-13): the item is in the commander's plan file already -- send nothing, wait for the record after
# the line count exam.sh took before the rig started
if [[ -n "${EXAM_PLAN_BASE:-}" ]]; then n0="$EXAM_PLAN_BASE"; else echo "$WHAT${START:+ $START}" >> "$CMD"; fi
for _i in $(seq 1 $((TO * 2))); do [[ "$(wc -l < "$JL")" -gt "$n0" ]] && break; sleep 0.5; done
[[ "$(wc -l < "$JL")" -gt "$n0" ]] || { echo "!! no $WHAT record within $TO s"; echo "stop" >> "$CMD"; exit 3; }
python3 - "$JL" <<'PY'
import json, sys
r = [json.loads(l) for l in open(sys.argv[1]) if l.strip()][-1]
S = [s for s in r["segments"] if s.get("name") not in ("reset",)]
fld = r.get("field")
print(f"{r['kind'].upper()} {r['result']}  t {r['t_total']} s  falls {len(r.get('falls') or [])} {[(f['seg'], f['why']) for f in r.get('falls') or []]}"
      + (f"  engage {r['engage']} (switch wait {r.get('switch_wait_s')} s) stand {r.get('stand_s'):g} s" if r.get("engage") else "")
      + (f"  field seed {fld['seed']} checksum {fld['checksum']}" if fld else ("  field NONE (flat floor)" if r["kind"] == "stumble" else "")))
for s in S:
    v, e = s.get("v_true") or [None] * 3, s.get("energy") or {}
    extra = ""
    if s.get("stop"): extra = f"  stop: settle {s['stop']['settle_s']} s, overshoot {s['stop']['overshoot_m']} m / {s['stop']['overshoot_deg']} deg, steps {s['stop']['steps']}"
    if s.get("push"): p = s["push"]; extra = f"  held {p['held']} dist {p['dist_m']} m tilt {p['tilt_peak_deg']} deg rec {p['recovery_s']} s steps {p['steps']} foot {p['foot_peak_bw']} BW"
    if s.get("stumble") or s.get("swing"):
        q = s.get("stumble") or s["swing"]; cm = lambda a: "/".join("-" if v is None else f"{100 * v:.1f}" for v in a)
        extra = (f"  swings {q['swings']} apex max {cm(q['swing_apex'])} mean {cm(q['swing_apex_mean'])} min {cm(q['swing_apex_min'])} cm"
                 f"  trips {q['trips']} (scuffed {q['scuffed']}, catches {q['catches']}) per step {q['trips_per_step']}  tilt max {s.get('tilt_max')}"
                 f"  on-field {q['on_field_frac']} ground rms {q['ground_rms']}")
        if not s.get("stumble"):   # a ladder segment (2026-11-02): the short form
            extra = f"  sw {q['swings']} scuffed {q['scuffed']} catches {q['catches']} apex max {cm(q['swing_apex'])} mean {cm(q['swing_apex_mean'])} cm" + (extra if s.get("stop") else "")
    if s.get("sep_dev_max") is not None and s["name"] in ("stand", "bal_settle", "settle", "walk_stand"):   # v3 (2026-11-03): the stand's departure from its start + the watch
        extra += (f"  sep {s.get('sep0')} -> {s.get('sep_end')} ({s.get('sep_dir')}) dev max {100 * s['sep_dev_max']:.1f} end {100 * s['sep_dev_end']:.1f} cm  stag dev max {100 * s['stag_dev_max']:.1f} end {100 * s['stag_dev_end']:.1f} cm"
                  f"  yaw {s.get('yaw_change_deg')} deg  z {s.get('z_mean')} (1 s min {s.get('z_run_min')})  tilt peak {s.get('tilt_peak')}")
    if s.get("floor_contact"): extra += f"  FLOOR {s['floor_contact']}"
    if (s.get("self_contact") or {}).get("peak_N", 0) > 20: extra += f"  self-contact {s['self_contact']}"
    if s.get("enc") and s["enc"].get("T"):   # Test 6 (2026-11-13): the segment's raw bias numbers (the paired differences are the scorer's)
        e = s["enc"]
        extra += (f"  ENC {e['phase']}: pitch {e['pitch']:+.2f} roll {e['roll']:+.2f} deg  drift {e['drift_mm_s']:.1f} mm/s  yaw {e['yaw_rate_deg_s']:+.2f} deg/s"
                  + (f"  vx {e['vx']:+.3f} vy {1000 * e['vy']:+.1f} mm/s" if e.get("vx") is not None else "")
                  + (f"  obs-true max {1000 * max(abs(v) for v in e['obs_minus_true']):.1f} mrad (n {e.get('inj_n')}, echo {'yes' if e.get('eo') else 'no'})" if e.get("obs_minus_true") else ""))
    if max(s.get("arm_up_s") or [0]) > 0.5 or max(s.get("arm_cross_s") or [0]) > 0.5: extra += f"  ARMS up {s.get('arm_up_s')} cross {s.get('arm_cross_s')} s"
    vs = "" if v[0] is None else f"v ({v[0]:+.3f} {v[1]:+.3f} {v[2]:+.3f})"
    print(f"  {s['name']:22s} cmd {s.get('cmd')} {vs}  tilt {s.get('tilt_mean')} P {e.get('P_W')} W  lifts {s.get('lifts')}{extra}{'  FELL' if s.get('fell') else ''}")
PY
