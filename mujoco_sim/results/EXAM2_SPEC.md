# Exam #2 — the refined standardized walk exam (2026-10-30)

Status: **implemented 2026-10-30** (operator: "I like what you have proposed + my refinements. Start implementing."), first smoke
on lm7f_kit_alive5; g/b values marked *provisional* wait for the calibration run (section 6). `tools/exam.sh` runs Exam #2 by default
(`--exam 1` = the 2026-10-29 exam). Decisions of 2026-10-30 (section 9 answered): quality-based score, the weights below, 4 s rungs,
pushes 0.5 / 1.0 / 1.5 m/s standing and walking at vx 0.3, P-law homing turns with counted rescues, starts all around the desk with
around-desk planning, the stance drawing. Open: the training band (section 8), seeds per exam (section 9).
Exam #1 = `tools/exam.sh` as of 2026-10-29 (Test 0 handover, Test 1 staircase, Test 2 homing, score 40/20/20/10/10; catalogue in
`EXAM_METRICS.md`). Exam #2 keeps the rig (lock-step headless MuJoCo, the real Bridge + MovementModule stack, the comx06 armature
hand790 body, p14h_stancewide as the balance policy) and changes what is tested and how it is scored.

**Current score version: v4 (2026-11-09, section 11)** — balance partner = CURRENT_BALANCE, 48 pushes (8 directions), rung error floor
0.05, the constraints block, calibration v4-2026-11-09; v3 = section 10, v2 = sections 1–7.
**EXAM #3 (2026-11-13, sections 12 + 13) = Exam #2 v4's tests + Test 6 "encoder offsets", scored inside the qualities**: `exam.sh --exam 3`
(owner: "No flags. This is our new test"); `exam.sh --exam 2` = Exam #2 v4 exactly (the running s1_pilot study). The default: section 13.6.

Operator brief (meeting 2026-10-30): everything the exam records must count (Test 0 and the stance included); the full command band,
in 0.05 steps, for vx, vy and wz; stands tested on their own; a push test; energy in every phase; stability first in the weights;
"crawl" = literally on the belly or knees (a 6 cm knee sink is a small deduction, not a fail).

## Done 2026-10-30
- Homing stop: torso front **0.25 m** from the desk edge (was 0.35): `exam.sh --stop-dist 0.25` (default), recorded as
  `meta.stop_dist`. The commander default (`HOMING_STOP_DIST`, used on hardware) is unchanged.
- `EXAM_METRICS.md`: the phantom fourth handover pose ("both arms up") removed — the code runs three.
- fleetdeck: per-test sections, readout strip instead of the popup, stop-zone zoom, commanded-vs-achieved plots, footprint drawing,
  archive score badges (fleetdeck `CONTEXT.md`, round 2026-10-30b).

## 1. The structure: score QUALITIES, use TESTS as instruments
Two ways to build the number:
- per test (4 tests × 25): simple, but a quality that shows in several tests (torso stability shows in the handover, the stands, the
  pushes) gets an accidental weight, and adding a fifth test silently re-weights everything;
- per quality (survival, stability, tracking, …): each weight is a decision; a new test only adds evidence to the qualities it
  measures.

**Proposal: per quality**, with the per-test sub-scores shown beside it on the page for diagnosis. Every metric becomes 0–1 through one
ramp between a good value g (full marks) and a bad value b (zero):

$$s = \min(\max(\frac{b - x}{b - g}, 0), 1)$$

Example: stand drift, g = 1 cm, b = 10 cm, measured 1.9 cm: s = (10 − 1.9) / 9 = 0.90. The g/b values below are first guesses; the
calibration run (section 6) sets them from real distributions, the same way the stance ranges were meant to come from the first wave.

## 2. Weights (from the operator's priority order)
Priority given: survival first, then stability (upright, torso stable), then tracking precision + not hurting itself (foot impacts)
+ energy + action smoothness, then push robustness and homing.

| quality | weight | what feeds it |
|---|---|---|
| survival | 10 | falls in Tests 0, 1, 3 (a push-test fall belongs to robustness) |
| stability | 30 | stands (20 s stand + every stop), torso while walking, the handover |
| tracking | 20 | the full-band ladders, vx / vy / wz |
| gait & body care | 10 | foot impact forces, action smoothness, foot sliding, swing height |
| energy | 10 | power standing, cost of transport walking |
| push robustness | 10 | Test 2 |
| homing | 10 | Test 3 |

Survival: 10 × 0.5^falls (each fall halves it), and a fall voids the test it happened in (that test's metrics score 0). A fall = base
below 0.55 m **or any non-foot body touching the floor** (knees, hands, pelvis: the literal crawl).

## 3. The tests

### Test 0 — handover (kept; scored now)
3 arm poses (default, right_up_left_side, both_forward), balance 10 s → walk at zero command 8 s → balance 10 s. Operator: the next
policy must hold the pose without stepping or losing balance, and be extremely smooth.

| metric (both switch directions, every pose) | g | b | note |
|---|---|---|---|
| lifts in the 8 / 10 s after the switch | 0 | 2 | any step |
| base drift after the switch | 1 cm | 5 cm | |
| feet separation change (after − before) | 1 cm | 5 cm | the inward draw that failed on the robot |
| heading change | 1° | 5° | |
| base height dip | 0.5 cm | 3 cm | |
| torso angular rate, peak in the first 2 s | calibrate | calibrate | NEW: smoothness |
| joint-target jump at the switch frame, max over joints | calibrate | calibrate | NEW: the action discontinuity |
| switch wait (request → active) | 0.5 s | 3 s | the gate holding the switch |

### Test 1 — stand + full-band ladders (replaces the 12-segment staircase)
Why not the current staircase: every command segment starts from a 2 s stand, so each one is a start-up from standing, not tracking
at speed, and the band stops at 0.15 m/s. The ladder separates the two: continuous rungs measure tracking at speed, a stop at the top
of each ladder measures stopping.

1. **Stand 20 s** (the stance record: drift, yaw, lifts, separation trend, stagger, toe, jitter, torso tilt and rate, height; the
   full joint state for the stance drawing).
2. **Three axes, each one continuous ladder** (operator refinement 2026-10-30: "the 1.0 should then transition back to 0.0, and then
   go backwards with -0.05 increments. Same with vy and wz"): rungs of 0.05 up to the top, a 4 s **stop**, rungs of −0.05 down to the
   negative top, a 4 s stop; no teleport between the axes (one teleport at the start: (−3, 0), facing away from the desk). A fall:
   recorded, the robot reset on the spot (`reset_at`: the standing snapshot), the ladder continues with its next half; the rungs
   not run score 0.

| axis | up | stop | down | stop | rungs |
|---|---|---|---|---|---|
| vx | 0.05 → 1.00 m/s | 4 s | −0.05 → −0.50 m/s | 4 s | 20 + 10 |
| vy | 0.05 → 0.30 m/s | 4 s | −0.05 → −0.30 m/s | 4 s | 6 + 6 |
| wz | 0.05 → 0.80 rad/s | 4 s | −0.05 → −0.80 rad/s | 4 s | 16 + 16 |

Rung length **4 s** (operator: OK), measured over the last 2.5 s. At ~2 steps/s, 2.5 s holds about 5 steps against 3 in a 1.5 s
window; the mean's noise falls as $1/\sqrt{n}$, so √(5/3) ≈ 1.3 × steadier. 20 s + 74 rungs × 4 s + 6 stops × 4 s = **340 s** of sim.

Tracking error per rung, overshoot counted like undershoot, off-axis motion included (sideways drift and heading drift while asked
to walk straight are tracking errors):

$$e = \frac{\lVert v - c \rVert}{\max(\lVert c \rVert, c_0)}, \qquad c_0 = 0.1 \text{ m/s}$$

on the vector (vx, vy, 0.5 m × wz): a yaw rate counts as the speed of a point 0.5 m out, so heading drift on a straight rung is a
tracking error, translation during a turn rung too, and c0 = 0.1 m/s is 0.2 rad/s on a wz rung.

Score per rung s = ramp(e; g 0.1, b 0.5). Example: vx 0.10 commanded, 0.131 achieved (lm7f_kit_alive5, gain 1.31): e = 0.31,
s = (0.5 − 0.31) / 0.4 = 0.48; Exam #1 gave that rung full marks (min(1.31, 1) = 1). Tracking 20 = vx 8 + vy 4 + wz 8. The ladder
also gives the turn deadband directly: the smallest |wz| with s > 0.5.

The stops (one per ladder, from its top rung): stopping distance after the command drops, steps to settle, residual drift — into
stability.

### Test 2 — pushes (NEW, after the ladders)
The same push the walk policies train on: a whole-body velocity kick (`push_by_setting_velocity`; the walk line's ceiling is 1.5 m/s).
Body 77.3 kg, so a kick of Δv carries the impulse $J = m \Delta v$: the max level, 1.5 m/s, is 116 N·s, i.e. a 1160 N shove for 0.1 s.

| | standing (cmd 0) | walking (vx 0.3) |
|---|---|---|
| directions | front, back, left, right | front, back, left, right |
| levels Δv | 0.5 / 1.0 / 1.5 m/s | 0.5 / 1.0 / 1.5 m/s |
| per push | 2 s standing, the kick, 5 s measured | 3 s walking, the kick, 5 s measured |

Order: all 0.5 m/s pushes first (front, back, left, right), then 1.0, then 1.5 — a fall at the max level never disturbs a smaller one.
24 pushes + 3 s settle = **183 s**. The kick lands on a planned frame (`push_at`). A fall → `reset_at` on the spot, 3 s settle, the
next push (that push scores 0). Later: a torso force pulse (closer to a hand push) beside the kick.

| metric per push | g | b |
|---|---|---|
| held (no fall, no non-foot contact) | yes | no → the push scores 0 |
| recovery distance: standing = settled − pre-push base position; walking = lateral offset from the pre-push line | 5 cm | 50 cm |
| peak torso tilt | 3° | 20° |
| recovery time (tilt < 2° and speed within 0.05 m/s of the command, held 1 s) | 1 s | 4 s |
| recovery steps | 1 | 5 |
| peak foot force / body weight (stamping) | 1.5 | 3.0 |
| energy spent in the recovery | calibrate | calibrate |

### Test 3 — homing (kept, harder)
- Stop at **0.25 m** (done).
- **Starts all around the desk**: 3–5 m from the desk edge centre, any direction (360°), random heading. Behind the desk the straight
  line crosses it, so the commander plans around it: inflate the desk rectangle by the robot's radius + margin (~0.45 m); if the line
  to the pre-pose cuts it, go via the inflated corners on the shorter side. One convex obstacle → at most two corner waypoints; about
  40 lines in `homing_commander.py`. Timeout 90 → 120 s.
- **Turning by the error, not bang-bang**: today ALIGN commands ±0.8 until the heading is within 3°, then ±0.8 pulses of 0.25 s
  (the lm7f/g policies barely turn below 0.5, so a smaller command would stall). Proposal: the same proportional law the cruise uses,
  $\omega = \min(\max(k e_\psi, -0.8), 0.8)$ with k = 1.5: 30° → 0.79 rad/s, 10° → 0.26, 3° → 0.08. A policy with a deadband then
  stalls, which is the honest reading. To keep trials from hanging: no heading progress (< 1° in 3 s) → one ±0.8 rescue pulse,
  counted and penalised.

| metric | g | b | points |
|---|---|---|---|
| settled planar error (after the balance's 10 s) | 2 cm | 10 cm | 4 |
| settled heading error | 1° | 5° | 1 |
| walk placement at the switch | 2 cm | 10 cm | 1 |
| progress speed = straight distance / time to the switch | 0.25 m/s | 0.05 m/s | 2 |
| turn: ALIGN yaw-rate gain over commands ≥ 0.05 rad/s, minus rescues | 0.9, 0 rescues | 0.3 | 2 |
| a trial that does not arrive scores 0 on all of the above | | | |

### Test 5 — stumble: a rough floor (2026-11-02; OFF by default since the same evening, `exam.sh --stumble` opts in; recorded, NOT scored)

Operator 2026-11-02: the toe-drag question is answered by the scuff counts on the flat ladders (next section); a fixed rough floor would convolute
the measurement (a terrain test measures terrain walking, not clearance) and, if ever trained on, teaches the terrain rather than clearance. Kept
as an opt-in probe only.
Why: on the robot (2026-11-02) `lm7i_combo_soft` at 0.15 m/s scuffed its LEFT toe on the floor and recovered with a huge right step. Its
exam record of the morning had the number — the 0.15 rung's swing height 1.6 cm L / 2.3 cm R — but the sim floor is perfectly flat (no
bump ever meets a 1.6 cm toe) and swing height was neither a key column nor a flag. Two changes: swing height is a key column (section
"Key columns" below), and the exam walks the policy over a floor with bumps. **Evening 2026-11-02:** the same detector run on the FLAT floor
(the Test 5 script with `EXAM_STUMBLE_FIELD=0`) already found 5 of 33 swings scuffed at 0.15 and 4 of 42 at 0.30 for the same policy — the
toe drag needs no bump to be seen, it needs the 50 Hz contact stream. So the swing engine now runs on **every ladder segment** (section
"Toe drag on the flat ladders" below), the flat scuff count is the key column, and Test 5 is kept as a milder stress probe (top cap 1 cm).

**The floor** (`tools/stumble_field.py`, generated into `unitree_robots/h1_2/<scene>_stumble<seed>.xml` at every launch, gitignored,
`LOCOSW_STUMBLE_SEED` = the exam seed): **sphere bumps** on the plain plane over 14 × 12 m at x 50..64, y −6..6 — far from every other test
(the ladders walk to −x from x = −3, the pushes stay at x = −3, the homing starts within 5 m of the desk at x = 0.35), so Tests 0–3 never
touch it and their timelines are unchanged (Test 5 runs LAST). One sphere of radius 0.15 m per 20 × 20 cm cell (±8 cm jitter), buried so
that its cap stands **0.4..1 cm** above the floor (uniform; `STUMBLE_AMP` = the top, **2 cm until 2026-11-02 evening**, `STUMBLE_AMP=0.02`
reproduces that field): ~4100 bumps, 26 % of the area covered, 0.4–1 bump under a 26 × 9 cm sole, mean ground +0.2 cm at 2 cm, +0.1 cm at 1 cm.
A 1 cm cap is a doormat edge or a cable lip, not terrain: that is what the robot's workplace holds. A cap of height h on a sphere of radius R has the rim radius $\sqrt{2Rh - h^2}$ (h = 1 cm: 5.4 cm, 2 cm: 7.5 cm) and meets
the floor at the slope $\arccos(1 - h/R)$ (21°, 30°): a pebble or a pavement joint, not a wall. The first 10 cm of the field hold no bump rim
(the robot steps INTO the roughness). Deterministic per `--seed`: every number is SplitMix64(seed, cell row, cell col, salt) / 2^64 — no numpy
RNG, the same field on every host; the field's checksum (sha1 of its geometry text) is printed at launch and stored in the record
(`stumble.field.checksum`); `Field.height(x, y)` is exact (the highest cap under the point; checked against MuJoCo raycasts: 0 error).
**Checksums at seed 1:** `d774b40ebe0b` = the 1 cm field (default since 2026-11-02 evening), `7b10db2021a4` = the 2 cm field (the three
`stumble_dev` records of 2026-11-02 14:33 and 14:40). The record carries the amplitude itself: `stumble.field.h = [0.004, 0.02]` (old) /
`[0.004, 0.01]` (new), so a reader never has to guess from the checksum; Test 5 numbers across the two amplitudes are NOT comparable (the 2 cm
runs tripped 5 / 13 swings per rung for lm7i_combo_soft; the 1 cm field is read fresh below).
The sim's exam counters (`fpk`, the crawl `nfc`) now count every WORLD-body geom as floor (the plane and the bumps; the desk is its own
body) — the old test on the geom named "floor" was blind to a scuff on a bump. Step cost with the robot on the field: 92 µs vs 69 µs per
physics step in a python check; the rig's rtf was unchanged (4.1).

*Why not a height field* (`STUMBLE_KIND=hfield` keeps it for A/B): the first implementation was a MuJoCo `<hfield>` (10 cm tiles, heights
0..AMP, the edge strip flush with the plane). lm7i_combo_soft walked the same spot on the plain plane at 0.14 m/s but on the field it
stalled at the near edge after the same 9 lifts at AMP 2 cm, 1 cm AND 0.4 cm (bit-identical up to the stall, i.e. not the bumps): the
prisms' vertical side faces meet a foot that slides at z ≈ 0 with a few mm of contact penetration and act as a wall (the minimum-penetration
normal is horizontal while the overlap is thinner than the penetration). A policy whose toe drags — exactly the one this test is for — is
the one a height field traps. Spheres have no faces.

**The run** (`exam2_test.sh stumble`, 36 s of sim, 7–9 s wall measured alone at rtf 4): teleport to the field's start (base 45 cm before the near edge on
the plain plane, facing +x), 3 s settle, **vx 0.15 for 15 s, vx 0.30 for 15 s**, a 3 s stop. A fall → `reset_at` on the spot, 3 s settle,
the next rung (the fallen rung is marked). Record: `logs/stumble_<k>.jsonl` → `exam/<tag>_<stamp>.json` key **`stumble`** (older records
lack it → the table prints "—").

**Metrics per rung** (commander `_stumble_sample` / `_stumble_metrics`, from the 50 Hz foot contact `cf` (> 1 N = loaded), foot world
position `fl`/`fr`, foot world velocity `vb`, and the sim's per-foot peak floor force since the previous message `fpk`):
- a **swing** = lift-off (loaded → air) to touchdown (air → loaded) with ≥ 2 airborne samples (a single 20 ms airborne sample = a contact
  flicker, counted apart); `swings` = [L, R].
- **clearance** $h(t) = z_{foot}(t) - g(x_{foot}, y_{foot}) - z_{rest}$: the foot body height above the ground under its origin (the same
  generator: the highest bump cap under the point, exact), minus the foot's rest height from the settle on the flat. `ground_rms` = the rms
  of $h$ over the LOADED samples: a foot standing on a bump's flank reads a few mm (6–9 mm measured); a wrong mapping would read ~1 cm everywhere.
- **swing apex** = max $h$ over a swing; per foot the max (`swing_apex` = the key column), mean and MIN over the rung's swings. The rung's
  `swing_height` IS this ground-corrected apex (`swing_height_raw` keeps the flat-floor definition).
- a **scuff** = a floor-contact burst of the swing foot: `fpk` > 20 N inside a 20 ms window whose BOTH ends are airborne — a contact that
  began and ended between two samples (the toe hit a bump and came off again). The lift-off window (it holds the lift-off force) and the
  touchdown sample are excluded. `scuffed` = swings with ≥ 1 burst, `scuff_bursts` = all bursts, `scuff_pre_apex` = bursts at or before
  the apex sample.
- a **catch** = the foot arrested in flight: the planar foot speed drops by more than max(0.5 × its peak so far in the swing, 0.15 m/s)
  within one 20 ms step, at or before the apex sample (a deceleration ≥ 7.5 m/s² while still rising; a normal landing decelerates over
  ~100 ms on the way down).
- **trips** = swings with a scuff or a catch; `trips_per_step` = trips / swings.
- plus the generic segment numbers: `tilt_max` (peak torso tilt), `lifts`, `dx_body` (lateral drift) and `yaw_change_deg`, `fell`,
  `on_field_frac` (share of samples with the base inside the field: a policy that turns away walks off it).

**Proposed score (NOT applied; calibration v2 stays; superseded in part by the toe-drag item of the next section — the flat scuff share
is the first candidate, Test 5 the stress probe):** a new quality row **stumble 5** (taken from stability 30 → 28 and gait 10 → 7, so
the total stays 100) = per rung (0.15, 0.30) the mean of: trips per step ramp(g 0, b 0.5) 0.4 · the lower foot's apex ramp(g 4 cm, b 1 cm)
0.3 · peak tilt ramp(g 4°, b 15°) 0.2 · lateral drift ramp(g 5 cm, b 50 cm) 0.1; a fall in the rung = 0; both rungs equal. Example on the
morning's lm7i_combo_soft numbers (flat floor): apex min(1.6, 2.3) = 1.6 cm → s = (1.6 − 1) / 3 = 0.2 of the 0.3 → the swing alone costs it
0.24 of 0.5 points per rung before any trip. Calibration ramp: run the 26 policies of calibration v2 (`land_lm7h_r2`) once with Test 5
(≈ 15 s wall each on 10 rigs), set the two *prov* ends (trips per step, tilt) to the population's p10 → p90 with `exam_calibrate.py`
(add `stumble_*` keys to `PROV`), keep the apex ends (4 / 1 cm are the operator's clearance tolerance) and the drift ends; then **v3**.
Open before scoring: seeds per exam (one field per seed — the same seed every landing keeps waves comparable; a second seed = a second
sample of the bumps).

### Toe drag on the flat ladders (2026-11-02 evening)
**What runs:** the Test 5 swing engine (`_stumble_sample` / `_stumble_metrics`, the same code) on every segment of the ladders record — the
20 s stand, the 74 rungs of vx / vy / wz in both directions, the 6 stops. Ground = 0 (the flat plane), the clearance zero $z_{rest}$ = the
mean height of each LOADED foot over the 20 s stand after its first second (Test 5 takes it from its 3 s settle), so the apex is
$h_{apex} = \max_t (z_{foot}(t) - z_{rest})$ over one swing; the thresholds are the same (contact 1 N, scuff 20 N, catch
max(0.5 × peak, 0.15 m/s), ≥ 2 airborne samples). Example: lm7i_combo_soft stands with $z_{rest}$ = 4.2 / 4.5 cm; a swing whose foot origin
peaks at 5.8 cm reads 1.6 cm of clearance.

**Record** (`ladders.segments[].swing`, the same field names as Test 5 so one reader serves both — `exam_score.seg_swing`): `swings [L, R]`,
`swing_apex` (max per foot = the segment's `swing_height`; `swing_height_raw` = the old first-sample definition), `swing_apex_mean`,
`swing_apex_min`, `scuffed`, `scuff_bursts`, `scuff_pre_apex`, `catches`, `trips`, `trips_per_step`, `fpk_air_max`, `flickers`, and
`trip_list` = the tripped swings only (t0, dur, apex, s_peak, scuffs, fpk_air, catch; Test 5 keeps the full `swing_list` — a ladder has ~700
swings, the tripped ones are the evidence). The ladders record carries `z_rest` and `thresholds` like Test 5. Records before this change have
no `swing` key and print "—"; everything else in them is unchanged (backward compatible).

**Can old records be re-scored? No.** The operator was told "every past record can be re-scored"; that is not true for the scuffs and this is
why: the detectors need the 50 Hz contact state and the per-20 ms peak floor force of each foot, and the exam stores (a) per-segment
aggregates, (b) the commander's 10 Hz trajectory CSV (`logs/homing_traj_<k>/ladders_<stamp>.csv`, columns cL/cR, zL/zR, fpkL/fpkR but no
foot velocity) and (c) a 5 Hz decimation of (b) in the record (`ladders.rows`). lm7i_combo_soft's swings last **60–140 ms** (the flat control
run's `swing_list`: dur 0.06–0.14 s), i.e. 3–7 samples at 50 Hz and **0–1 at 10 Hz**; the 10 Hz `fpk` column holds only the LAST 20 ms window
of each 100 ms (the sim resets the peak per message), so 80 % of the force bursts are not in the file at all. Measured on the control run's
own 10 Hz CSV (`stumble_1793621621.csv`, the run whose 50 Hz truth is 5 / 33 scuffed at 0.15 and 4 / 42 at 0.30): airborne runs of ≥ 2
samples = **0 of 19 left swings and 1 of 14 right** at 0.15 (0 of 21 L, 7 of 21 R at 0.30) — the swing segmentation itself fails, before any
scuff test. A "lower-bound" re-score from these rows would report ~0 scuffs for a policy that drags its toe, which is the false reassurance
this change exists to remove, so no `exam_rescore_scuffs.py` was written. The honest route for an old record is a re-examination: the
lock-step rig repeats a seed bit for bit when no frame is lost (section 7b), so re-running a policy reproduces its ladders and adds the
`swing` fields; the `land` skill examines unexamined rows anyway. Nothing stored before 2026-11-02 evening carries a scuff count.

**Proposed quality item (NOT applied; calibration v2 stays): toe drag**, a 3-point row under gait & body care (gait 10 = impact 3 →
2, smoothness 3 → 2, toe drag 3, pivot 1, slip 1, swing 0 (absorbed), steadiness 1) = the mean over the vx +0.15 and +0.30 rungs of the
scuffed share ramp:

$$s = \min(\max(\frac{0.3 - x}{0.3 - 0}, 0), 1), \qquad x = \frac{\text{scuffed}}{\text{swings}_L + \text{swings}_R}$$

g = 0 (no swing touched the floor), b = 0.3 (nearly one swing in three drags: the robot's 2026-11-02 walk). The flat control run's
lm7i_combo_soft: 5/33 = 0.152 → s = (0.3 − 0.152) / 0.3 = 0.49 at 0.15, 4/42 = 0.095 → s = 0.68 at 0.30, mean 0.59 → 1.8 of 3 points. The
ends are a definition (a toe on the floor is the operator's tolerance), not a calibration key; the companion swing columns stay unscored
flags. A catch (the foot arrested in flight) counts in `trips` but not here: on the flat floor a catch is a policy artefact (none seen so far),
on the field it is the bump. Open: whether the two rungs weigh equally (0.15 is the homing's cruise, 0.30 the push test's walk).

**Verified 2026-11-02 16:03** (`scuff_dev_2026-11-02_16-03-06`, one unpinned rig, seed 1, n 1: **2 min 2 s** wall for the whole exam, rtf 6.7,
record `lm7i_combo_soft_2026-11-01/exam/scuff_dev_2026-11-02_16-05-07.json`, 1058 KB = 46 KB more than the morning's record; the ~1 MB is the
5 Hz rows + pushes, not the swing fields). lm7i_combo_soft on the flat ladder: $z_{rest}$ 4.23 / 4.14 cm; **scuffs .15 = 0/8, scuffs .30 = 1/11**,
swing .15 1.8/2.4 cm, swing .30 2.1/2.5 cm; the neighbouring rungs: vx 0.10 **4/8**, 0.20 1/11, 0.25 2/11, 0.45 1/11; pooled over vx 0.05–0.50
**9 of 96 swings (9.4 %)** — the order of the 15 s flat control (15 % at 0.15, 9.5 % at 0.30). A 4 s rung holds only 8–11 swings, so one rung's
share is a coarse estimate: with a true share p = 0.15 the chance that 8 swings show none is $(1-p)^8$ = 0.27, so a "0/8" is no clearance
certificate; read the two columns together with the neighbours (fleetdeck can show the per-rung `scuffed`), or pool 0.10–0.30 (the walking band
of the homing and the push test) if the operator prefers one number. Two details of the record: (1) `swing_height_raw` (the old column) reads
0.3–0.7 cm on rungs whose foot was airborne at the rung's first sample (the old zero was that sample) while the apex reads 1.7–1.8 cm — the
morning's "swing 1.6" was this definition, the apex vs $z_{rest}$ is the right one; (2) above ~0.6 m/s the bursts are 300–370 N with an
apex of 3.3–3.7 cm and come AFTER the apex (`scuff_pre_apex` 0): a touchdown bounce (the foot struck and left the floor inside one 20 ms
window), not a drag — at 0.15 / 0.30 the apex is 1.5–2.5 cm and the bursts sit at or before it. Test 5 at 1 cm (field `d774b40ebe0b`): 0.15 rung
v 0.028 m/s (the policy barely moved off the start in this run: 8 swings, 1 scuffed, tilt max 4.7°), 0.30 rung v 0.208, **10 of 44 swings
tripped**, apex max 12.3 / 6.9 cm, tilt max 17.4° — the milder field still trips this policy one swing in four at 0.30.

### Key columns (2026-11-02)
`EXAM.md` prints, beside the score: `swing .15 L/R cm` and `swing .30 L/R cm` — the FLAT ladder's peak foot lift per foot on the vx +0.15
and +0.30 rungs (`ladders.segments[].swing_height`; the operator's first flag: below ~2 cm a toe meets any threshold) — **`scuffs .15` and
`scuffs .30`** (evening) = the FLAT ladder's scuffed swings / all swings (L + R) on the same two rungs, e.g. "5/33" (`exam_score.ladder_scuffs`,
`ladders.segments[].swing.scuffed` / `.swings`; the number the robot's toe drag would have shown) — and the Test 5 columns `stumble .15` /
`stumble .30` = "apex L/R cm · trips · peak tilt°" per rung, and `stumble falls`. Old records print "—". `tools/exam.sh` echoes the two
key rungs' `sw` lines per policy as they come in.

## 4. Stability, gait and energy metrics (all phases)
Stability 30 = stands 12 + walking torso 6 + handover 12.

| metric | where | g | b |
|---|---|---|---|
| drift, heading change, lifts | the 20 s stand + the stops | 1 cm, 1°, 0 | 10 cm, 10°, 3 |
| torso tilt rms, torso angular rate rms | stands; rungs ≤ 0.5 m/s | calibrate | calibrate |
| feet separation trend, stagger, toe angle | stands | 0, 1 cm, 2° | −3 cm, 8 cm, 10° |
| knee jitter | stands | 0.2 °/sample | 1.0 °/sample |
| height sag vs the balance policy's stand (Test 0) | stands + rungs | 3 cm | 15 cm |

The sag example: still_alive10 at vx 0.05 stands at 0.930 m against the balance's ~0.99 m, a 6 cm sag: s = (15 − 6) / 12 = 0.75 on a
1-point metric, a quarter point off — the "small deduction" the operator asked for.

Gait & body care 10: foot impact (the 95th percentile of the vertical foot force at touchdown, ÷ body weight; g 1.2, b 2.0) 4 · action
smoothness (rms of the joint-target change per policy step) 3 · foot sliding (pivot index, slip speed of a loaded foot; g 0.05,
b 0.5 rad/s) 2 · swing height inside 3–10 cm 1. "Action std": the deployed policy is deterministic (no std at deploy; the std is a
training quantity in wandb) — the action rate is what the rig can see.

Energy 10: mechanical power $P = \sum_j |\tau_j \dot q_j|$ over all joints, plus the $\tau^2$ term (motor heating; standing does
almost no mechanical work but still burns current). Walking: the cost of transport at the 0.3 and 0.5 m/s rungs,

$$\mathrm{CoT} = \frac{\bar P}{m g v}$$

e.g. 200 W at 0.5 m/s: CoT = 200 / (77.3 · 9.81 · 0.5) = 0.53 — 6 points. Standing: mean $\tau^2$ over the 20 s stand — 4 points.
Reported per phase (stand, each ladder, pushes, homing) on the page.

### As implemented (tools/exam_score.py `score_v2`, 2026-10-30) — the source of truth; fleetdeck shows every row
| quality | metric (weight) |
|---|---|
| survival 10 | 10 × 0.5^falls over Tests 0, 1, 3 (0 if Test 1 did not run) |
| stability 30 | 20 s stand 7: drift 1.5, heading 1, lifts 1.5, torso rate rms 1 *(prov)*, separation trend 0.5, stagger 0.5, toe 0.5, knee jitter 0.5 · stops 5: per stop the mean of settle time (1 → 4 s), steps (1 → 5), drift in its last 1.5 s (0.5 → 5 cm), overshoot (10 → 60 cm; ω: 5 → 30°) · walking (|v| ≤ 0.5) 6: tilt 2.5, rate rms 2.5 *(prov)*, sag vs the balance stand 1 · handover 12: per switch steps .25, drift .2, inward draw .15, turn .1, dip .1, jolt .1 *(prov)*, target jump .05 *(prov)*, wait .05 |
| tracking 20 | vx 8, vy 4, wz 8: mean rung score (missing rungs = 0) |
| gait 10 | touchdown impact p95 / body weight 3 (1.5 → 3.5 *(prov)*: the per-frame peak on the rigid sim floor ran 2.4 BW in the alive5 smoke, above the first guess 1.2 → 2.0) · action roughness 3 = rms of the joint-target SECOND difference, max joint (0.03 → 0.10 rad *(prov)*; the first difference counted the gait's own swing, ~0.1 rad per step) · pivot 1 · slip 1 *(prov)* · swing 3–10 cm share 1 · speed steadiness 1 *(prov)* |
| energy 10 | cost of transport at vx 0.3 / 0.5: 6 *(prov)* · 20 s stand mean Σ τ²: 4 *(prov)* |
| pushes 10 | mean push score over 24 (a fall = 0; per push the mean of distance 5 → 50 cm, tilt peak 3 → 20°, recovery time 1 → 4 s, steps 1 → 5, foot peak 2 → 6 × body weight — the smoke's recoveries peaked 2.9–5.4 BW) *(prov)* |
| homing 10 | per trial (not arrived = 0): settled 4, heading 1, at the switch 1, progress speed 2, turn gain 2 minus 0.1 per rescue pulse |

### As implemented 2026-11-01 (the operator's three focus items made visible; calibration v2-2026-11-01) — replaces the rows above where they differ
Operator 2026-11-01: counter-sway after pushes, arms "like pendulums", chaotic recoveries — "We need all of those in the exam to be seen";
energy "for all test phases". New raw values (commander `homing_commander.py` + `exam_score.raw_values`):
- **push counter-sway:** the lean along the push direction from the signed roll / pitch minus the pre-push mean,
  $L = (\theta - \theta_0)\cos a - (\phi - \phi_0)\sin a$ for a push from direction $a$; the first excursion = the largest $|L|$ within 1 s of the
  kick, the counter-sway = the largest lean to the other side after it. Also torso roll/pitch rate rms after the kick (sway), the feet
  separation's largest change (coordination), the arms' largest move from their pre-push pose, the mechanical energy of the recovery.
- **arm sway:** mean over the 14 arm joints of the angle std, over the 20 s stand, the stops and the walking rungs ≤ 0.5 m/s.
- **all-phase energy:** mean Σ τ² and mean Σ |τ q̇| per second over the segments of Tests 0–2.

| quality | change |
|---|---|
| stability 30 | stand 6 (drift 1.25, heading 0.75, lifts 1.25, rate 1, separation trend 0.5, stagger 0.5, toe 0.25, jitter 0.5) · stops 4 · walking 5 (tilt 2, rate 2, sag 1) · handover 12 · **arm sway 3** (0.3 → 3.0° *(prov)*) |
| energy 10 | **all phases Σ τ² 2** (3000 → 9000 *(prov)*) · **all phases mechanical power 2** (100 → 350 W *(prov)*) · **push recovery energy 2** (200 → 1000 J *(prov)*) · cost of transport 2 · stand Σ τ² 2 |
| pushes 10 | per push: tilt .15, **counter-sway .15** (1 → 8°), **sway rate .10** (.15 → .6 rad/s), steps .15 (2 → 6), **feet separation .10** (2 → 12 cm), recovery time .10, distance .10, foot peak .05, **arm peak .10** (3 → 25°); all *(prov)* |
| survival 10 | 0 when Test 1 did not run (a never-granted switch = `FAIL_switch_start`, now after 10 s: operator "max 10 seconds") |

Calibration **v2-2026-11-01** (`results/exam2_calibration.json`): the 25 policies of `land_lm7h_r2_2026-11-01_11-22-18` (lm7h 11 + lm7g 12 +
lm7f 2; lm7g_energy_cot's record of that run was broken -- its switch was refused -- and excluded). Examples: arm sway 1.16 → 1.96°,
counter-sway 0.32 → 5.78°, push feet-separation change 11.8 → 55.3 cm (was DEAD: 89 % at one end), push energy 577 → 1846 J, all-phase τ²
7247 → 12190 (was DEAD), all-phase power 95.5 → 165 W. `walk_sag` stays DEAD (operator: "1 cm sag is completely fine").

## 5. Stance drawing
Possible, two levels:
- **Top view (done):** the two soles at true size (26 × 8.6 cm) from the recorded separation, stagger and toe angles.
- **Front / side view (Exam #2):** record the full joint state through the 20 s stand, average it, run MuJoCo forward kinematics and
  draw the robot as a stick figure (or render the model in that mean pose), with the 20 s spread as a ghost and the support polygon +
  CoM projection in the top view.

## 6. Noise floor and calibration

### Calibration v1-2026-10-31 (done; `tools/exam_calibrate.py`, `results/exam2_calibration.json`)
What it is: every metric scores through a ramp between a good value g and a bad value b; a guessed pair that every policy beats (or
fails) gives everyone the same points and measures nothing (before v1: the handover target jump scored 0 for 64 % of switches, push
distance 0 for 52 % of pushes, speed steadiness full marks for 54 % of policies). The calibration reads the raw values of many exam
records (`exam_score.raw_values`) and re-sets ONLY the provisional keys (`exam_score.PROV`, the spec's guesses) to the population's
10th -> 90th percentile, never narrower than half the median (a tight population -- walking torso rate 0.17-0.21 rad/s -- would
otherwise turn a 20 % difference into the whole weight). Every other key is a definition (the operator's tolerances, physical
meaning) and is kept; the tool flags it DEAD when > 80 % of the samples sit at one end -- v1: `walk_sag` (all 26 policies sag < 1 cm
below the balance stand: 100 % full marks), left for the operator. v1 = 26 policies (land_lm7h: lm7h 11 + lm7g 13, + lm7f alive5 /
gforce_airtime). Every score carries its version (`meta.score.calib`) and the table it used (`meta.score.cal`); `--apply` re-scores
stored records (the raw data stays; scores are recomputable). Effect: scores +0.5..+3, order nearly unchanged.

### Noise floor (measured 2026-10-31)
Only homing depends on the seed. 4 policies x seeds 1-3 (homing-only runs, tools/homing_seed.sh): the homing points (= the total)
move by sd 0.43 on average (0.12-0.73), the largest range 1.7 (lm7g_still_alive10). **Differences under ~1 point are ties.**
The rest repeats bit for bit only when no policy frame is lost (section 7b); the land_lm7h run lost one frame on 9 of 24 rows.
The lock-step rig repeats a seed bit for bit, so re-running the same seed shows no noise; the noise is in **which starts the seed
draws**. One seed = one sample of 10 homing starts. Example: lm7f_kit_alive5's settled errors (4.3, 8.6, 7.2, 9.3, 8.6, 6.2, 1.5,
5.0, 12.9, 6.9 cm) have a mean of 7.05 cm and a std of 3.1 cm; the standard error of the mean is $\sigma/\sqrt{n}$ = 3.1/√10 = 1.0 cm.
Exam #1's placement = 20 (1 − ē / 10 cm), so 1 cm = 2 points: two policies 2 points apart on placement are a coin toss.

### Run-to-run spread (measured 2026-11-01) — larger than the floor above
The same 10 policies ran twice on the same exam timeline (`speed10_2026-11-01_12-23-37` on the old wire, `compact10_2026-11-01_12-40-02`
on the compact wire, which is bit-identical physics -- section 7c): the totals moved by a mean |Δ| of 2.75 points, sd of the paired
differences 3.2 → **one run's sd ≈ 3.2 / √2 = 2.3 points** (lm7h_a_h4_still 56.1 → 59.2, lm7g_fric 52.9 → 57.1, lm7h_b_h3_nostep 54.3 → 49.4).
Where it comes from: two compact rigs with the same policy traced frame by frame are IDENTICAL for 64.28 s, then the first difference is in
the command (ctrl), not the state, 1.8 s after the walk → balance switch of Test 0: the arm targets (`/BridgeModule/joint_set_arms`) are
delivered by wall-clock arrival, not stamped to a sim frame, so once their value changes it lands one or two frames apart between runs. The
contact dynamics of walking amplify that difference until every later test starts from a different state. Typical consequence:
lm7h_b_h3_nostep's 20 s stand turned 107° (r2: 1.4°) in 2 of 3 runs, it then walked the vx rungs into the desk (tracking 9.4 → 5.8).
Meaning: one run is one sample of the policy's outcome under tiny timing differences; two policies less than ~2 × 2.3 × √2 ≈ 6.5 points
apart in single runs are not separable at ~95 %; the mean of 2 runs has sd 2.3 / √2 = 1.6. Frame-stamping the arm stream would make
re-runs bit-identical (reproducible) but would not remove the spread: any other tiny change moves the outcome the same way.

Calibration run (once, ~1 h on 8 rigs): 4 reference policies (gforce_airtime, still_alive10, footbonus6_still, alive5) × 3 seeds →
(a) the g/b values from the real distributions, (b) the total score's spread across seeds = the noise floor, printed as score ± floor
on every exam; differences inside the floor are ties.

## 7. Instrumentation (the rig) — implemented 2026-10-30
- Per-step export from the simulator (the existing sim JSON that carries fl / fr / fyaw / qj): joint torques and velocities (all 27),
  vertical and total contact force per foot, non-foot floor contacts, torso orientation and angular velocity, joint targets.
- A push hook in the simulator (like the teleport): `push DVX DVY` → add Δv to the free joint's linear velocity at a planned frame.
- Commander: `stand 20`, the ladder items, push items, the P-law ALIGN with the rescue counter, the around-desk waypoints, 360° starts.
- `exam_score.py` v2 (qualities + ramps), `meta.exam_version = 2` (Exam #1 and #2 scores do not compare; re-examine the reference set).
- fleetdeck: Test 0 and push sections, the quality breakdown, energy per phase, the front/side stance drawing.

Where it lives:
- sim (`simulate/src/main.cc`): `push_at <t> dvx dvy`, `reset_at <t> x y yaw` (the standing snapshot taken at the first teleport of a
  standing robot), per-frame counters in rt/sim_base_pose: `en` = [Σ|τ q̇| dt J, Σ τ² dt], `fpk` = per-foot peak floor force since the
  previous message, `nfc` / `nfb` = frames / the last non-foot body on the floor (> 5 N), `tq` / `dq` / `qt` = torques, joint
  velocities, lowcmd joint targets.
- commander (`MovementModule/Utils/homing_commander.py`, branch MM_loco_switch_homing): `ladders` / `pushes` queue items (one segment
  engine with the staircase), the exam #2 per-segment metrics, fall → reset → continue, `HOMING_TURN_LAW=p` (+ `HOMING_K_TURN`,
  `HOMING_RESCUE_S`, `HOMING_RESCUE_DEG`), `HOMING_DESK="depth width"` (around-desk planning), a fall in a homing trial
  (`FAIL_fell`, then a reset start), trial energy + cost of transport + turn gain.
- tools: `exam2_test.sh ladders|pushes`, `homing_trials.sh --ring`, `exam.sh --exam 2` (default; sets HOMING_TURN_LAW=p,
  HOMING_DESK="0.8 1.2", HOMING_TIMEOUT_S=120), `stance_fk.py` (forward kinematics of the 20 s stand, run by `exam_record.py` with a
  python that has mujoco), `exam_score.py score_v2`, `exam_report.py --exam 2`, `land_table.py` (both versions).
- fleetdeck `exam2.py`: score (qualities + every metric), Test 0 scorecard + traces, ladders (tracking + rung scores, stops, power),
  the stand (front / side / top drawing + footprint), stance over the ladders, pushes (grid, paths, tilt), homing (route corners,
  turn gain, rescues, energy).

## 7b. Rig speed and the lock-step stall (2026-10-31)
- **The stall:** about 1 policy frame in 20 000 (2–3 per exam policy) the Bridge setter got no MovementModule command and waited
  `BRIDGE_LOCKSTEP_WAIT_S` = 120 s. The MovementModule detector named the cause on the first catch:
  `no MovementModule command for policy frame stamp 29180` + `[MovementModule] LOCK-STEP: policy frame(s) NOT SEEN between stamp 29160
  and 29200` = the frame's **joints_imu sample never reached MovementModule** (best-effort KEEP_LAST(1) ROS read), so it never stepped.
- **Fixes (Aspired_Robot_Project-homing):** the Bridge conductor re-sends the latest joints_imu after 20 ms of silence in lock-step
  (MovementModule skips a stamp it has seen; a lost sample now costs 20 ms, same data → deterministic); the setter waits 120 s only until
  MovementModule's first command (bring-up stays deterministic), then at most `BRIDGE_LOCKSTEP_RUN_WAIT_S` = 3 s; the two 2-slot queues
  (joints_q, joint_cmd_q) evict the OLDEST item when full in lock-step (they dropped the newest).
- **Speed:** the "25 % slower" reading was CPU pinning, not code: the same Test 0 on one rig ran at rtf 1.96 pinned to 4 threads
  (`exam.sh` default) and **2.80 unpinned** (`--no-pin`, as `land` runs); the exam counters on / off (`SIM_EXAM_COUNTERS=0`) changed
  nothing (rtf 1.96 vs 1.97, non-policy frames 747 vs 744 µs).
- **Wall time:** one policy, full Exam #2 with n 10 homing = 742–746 s on a pinned rig (2 rigs in parallel, 2026-10-31); 24 policies
  on 8 unpinned rigs = 59 min 39 s (rtf ~1.4 per rig; ~15 min per policy per rig); one policy alone unpinned = 11 min.
- **Bring-up deadlock (fixed 2026-10-31):** every rig launch lost up to 120 s: the sim published its first lowstate once, before the
  stack's container subscribed, then both sides waited for the 120 s fallback. The sim now re-publishes the unchanged state every
  200 ms until the first command (no step; deterministic).
- **The stall, second cause (fixed 2026-10-31):** the lost sample can also be the lowstate on the Bridge GETTER's reader (the conductor's
  re-send then repeats the previous frame): the sim re-publishes the unchanged state every 50 ms while it waits for a command; the
  setter ignores a repeat of the frame it answered. Setter cap: 10 s, and only after 100 answered frames (a 3 s cap cut off slow
  answers under 8 rigs at the bring-up and at the first exam item = a wall-clock-dependent run).
- **A refused switch (fixed 2026-10-31):** lm7g_energy_cot's balance switch was never granted under 8-rig load (alone it was) and the
  commander's queue waited forever (rig 4 lost 45 min). The scheduler now fails such an item after 30 s of sim (`FAIL_switch_start`
  record, the driver moves on); a Test 1 that never ran scores survival 0. Open: MovementModule's switch gate looks wall-clock
  dependent (granted alone, refused under load).

## 7c. Rig speed 2026-11-01 (operator target: "10 policies running in parallel and all take 10 minutes")
- **Measured** (10 unpinned rigs, old code, per process): Bridge conductor 86 % of a core (a 0.2 ms spin poll), setter 72 %, getter 36 %,
  MovementModule 25 %, sim 21 %. Per 2 ms frame the sim waited 0.81 ms for the setter on the 9 non-policy frames and 2.9 ms on the policy
  frame; the setter's Python DDS work per frame was LowState deserialize 348 µs + CRC 86 µs + LowCmd serialize 184 µs.
- **Fixes (Aspired_Robot_Project-homing):** the conductor sleeps on a guard condition fed by a joints_q pump thread (spin cap 20 ms);
  MovementModule spin 20 ms; the getter forwards only policy frames in lock-step; the setter treats only a NEWER tick as a new frame
  (start-up 120 s → 2 s), blocks on its command queue instead of 0.5 ms polls, skips the CRC in lock-step (the sim never checks it); switch
  timeouts 10 s. **unitree_mujoco:** the standing snapshot at sim 10 s; `exam.sh` TRIAL_T0 15 (was 40), `--no-milestone`, launch stagger 3 s.
- **The compact wire** (`LOCOSW_COMPACT=1`, default in lock-step; `unitree_sdk2_bridge.h` + setter): every frame the sim publishes
  `rt/lockstep/state` = hex of uint32 tick + float32 q[27] and the full `rt/lowstate` only on policy frames (the only ones the getter
  forwards); the setter answers on `rt/lockstep/cmd` = hex of tick + q + tau, written onto the last full `rt/lowcmd` (kp / kd / dq stay; a
  full lowcmd whenever the setter's constants change). ~6 µs each way instead of 348 / 270 µs. **Proof:** `tools/wire_check.sh` runs the
  old and the compact wire side by side with SIM_TRACE: identical over 39 358 frames (78.7 s); 2 rigs: rtf 3.12 → 6.31.
- **Result:** 10 policies on 10 rigs: 13 min 29 s on the old wire (rtf 1.70 each, slowest policy 809 s) → **7 min 12 s** on the compact wire
  (rtf 3.2 each, slowest 421 s; all handovers done 72 s after the start). CPU per rig now: setter 40 %, getter 14 %, conductor 27 %,
  MovementModule 28 %, sim 34 %, commander 19 %. The policy frame (~3.0 ms at 10 rigs, 1.3 ms at 2) is the next lever; homing trials that
  time out (120 s each) set the slowest policy's time.

## 7d. The arm gravity feedforward runs in every rig (2026-11-03)
Since 2026-11-03 `tools/exam.sh` starts the robot's arm gravity feedforward sender in every rig (`ActionModule/Utils/arm_ff_only.py`
= `robot/gravity_ff.GravityFF`, pinocchio $g(q_{measured})$ for both arms on `/BridgeModule/arm_tau_ff` at 30 Hz, the Bridge setter
clips at 25 Nm per motor), and the Bridge conductor keeps that ff while a walk stream owns the arm targets (Aspired `af7e173` on
MM_loco_switch, cherry-picked to MM_loco_switch_homing; until then the ff was dropped with the targets and faded to 0 after 0.5 s). The
exam therefore runs the plant the robot runs: the firmware/sim law is $\tau = \tau_{ff} + k_p e - k_d \dot q$, so with the ff on
the kp-40 PD no longer has to manufacture gravity torque from position error (the elbow at the default pose carries $\tau_{ff}
\approx -4.4$ Nm, i.e. 0.11 rad of sag at kp 40 that the walk rows since lm7j never saw in training). Records before this date are
PD-only-arm records; `EXAM_FF=0` reproduces them. The exam log prints `arm ff ON` per rig with the first sample and, at the end of each
policy, `arm ff: applied N reverted M` (a revert means the sender lapsed; expected 0); `ff_<policy>.log` keeps the sender's log and the
Bridge's ff / MUTED / LIVE events. The sender is wall-clock (30 Hz) like the arm-pose stream, so it belongs to the run-to-run spread of
section 6, not to the bit-for-bit repeat.

## 8. Training band (recommendation, separate decision)
Today's walk box: vx (−0.8, 1.2), vy ±0.3, wz ±0.5 or ±0.8. Proposal: vx (−0.5, 1.0), vy ±0.3, wz ±0.8. Homing never asks more than
0.5 forward or −0.15 backward; fast backward walking is the riskiest motion (no view behind, a backward fall is the worst fall).
Commands are drawn uniformly over the box, so the share of training spent inside |vx| ≤ 0.5 rises from 1.0/2.0 = 50 % to
1.0/1.5 = 67 %. The exam band follows the training band.

## 9. Decisions
Settled 2026-10-30: 1 quality-based score — yes · 2 the weights — yes · 3 rung 4 s — yes (+ the up / stop / down ladder) · 4 push
levels and walking at vx 0.3 — yes · 5 P-law turning with counted rescues — yes · starts around the desk + planning — go-ahead ·
the stance drawing — go-ahead.
Open: 6 the training band change (section 8, a training decision) · 7 seeds per exam (1 or 2).

## 10. Exam #2 v3 (2026-11-03): the clean-engagement 60 s stand, hard fails, measured-only rows
Score version 3 (`tools/exam_score.py score_v3`; `score_v2` stays callable, fleetdeck reads the same `meta.score` keys — new entries
only: `v3`, the extra metric rows, hard-fail entries in `falls` as `[test, "criterion: ..."]`). `meta.arm_ff` records the plant
(1 = the arm gravity feedforward sender ran, section 7d; 0 = PD-only arms; absent = a record before 2026-11-03). The validation runs
of this section ran WITH the ff (the default since 2026-11-03); the probes that found the teleport artifact ran without it
(the hand-launched homing rig has no sender) — the artifact is independent of the plant.

### 10.1 The 20 s stand was a rig artifact: the teleport under a latched yaw-hold
Every lm7j ladder record (land_lm7j, 2026-11-03 morning) showed the stand rotating with both feet loaded (yaw −68° clear, +59°
stopramp with 7 lifts, +151° stance, +149° combo_ff; torso rate rms 0.096–0.145 against 0.027–0.054 in the handover's clean
`walk_after` stands of the same policies). Cause, read in the rig logs: the ladders item requested the walk policy, waited 1 s, then
teleported the robot to (−3, 0, 180°). MovementModule's heading hold (`ARCHB_YAW_HOLD`, latched after 0.3 s of wz = 0) kept its
pre-teleport reference (yaw ≈ 0), so after the 180° teleport the hold commanded wz = clip(1.0 · (−3.06), ±0.4) = **−0.40 rad/s for
the whole 20 s stand and into the first vx rungs** (the error reached 0 only ~40 s later, at the 0.30–0.35 rung) — the MM log line
`yaw-hold err=-3.055 rad -> wz=-0.40` sits one line after `trial: teleport`. The rotation's sign per policy is the wrapped error's
sign (clear −, stopramp +). The same latched hold poisoned the push test's teleport (−3, 0, 180) and every homing trial's start.
Reproduced at 60 s under the old protocol (no guard, hand rig, ff off): clear −192° / 7 lifts / 0.35 m drift, stopramp +158° /
19 lifts / 0.47 m.
Fixes (both): (1) MovementModule `_apply_yaw_hold` releases the latch on a heading jump > `ARCHB_YAW_HOLD_JUMP_RAD` (0.5 rad)
between two policy steps and re-latches 0.3 s later (no real yaw rate reaches 25 rad/s; an IMU glitch costs a 0.3 s re-latch; 0 = off);
this un-poisons the pushes, homing and stumble teleports without changing their planned frames. (2) The ladders start from a
**clean engagement at the spot** (operator's protocol, `HOMING_LADDER_ENGAGE=balance`, default): the balance policy is requested,
the teleport happens under it, a measured `bal_settle` segment (5 s), the switch to the walk, and the **stand runs 60 s from the
switch frame** (`HOMING_STAND0_S` 60, operator: "increase the stance window check to a minute"). The ladders follow unchanged.
The record carries `engage`, `stand_s`, `switch_wait_s`. `HOMING_LADDER_ENGAGE=walk` reproduces the old protocol. Sim time per
ladder test: 340 → ~387 s.

### 10.2 The clear "stall" at vx ≥ 0.35 was the same artifact
lm7j_clear's land record: tracks 0.05–0.30, then v_true ≈ 0 with lifts [0, 0] and a 15–24° mean tilt at every rung 0.35–1.00 (the parent
tracked 0.35–1.00 at gain 0.95–0.99 on the same rig). Probe through the ladders engine from a clean engagement (`script`, ff off):
0.30 → 0.294, 0.35 → 0.362, 0.40 → 0.409 (13/13 lifts in 10 s), 0.60 → 0.597 (14/13), and from a fresh stand straight to 0.40 →
0.400, 0.60 → 0.596. No stall: the policy tracks; the exam protocol caused it (the residual −0.1 rad/s hold command and the
unwound stance at the 0.35 rung). Control stopramp on the same script: 0.356 / 0.556 at 0.40 / 0.60 (gain 0.89–0.93).

### 10.3 wz column spread
`track_wz` over the 8 lm7j rows (land_lm7j): 0.51–1.07 of 8 points, spread **0.57 pt** — the column carries no information
(no policy turns below 0.8 rad/s; at 0.8 they turn at gain ~0.5: "turn = drift + restep"). Proposal, NOT implemented: replace the
32-rung wz ladder's score by (a) a **turn deadband** metric = the smallest |wz| rung with s > 0.5 (ramp g 0.2, b 0.8 rad/s), (b) the
**0.8 rad/s turn quality** = the mean rung score at ±0.8 (as now), and (c) a **walking turn** = wz ±0.3 while walking at vx 0.3 (an
arc; the policies do turn while walking — the homing's arc law uses exactly this), scored by the rung error; 8 points as 2 + 3 + 3.
The in-place rungs 0.05–0.75 stay recorded (the deadband reads them) but no longer each cost a point.

### 10.4 Hard fails (binary, operator-approved; a hard fail = that window counts as a fall: the rung / push / trial / switch scores 0,
ONE `falls` entry per (test, criterion), the record keeps every raw number)
a. **Height**: $z_{stand}$ = the walk's mean base height over the clean 60 s stand. Walking (ladder rungs + stops, the handover's
   `walk_after`, the homing trials): FAIL if the 1 s running mean $\bar z_{1s} < z_{stand} - 0.10$ m at any time, or any sample
   $z < z_{stand} - 0.25$ m; a push recovery: FAIL only if a sample $z < z_{stand} - 0.40$ m; FAIL if $z_{stand} < 0.90$ m. Data
   (34 walk policies): stand z 0.988–0.994; walk drop median 6.7 / p90 9.6 / max 18.5 cm; push drop median 15.3 / p90 19.7 / max
   30.1 cm. Worked: stopramp stands at 0.988 → a rung fails below $\bar z_{1s}$ = 0.888 m. A floor only, no bonus above (operator:
   straighter legs are not to be rewarded). Per window the commander records `z_min` (50 Hz sample) and `z_run_min` (1 s mean).
b. **Tilt**: training ends an episode at 0.8 rad = 45.8°: tilt ≥ `TILT_FAIL_DEG` in any window of any test = a fall. ONE constant
   (`exam_score.TILT_FAIL_DEG`, env `EXAM_TILT_FAIL_DEG`, `exam_record.py --tilt-fail-deg`); the peak tilt per test is a weight-0
   row (`tilt_peak`) and a report column so the operator can lower it (lm7j land: ladders 17–26°, pushes 27–28°, handover ~2°).
c. **Contacts**: any non-foot body on the FLOOR (the world body) with > 20 N = a fall. The sim's counter now publishes the peak
   non-foot floor force since the previous pose message with its body (`nfp`, `nfpb`); the commander converts > `HOMING_CRAWL_N`
   (20, was the 5 N frame counter) into the fall + reset and records `floor_contact {body, t, peak_N}` per window. Arm **self-contact**
   is MEASURED (no fail): an arm body (shoulder..wrist/hand) against a body outside its own arm (torso, pelvis, legs, the other arm)
   — `sc` = [peak N, body A, body B] per message, `scd` = cumulative seconds above `SIM_SELF_CONTACT_N` (20 N); per window
   `self_contact {peak_N, pair, t, dur_s}`; scored row `sc_dur` (gait, 1 pt, prov g 0 / b 2 s). Intra-arm pairs never count, so the
   Isaac thumb/palm hull defect has no MuJoCo twin anyway: the exam body's hand is ONE geom inside `*_wrist_yaw_link` (28 bodies,
   no finger bodies) and MuJoCo never collides geoms of one body — at rest the probes read `sc` = 0 N. First reads: stopramp's
   `torso_link vs left/right_shoulder_roll_link` 89–153 N for 0.03–0.06 s at the 0.30 rung and the stops (the arm swings into the torso
   during a stop); clear 0.
d. **Joint limits**: MEASURED only — `jm` = per-joint distance to the nearer limit (rad) in every message; per window the share of
   samples within `HOMING_LIMIT_DEG` (2°) per joint, `joint_limit {frac[27], worst[3]}`; the scorer aggregates walking (rungs +
   stops, duration-weighted) and standing into the weight-0 rows `jl_walk` / `jl_stand` naming the worst joints. A fail here would
   sink every policy (ankle roll / hip yaw are held at their limits by design).
e. **Arms**: per window the max-over-joints |q − q_default| per arm (default = shoulder pitch 0.4, elbow 0.3, the rest 0 — the
   handover record's `arms`), mean and peak (`arm_dev_deg`); scored rows `arm_dev_mean` (g 2 / b 15°, 0.5 pt) and `arm_dev_max`
   (g 5 / b 30°, 0.25 pt), prov. FAIL if a wrist stays ABOVE its shoulder joint (world z of `*_wrist_yaw_link` vs
   `*_shoulder_pitch_link`, from the sim's `lw`/`rw` + `xb`) or ACROSS the body midline (the wrist's y in the base frame on the other
   side) for > 0.5 s in a walk-policy window (`arm_up_s`, `arm_cross_s` per arm). `arm_sway` stays.

### 10.5 The 60 s stand's new rows (stability, prov)
`sep_dev_max` = max |sep(t) − sep₀|, `sep_dev_end` = |sep(end) − sep₀| (sep₀ / sep(end) = the first / last second's mean), the same for
the fore-aft stagger (`stag_dev_max`, `stag_dev_end`); `stand_sep_trend` stays. Ramps g 0.5 cm / b 5 cm (sep max), 0.5 / 4 (sep end),
0.5 / 6 and 0.5 / 5 (stagger). Weights 1.0 / 0.5 / 0.5 / 0.25. Stability keeps 30: handover 12 → 10, arms locked 3 → 2; gait keeps
10: smoothness 3 → 2, self-contact 1.

### 10.6 Validation (examdev, 3 rigs, seed 1, ff ON, 2026-11-03 afternoon)
`bash tools/exam.sh --parallel 3 --no-pin --seed 1 --n 10 --tag examdev lm7j_stopramp lm7j_clear lm7j_roll`: 5 min 40 s wall (rtf 5.6–6.8),
ladders 386 s of sim each, switch waits 0.34–1.02 s, no falls, **no hard fail tripped**. The 60 s clean-engagement stand against the
operator's reads:

| policy | yaw° | drift cm | lifts | rate rms | sep₀ → mean cm, dev max·end cm | stagger dev max·end cm | z stand | read |
|---|---|---|---|---|---|---|---|---|
| stopramp | +3.8 | 1.1 | 0+1 | 0.019 | 34.4 → 30.2, 6.1·5.1 | 4.2·1.5 | 0.994 | "good default stance, still" — yes (the feet draw 5 cm inward) |
| roll | −22.0 | 4.6 | 0+8 | 0.031 | 33.4 → 40.7, 17.2·13.1 | 31.0·20.9 | 0.988 | "starts a split, feet slide apart" — yes (8 right re-steps) |
| clear | +20.2 | 7.7 | 0+2 | 0.056 | 34.8 → 32.4, 8.4·4.8 | 28.8·27.7 | 0.987 | pivots 20° and staggers 29 cm (one foot walks forward) |

Before (land_lm7j, 20 s teleported stand): stopramp yaw +59° / 7 lifts / rate 0.096, clear −69° / 5 lifts / rate 0.112. The bal_settle
segment under the balance policy reads sep dev ≤ 0.5 cm, stagger dev ≤ 1.3 cm, yaw ≤ 2° in 5 s (the reference the stand starts from).
vx tracking: stopramp 0.86 (land 0.78), clear 0.81 (land 0.35: the stall is gone — 0.35 → 0.370, 0.40 → 0.418, 0.60 → 0.599, 1.00 →
0.968), roll 0.83. Measured-only rows: height drop walking 3.7–4.5 cm (1 s mean), in a push 13.3–19.0 cm (sample), both inside the
bounds by a margin; peak tilt ladders 15.6–16.3°, pushes 28.6–29.5°, homing 8–17°, handover 2–4° (a 45.8° line trips nothing; the
operator could put it at 35° and still trip nothing on these three); arm self-contact `torso_link vs *_shoulder_roll_link` 1.6 s
(stopramp) / 2.2 s (clear) / 6.6 s (roll), peaks 324–505 N, at the 0.15–0.30 rungs, the stops and the pushes — the upper-arm
capsule meets the torso box when the arm swings; never in a stand; arm stance departure mean 6.8–8.2°, peak 22.6–26.6° (pushes);
joint-limit dwell < 0.5 % on every joint (hip yaw / ankle roll the only non-zero ones). wz: 0.30 / 0.50 rungs still ~0 for all three,
0.80 at 0.37–0.68 rad/s.

### 10.7 A second stand, AFTER walking (2026-11-03 evening)
The v3 records showed two different standing behaviours and only one was scored: the clean-engagement stand (section 10.1, after the
balance → walk switch) draws stopramp's feet IN 5 cm over 60 s, while in the 4 s ladder stops the feet slide OUT fast — stopramp sep
0.275 → 0.345 m within the stop after vx+0.30 (sep_dev_max 0.13), combo 0.275 → 0.38, combo's stop after wz+0.80 ends with a stagger
of −0.45 m. The operator's own sim2sim stand comes AFTER walking, so the ladders now end with `walk_pre` (vx `HOMING_WALK1_VX` 0.30 for
`HOMING_WALK1_S` 10 s) and **`walk_stand`** (cmd 0 for `HOMING_STAND1_S` = 120 s), measured exactly like `stand` (base drift, heading
change, lifts, rate, sep₀ / sep_end / sep_dev_max / sep_dev_end, stag₀ / stag_dev_max / stag_dev_end, z, the Watch). Score rows
`wstand_*` mirror the `stand_*` rows with the same g / b (the twins share the stand's calibration keys); each stand row carries HALF
its former weight and the other half goes to its twin, so stability stays 30 (stand 4.125 + stand-after-walking 4.125). Labels: the
base rows read "BASE drift", the feet rows "feet separation deviation" / "feet stagger deviation". Both stands record `sep_dir`
(out / in / flat = sign of sep_end − sep₀ beyond 0.5 cm; no score) and `sep_end` / `stag_end`. Hard fails: `walk_pre` and `walk_stand`
get the walking height rule, tilt, contact and arm checks like a rung. Sim time per ladders test: 386 + 10 + 120 = **516 s**
(~1.7 min wall at rtf 5, ~2.7 min at the 8-rig rtf 3.2). Validation (examdev2, stopramp + combo, ff on): see the table below.

| policy (examdev2) | stand | yaw° | base drift cm | lifts | rate | sep₀ → end cm (dir), dev max·end | stagger₀ → end cm, dev max·end | z |
|---|---|---|---|---|---|---|---|---|
| stopramp | 60 s clean engagement | +4.0 | 1.9 | 0+2 | 0.019 | 33.8 → 29.3 (in), 5.4·4.5 | 3.3 → 5.2, 4.2·1.9 | 0.994 |
| stopramp | 120 s after walking 0.30 | +19.4 | 19.7 | 3+2 | 0.056 | 23.4 → 31.7 (out), 13.4·8.4 | −8.8 → +19.2, 28.4·27.9 | 0.991 |
| combo | 60 s clean engagement | +1.6 | 1.8 | 0+0 | 0.015 | 34.2 → 46.5 (out), 12.4·12.3 | 2.7 → 6.9, 4.3·4.3 | 0.993 |
| combo | 120 s after walking 0.30 | +15.6 | 31.2 | 0+2 | 0.063 | 26.3 → 40.2 (out), 14.1·13.9 | −9.2 → −5.6, 15.4·3.6 | 0.990 |

The two stands differ as the finding said: stopramp draws IN after the switch and slides OUT (+ a 28 cm stagger, 19° turn, 20 cm base
drift) after walking; combo slides out in both. Wall 3 min 53 s for 2 rigs (ladders 516 s sim); no hard fail tripped.

## 11. Exam #2 v4 (2026-11-09): CURRENT_BALANCE partner, diagonal pushes, small-command precision, the constraints block
Operator-approved scope 2026-11-09 (the Optuna study's objective + constraints, memory `optuna-study-design`); frozen for the whole study.
**Weights UNCHANGED** (survival 10, stability 30, tracking 20 = vx 8 + vy 4 + wz 8, gait 10, energy 10, pushes 10, homing 10; a tracking-25
reweight was discussed and reverted by the operator). Score version 4 = `tools/exam_score.py score_v4` (= `score_v3` with the changes
below); `score_v2` / `score_v3` and the v2 calibration stay as they were, so every v2 / v3 record keeps rendering and re-scoring exactly as
before (checked: `score_v2` / `score_v3` output byte-identical on all 150 stored Exam #2 records; fleetdeck's sections identical on 38 v2 / v3
records). `exam_record.py` / `exam_report.py` write and print v4; `exam_calibrate.py --apply` now re-scores each record with its own version.

### 11.1 Balance partner = CURRENT_BALANCE, read at launch
`tools/exam.sh` without `--balance` reads `$ASPIRED_W/MovementModule/policy/CURRENT_BALANCE` (the rig worktree, `Aspired_Robot_Project-homing`
on MM_rig; **p16c_sw_enc01** on 2026-11-09, was the fixed `p14h_stancewide`), once per run; `--balance` (or `HOMING_BALANCE`) overrides. The
slug goes into every record's `meta.balance` (it always did) and each rig checks that MovementModule loaded it (the stack log's `SHADOW
POLICY` line; a mismatch -> `failed.txt BALANCE_MISMATCH`). `run_mujoco_homing.sh` defaults to the same file. Consequence: the handover's
`to_balance` switch, the homing settle, the ladders' clean engagement and `walk_sag`'s reference stand all run p16c_sw_enc01 now -- a v4
record is not like-for-like with a v3 record (p14h_stancewide) on those columns.

### 11.2 Pushes from 8 directions (48)
`homing_commander.PUSH_DIRS` = the 4 sides + **front_left +45°, front_right −45°, back_left +135°, back_right −135°** in the same body-frame
convention (angle a from the heading, + = to the robot's left; the kick $\Delta v = L (\cos(\psi + a), \sin(\psi + a))$). Levels unchanged
(0.5 / 1.0 / 1.5 m/s), standing and walking at vx 0.3 → 3 × 8 × 2 = **48 pushes** (v3: 24), per level the 4 sides first (the v3 order), then
the diagonals; 2 × (12 × 7 s + 12 × 8 s) + 3 s = **363 s** of sim (v3: 183 s; `exam.sh` push timeout 600 → 900 s wall). The push score is
the mean over `EXPECTED_PUSHES_V4` = 48 (a fall = 0). The lean along the push already took any angle,
$L = (\theta - \theta_0)\cos a - (\phi - \phi_0)\sin a$; at a = +45° a 2° pitch and −2° roll excursion read L = 0.707 · 2 + 0.707 · 2 = 2.83°.
fleetdeck's push grid draws 8 columns per mode (front, front-L, left, back-L, back, back-R, right, front-R) for a v4 record, 4 for an old one.

### 11.3 Small-command precision: the rung error floor 0.05
$$e = \frac{\lVert v - c \rVert}{\max(\lVert c \rVert, c_0)}, \qquad c_0 = 0.05 \text{ m/s (v4; 0.1 in v2 / v3)}$$
on the same vector (vx, vy, 0.5 m × wz) — the LEVER stays, so the wz floor is 0.1 rad/s (was 0.2). Worked: at c = 0.05 an achieved 0.03 gives
e = 0.02 / 0.1 = **0.2** (v3, rung score (0.5 − 0.2) / 0.4 = 0.75) vs e = 0.02 / 0.05 = **0.4** (v4, score 0.25). Rungs from |c| ≥ 0.1 (wz ≥ 0.2)
are unchanged; the rungs below it — vx / vy ±0.05, wz ±0.05..±0.15 — now count their error against the command itself. `rung_e` stays a
definition (g 0.1 → b 0.5).

### 11.4 The constraints block (feasibility, separate from the points)
`meta.score.constraints` = a list of `{name, value, threshold, sense, c, pass, provisional, detail}` and `meta.score.feasible` = all pass; c ≤ 0
= feasible (Optuna's `constraints_func`): c = threshold − value for a "≥" constraint, value − threshold for "≤"; a value that could not be
measured (Test 1 did not run) = c 1. The constraints read the hard-fail-marked records (a hard-failed rung counts as not walked).
- **no_falls** (definition): the survival row's count = falls in Tests 0 / 1 / 3 + the v3 hard-fail groups outside the pushes ≤ 0. A push that
  knocks the robot over belongs to the push score (section 2) and is only shown in the detail.
- **walks_vx / walks_vy / walks_wz** (definition, the operator's hard rule "a locomotion policy must not decide that standing beats walking"):
  per axis the share of the ladder rungs with |c| ≥ 0.10 m/s (vx, vy) or ≥ 0.2 rad/s (wz) whose gain $g = v_{axis} / c$ ≥ 0.5 (sim truth along
  the command; the wrong sign is negative; a fell or missing rung fails) must be ≥ 0.90. Rung counts: vx 28 (0.10..1.00, −0.10..−0.50), vy 10,
  wz 26 → at most 2, 1 and 2 short rungs. Example: 16 of 26 wz rungs at gain ≥ 0.5 → 0.615, c = 0.90 − 0.615 = +0.285, infeasible.
- **track_floor_vx / _vy** (provisional, the 11-03 design's floors vx 0.9, vy 0.8): the mean capped gain $\min(\max(g, 0), 1)$ over the same
  rungs (an overshoot does not buy back an undershoot); **track_floor_wz08** (provisional, 0.5): the same over the two ±0.8 rad/s rungs.
- **tilt_floor** (provisional): `walk_tilt` (the stability row's value: the mean torso tilt over the walking rungs |v| ≤ 0.5 m/s) ≤ **8.0°**.
  Set above the six anchors with margin (their v3 records 2.1–6.3°, the highest lm7l_enc; 59 lm7j..lm7l records 1.6–9.6°, median 3.6°;
  two runs of one policy differ by ~0.05°). Torso tilt is not lean alone: it includes the gait's pitch.
The provisional thresholds live in `exam_score.TRACK_FLOOR` / `TILT_FLOOR_DEG` (env `EXAM_TRACK_FLOOR="vx=0.9,vy=0.8,wz08=0.5"`,
`EXAM_TILT_FLOOR_DEG`; `score_v4(track_floor=..., tilt_floor_deg=...)`); their final values come from the pilot's noise measurement
($\theta = \mu^* - z\sigma$). fleetdeck's score section shows one pass / fail badge per constraint (hover = value, threshold, c, detail).

### 11.5 Calibration v4-2026-11-09 (`results/exam2_calibration_v4.json`; the v2 file stays and keeps scoring the v2 / v3 records)
Method = section 6 (a provisional key → the population's p10 → p90, never narrower than half the median; a definition is kept and only
flagged DEAD at > 80 % at one end), applied to **the keys whose distribution v4 changes**, every other key inherited from v2-2026-11-01
(`tools/exam_calibrate.py --v4 --keys ... --write`; the file records `base`, `e_floor`, `recalibrated`, `changed`). Which keys: (a) by
construction — the push keys (8 directions), `push_energy`, the all-phase energy (the pushes are now 363 of ~960 s of Tests 0–2, 38 %, instead of 183 of
~780 s, 23 %), the arms' stance departure and the self-contact seconds (their windows include the pushes); (b) through the balance partner —
measured on the six anchors, same policies, v3 record → v4 record: a provisional key is affected when its median moved ≥ 10 % (the keys v4
cannot touch moved −3..+4 %: walk_tilt −1 %, smooth +1 %, slip 0 %, cot −1 %, stand_tau2 −3 %, arm_sway −2 %): `ho_jolt` +10 %,
`stand_rate` −14 %, the stand separation / stagger deviations +17..+43 %. `ho_jump` (+3 %) stays. The two stands (60 s + after walking) share
their keys, so the tool now pools both into the `stand_*` samples. Population: the six v4 anchor records (`v4_anchor_2026-11-09`, below; 283
held pushes, 36 switches, 12 stands, 6 per-policy values) — no older record carries a diagonal push or the p16c partner. **Thin for the
per-policy keys (n 6): re-run the same command over all v4 records once the next wave lands.**

| key | v2 / default g → b | v4 g → b | note |
|---|---|---|---|
| push_dist | 0.211 → 1.111 m | 0.211 → 1.035 m | 283 held pushes |
| push_tilt | 2.78 → 23.7° | 2.93 → 17.6° | |
| push_rec | 0.98 → 3.10 s | 0.96 → 3.15 s | |
| push_foot | 2.23 → 4.60 BW | 1.97 → 4.35 BW | |
| push_counter | 0.32 → 5.78° | 0.50 → 4.47° | |
| push_sway | 0.235 → 0.629 rad/s | 0.198 → 0.513 rad/s | |
| push_sep | 11.8 → 55.3 cm | 9.2 → 52.0 cm | |
| push_arm | 12.7 → 26.0° | 6.7 → 15.9° | was 70 % at full marks |
| push_energy | 577 → 1846 J | 521 → 1638 J | |
| push_steps | 2 → 6 | 2 → 6 | definition kept (37 % full / 16 % zero) |
| en_all_tau2 | 7247 → 12186 | 7024 → 11912 | n 6, widened to 0.5 × median |
| en_all_power | 95.5 → 165 W | 99.7 → 167 W | n 6, widened |
| arm_dev_mean | 2 → 15° (default) | 2.55 → 4.27° | n 6 (3.2–3.6°), widened; first calibration |
| arm_dev_max | 5 → 30° (default) | 18.4 → 30.0° | n 6, widened; first calibration |
| sc_dur | 0 → 2 s (default) | 1.28 → 5.90 s | n 6; was 67 % at zero; first calibration |
| ho_jolt | 0.135 → 0.478 rad/s | 0.118 → 0.301 rad/s | 36 switches (balance partner) |
| stand_rate | 0.041 → 0.102 rad/s | 0.047 → 0.079 rad/s | 12 stands; v2 came from the poisoned 20 s stand (10.1) |
| stand_sep_dev_max | 0.5 → 5 cm (default) | 5.2 → 20.0 cm | was DEAD (92 % at zero); first calibration |
| stand_sep_dev_end | 0.5 → 4 cm (default) | 1.3 → 19.7 cm | was 75 % at zero |
| stand_stag_dev_max | 0.5 → 6 cm (default) | 4.7 → 16.4 cm | was DEAD (83 % at zero) |
| stand_stag_dev_end | 0.5 → 5 cm (default) | 0.5 → 8.3 cm | |
| rung_e | 0.1 → 0.5 | 0.1 → 0.5 | definition kept: anchors 15 % full / 36 % zero; 67 lm7j..lm7l records re-read at the 0.05 floor 15 % / 50 % (at 0.1: 15 % / 45 %) |

Effect on the six v4 records (re-scored with `--apply`): −0.7..+0.5 points (legs 62.6 → 62.4, clear10 61.2 → 60.8, squeeze 60.5 → 59.8, arch
54.4 → 54.6, armsway 56.5 → 57.0, enc 47.1 → 47.1). The stand deviation ramps now end at 16–20 cm because the population drifts that far
(the walk policies' feet slide out after walking and after the p16c engagement); if the operator wants "how they start is how they finish"
as a tolerance rather than a population rank, these four keys should become definitions (operator's call).

### 11.6 Re-examination of the v4 anchors (`v4_anchor_2026-11-09_15-10-57`, 6 rigs unpinned, seed 1, n 10, Test 5 on, ff on)
`bash tools/exam.sh --parallel 6 --no-pin --seed 1 --n 10 --stumble --tag v4_anchor <6 slugs>`: **9 min 8 s wall** (per policy 365–542 s;
the two arch rows' homing timeouts set the end), rtf 4.0–5.7, 0 pose gaps, 0 ff reverts, every rig confirmed p16c_sw_enc01; ladders 516.9 s,
pushes 362–363 s of sim; the ladder engagement's balance → walk switch wait is **1.86 s** under p16c (p14h: 0.34–1.02 s).

| policy | v3 (p14h, land) | v4 | v4 parts surv/stab/trk/gait/energy/push/homing | feasible: failed constraints |
|---|---|---|---|---|
| lm7k_legs | 65.8 | **62.4** | 10.0 / 18.8 / 13.5 / 2.5 / 6.2 / 5.8 / 5.6 | walks_wz 0.58 |
| lm7j_stack_clear10 | 64.7 | **60.8** | 10.0 / 20.0 / 11.0 / 4.1 / 5.9 / 5.3 / 4.5 | walks_wz 0.62, track_floor_vy 0.78 *p* (walks_vy exactly 0.90 = c 0, passes) |
| lm7k_micro_legs_squeeze | 63.1 | **59.8** | 10.0 / 17.3 / 10.6 / 3.3 / 6.7 / 6.0 / 5.9 | walks_wz 0.69, track_floor_vx 0.83 *p* |
| lm7l_arch_armsway | 61.3 | **57.0** | 10.0 / 21.6 / 7.5 / 3.6 / 8.2 / 6.0 / 0.1 | walks_vy 0.40, walks_wz 0.50, track_floor_vx 0.89 *p*, track_floor_vy 0.44 *p* |
| lm7l_arch | 56.9 | **54.6** | 10.0 / 19.5 / 7.4 / 4.1 / 7.6 / 6.0 / 0.0 | walks_vy 0.40, walks_wz 0.42, track_floor_vx 0.87 *p*, track_floor_vy 0.40 *p* |
| lm7l_enc | 49.2 | **47.1** | 10.0 / 16.8 / 4.5 / 3.2 / 3.2 / 5.5 / 3.9 | walks_vx 0.29, walks_wz 0.69, track_floor_vx 0.42 *p* |

Same order as v3. **No anchor is feasible, and every anchor fails `walks_wz`**: no policy turns in place at 0.20–0.40 rad/s (gains −0.04..0.13 on
those rungs; ±0.8 is tracked at 0.81–0.99) — the "standing beats walking" pattern the rule exists for, on the turn axis. The arch rows also
stand under vy 0.10–0.20, enc under vx 0.10–0.45. no_falls passes everywhere (push falls: squeeze 1, clear10 2, enc 2 — scored in pushes).
tilt_floor passes everywhere (2.1–6.3°, margin ≥ 1.7° at enc). Where the 2–4 points between v3 and v4 come from, on the same v4 records:
the v3 rules give 63.0 / 61.5 / 60.8 / 56.8 / 54.7 / 47.7, so the floor 0.05 costs 0.3–0.6 points (tracking), the v4 table −0.7..+0.5, and the
rest is the new exam content (diagonal pushes: push points 5.3–6.0 vs v3's 6.2–7.0; the p16c partner: homing points fell on all six, legs 7.0 → 5.6 — over
the anchors' arrived trials the settled-error median +11 %, heading +19 % — and the handover switch-wait median 0.36 → 1.13 s) plus the run-to-run spread (sd ≈ 2.3 points per run, section 6).

## 12. Test 6 — encoder offsets (2026-11-13): the injection, the patterns, the test, the first validation (the bias numbers)
Part of **Exam #3** (section 13: how it is scored, the offset pushes, the constraints). This section = the instrument and what its bias
numbers showed. (First built the same morning as an opt-in "v5" with two bias constraints; the owner replaced that scoring -- 12.4.)
Why: the robot's joint encoders have zero errors (stop tests 10-07 / 11-06: residuals up to 1.75° at the ankle pitch) and the walk rows now
train with an encoder-offset band (13 joints = legs + torso, uniform ±0.010 rad per episode: the policy observes $q + o$, its target lands
at $q_{cmd} - o$). The rig's encoders were perfect, so Tests 0–5 could not tell whether a policy tolerates offsets.

### 12.1 Injection: the SIM's robot is miscalibrated, the deployed stack is untouched
`simulate/src/unitree_sdk2_bridge.h` (`set_encoder_offsets`): reported $q = q_{true} + o$ in `rt/lowstate` and the compact `rt/lockstep/state`,
and the motor PD closes on the reading,
$$\tau = \tau_{ff} + k_p (q_{cmd} - (q_{true} + o)) + k_d (\dot q_{cmd} - \dot q)$$
i.e. the target is applied at $q_{cmd} - o$. Worked: $k_p$ 200, $o$ = +0.010 rad → the first frame's torque moves by −2 N m and the joint
settles 0.57° below where the policy believes it is. Velocity and torque are unchanged; the truth channel `rt/sim_base_pose` ("qj") stays true
and gains "eo" (the table in force) only while an offset is non-zero. Set at run time on a planned frame: `enc_at <sim_t> zero | <27 rad>`
(`enc ...` = the next frame; stdin or `rt/sim_cmd`, a FIFO, applied before that frame's PD; |value| > 0.2 refused). Never sent = the old sim:
the zero path keeps the original expressions, and `tools/wire_check.sh` traces (per-frame hashes of ctrl / qpos / qvel) of the rebuilt binary
are **identical to the pre-feature binary's over 48 153 frames (96.3 s)**.
The rig's Bridge corrects nothing in sim: `BridgeModule/main/config.py` returns zero offsets under `BRIDGE_SIM=1` (the container's launch),
`ENCODER_OFFSET_ON` is False and the "ENCODER OFFSET CORRECT" line never appears in a rig log — the injected table is the whole error.

### 12.2 The patterns (`tools/enc_patterns.py`, fixed: the same numbers for every policy and run; `--check` re-draws P3 / P4)
mrad, reported − true; RESID = the measured residual per joint group (`study_env_cfg.RESID`), band = 10 mrad:

| joint (L / R) | P0 zero | P1 resid_band | P2 resid_mirror | P3 rand_a | P4 rand_b | P5 resid_full (stress) |
|---|---|---|---|---|---|---|
| hip yaw | 0 | 5 / 5 | −5 / −5 | −6.3 / −0.1 | 5.8 / 8.2 | 5 / 5 |
| hip pitch | 0 | 4 / 4 | 4 / 4 | 6.8 / −9.0 | 5.2 / −6.8 | 4 / 4 |
| hip roll | 0 | 10 / 10 | −10 / −10 | −5.8 / −4.2 | −0.7 / −3.9 | 20 / 20 |
| knee | 0 | 4 / 4 | 4 / 4 | −6.5 / −6.6 | 5.6 / 8.4 | 4 / 4 |
| ankle pitch | 0 | 10 / 10 | 10 / 10 | 0.9 / 6.1 | 9.8 / −6.4 | 30.5 / 30.5 |
| ankle roll | 0 | 10 / 10 | −10 / −10 | 4.2 / −4.0 | 1.8 / −6.3 | 13 / 13 |
| torso | 0 | 0 | 0 | −9.7 | 0.8 | 0 |

P1 = +RESID **clipped** to the band (the three large groups sit at the edge, the small ones keep their measured size; a scaled copy would
have shrunk them to 1.3–1.6 mrad). The stop tests give pair sums, not a sign per joint, so "+" = each joint's own positive direction. P2 =
the same error on the other side (sides swapped, roll / yaw signs flipped). P3 / P4 = numpy 2.4.5 `default_rng(20261113).uniform(-0.010,
0.010, 13)` twice, rounded to 0.1 mrad, hard-coded. P5 = the unscaled residuals (3 × the band at the ankle pitch): recorded, never counted.

### 12.3 The test (`homing_commander.py` kind "enc"; `tools/exam2_test.sh enc`; `exam.sh --exam 3` runs it LAST, after Tests 0–5)
Teleport to (−3, 0, 180°) under the balance policy, 5 s settle, switch to the walk (the ladders' clean engagement), 2 s stand + 8 s walk on
perfect encoders (so every stand, P0's too, follows a walk and its stop), then per pattern P0..P5: **settle 4 s** (the pattern lands on its
first frame) → **stand 15 s**, all measured → **walk 19 s at vx 0.30**, the last 15 s measured; then the 8 offset pushes under P5 (13.2);
then offsets back to zero. 311 s of sim + the switch wait (312.9 s measured), **59–70 s of wall** per policy at rtf ~5 (1 to 4 rigs; 247 s /
46–51 s before the offset pushes were added). A fall under a pattern: offsets to zero, reset on the spot, the next pattern. Every stand also runs the **injection check**: the mean of (joint angle the stack observes on `/BridgeModule/joints_imu_b` −
the sim's true angle) per joint must equal the pattern within 1.5 mrad, and the sim's "eo" echo must match; if not, the enc constraints are
"not measured" (c = 1) — a policy that never saw the offsets must not pass.

### 12.4 The bias numbers (`tools/exam_score.py` `enc_summary`; recorded, not scored)
The record's `enc.segments[*].enc` keeps the raw time-means per pattern (signed pitch / roll, base displacement in the heading frame, yaw
rate, walking body-frame vx / vy, tilt); the score block `meta.score.enc` holds the **paired** differences against the same run's P0,
$d = x(P_k) - x(P_0)$: `lean` = standing $\lVert(\Delta pitch, \Delta roll)\rVert$ (deg), `stand_drift` (mm/s), `walk_vy` = |Δ lateral
velocity| (mm/s), `walk_yaw` (deg/s), `walk_vx` (m/s), `walk_tilt` (deg); `worst` / `mean` over P1..P4. Worked: P0 pitch −0.28°, roll −0.10°;
P1 +1.40°, +0.20° → Δ = (1.68, 0.30), lean 1.70°.
These bias numbers are **recorded, not scored** (owner 2026-11-13, after 12.5: they do not separate a band-trained policy from an untrained
one). The morning's first scoring -- `score_v4` + the constraints `enc_lean` ≤ 2.9° and `enc_drift` ≤ 75 mm/s -- was replaced by Exam #3
(section 13); its 13 records were moved out of the milestones to `logs/exam/superseded_examdev_enc_2026-11-13/`.

### 12.5 First validation: the bias numbers (`examdev_enc`, 13 full exams of 5 checkpoints without the offset pushes, seed 1, n 10, ff on)
v4 unchanged: the exam without the flag on lm7k_micro_legs_squeeze scored **61.6** (stored v4 records 59.8 / 59.9, one run's sd ~2.3), same
record structure, no `enc_at` sent. Injection: max |observed − true − offset| 0.08–0.27 mrad over all 13 records. No fall under any pattern,
P5 included, for any policy. Checkpoint means (runs), worst over P1..P4 unless marked; noise = one run's sd pooled within checkpoints (8 dof):

| policy | enc training | runs | lean ° | stand drift mm/s | walk vy mm/s | walk yaw °/s | walk Δvx m/s | P5 lean ° | P5 Δvx m/s |
|---|---|---|---|---|---|---|---|---|---|
| lm7k_micro_legs_squeeze | none | 3 | 1.75 | 4.3 | 19 | 0.28 | 0.107 | 3.63 | 0.213 |
| lm7l_arch_armsway | none | 2 | 2.36 | 1.7 | 64 | 0.20 | 0.037 | 4.04 | 0.119 |
| s1_pilot_t37 | none | 2 | 2.12 | 5.6 | 42 | 0.08 | 0.043 | 2.97 | 0.135 |
| lm7m_enc_ladder | ±0.003 | 2 | **3.47** | 2.3 | 65 | 0.29 | 0.071 | 3.41 | 0.130 |
| lm7n_enc_survive (+ nodecay, the same checkpoint) | ±0.010 | 4 | 1.74 | 1.5 | 53 | 0.11 | 0.037 | 4.24 | 0.090 |
| noise (one run's sd) | | | 0.29 | 1.1 | 3.5 | 0.07 | 0.006 | 0.33 | 0.002 |

**The test does not separate the ±0.010-trained policy from the untrained ones on the bias metrics.** Its lean (1.74°) equals the untrained
squeeze's (1.75°, 0.0 sd apart) and its lateral drift (53 mm/s) lies inside the untrained range (19–64). The worst in-band lean is always
P1 or P2, 1.3–3.5° per run, mostly more than the 1.03° the pitch chain alone gives on rigid flat feet (0.004 + 0.004 + 0.010 rad): no policy cancels the offsets, the
band training taught surviving them. Where the ±0.010 policy is best: standing creep under P1..P4 (raw 1.5 vs ≥ 2.7 mm/s, 1.8 sd to the best
untrained), mean walking-tilt change (0.50 vs ≥ 0.64°, 2.8 sd) and the stress pattern's forward speed-up (+0.090 vs ≥ +0.119 m/s) and
stillness (0 steps in the P5 stand in 4 of 4 runs; the others 1.5–6.5). What the test does show clearly: lm7m_enc_ladder is the outlier on
lean (3.47°, 3.9 sd above the next), and P4 moves the lateral drift by 38–66 mm/s for four of the five checkpoints.
Consequence (owner, the same day): the bias limits were dropped, the P5 stand and pushes under P5 became the scored part -- section 13.

## 13. EXAM #3 (2026-11-13): Test 6 inside the qualities, the offset pushes, `walks_low`
Owner 2026-11-13: "No flags. This is our new test ... The previous runs are just flagged with what exam version they were done"; the pushes
under an offset go "under survivability, stability and gait and body care. Energy." Exam #3 = every test of Exam #2 v4, unchanged and in
the same order (so Tests 0–5 of an Exam #3 run are a v4 run: `meta.score.score_v4` is the same record under the v4 rules), then Test 6.
Records: `meta.exam_version` 3, `meta.score.version` 5, `meta.score.exam` 3, a top-level `enc` block. **Exam #3 scores do not compare
with Exam #2 scores** (the weights inside the qualities moved). `score_v2 / v3 / v4` are untouched (byte-identical on the 162 stored records).

### 13.1 The weights (`tools/exam_score.py score_e3`; the seven quality totals stay)
Inside a quality Test 6 takes a fixed share and every v4 row of that quality is scaled by $(W_q - share) / W_q$ (the v3 way: it halved
the stand rows to make room for the second stand). A row's ramp score s is the same number in both readings; only its weight moves.

| quality | total | v4 rows | Test 6 rows (weight; ramp) |
|---|---|---|---|
| survival | 10 | 10 × 0.5^falls | falls now also = a fall / hard fail in any pattern's stand or walk + an offset push not held |
| stability | 30 | × 25/30 = 25 | 5: P5 stand base drift 1.5 (`stand_drift` 1 → 10 cm), P5 stand foot lifts 1.0 (`stand_lifts` 0 → 3), P5 stand torso rate 0.5 (`stand_rate`), worst in-band stand base drift 1.0 (`stand_drift`), worst walking tilt change vs P0 over P1..P5 1.0 (`enc_walk_tilt` *p*) |
| tracking | 20 | 20 | — |
| gait & body care | 10 | × 8/10 = 8 | 2: touchdown impact p95 change vs P0, mean over the pattern walks 1.0 (`enc_impact` *p*), action roughness change vs P0 1.0 (`enc_smooth` *p*) |
| energy | 10 | × 8/10 = 8 | 2: recovery energy of the held offset pushes 1.0 (`enc_push_energy` *p*), walking mechanical power change vs P0 1.0 (`enc_power` *p*) |
| push robustness | 10 | × 48/56 = 8.57 | 1.43: the 8 offset pushes, the Test 2 per-push formula and ramps (one score over 56 pushes) |
| homing | 10 | 10 | — |

Test 6 carries 10.43 of the 100 points + its falls in survival. Worked: P5 stand drift 2.1 cm → s = (10 − 2.1) / 9 = 0.88 → 1.32 of 1.5;
37.6 cm → 0. The P5 stand uses the stand's own ramps on purpose (a stand is a stand; no new calibration). If Test 6 did not run or its
injection check failed, its rows score 0 and its constraints are "not measured".

### 13.2 The offset pushes
After the P5 walk, still under P5 ("enc:P5:hold" 4 s re-sends the pattern): Test 2's kick (`push_at`, the same "pre" / "push:mode:level:dir"
segments and metrics) at **1.0 m/s from front / back / left / right, standing (2 s before, 5 s after) and walking at 0.3 (3 s before)** =
8 pushes, 64 s. Test 2 holds the very same 8 pushes on perfect encoders; `meta.score.enc.pushes[*].t2` pairs them. A fall: reset on the spot
with the pattern still on, the next push. The v3 push hard fails (deep drop, tilt, floor contact, arm envelope) apply.

### 13.3 Constraints added (the v4 eight are unchanged; `no_falls` keeps its v4 definition = Tests 0, 1, 3)
- **walks_low** (provisional, ≥ 0.5): the share of the LOW ladder rungs — vx ±0.05 / ±0.10, vy ±0.05 / ±0.10, wz ±0.10 .. ±0.30 (18) — at
  gain ≥ 0.5; per axis in the detail and in `meta.score.e3.walks_low`. No new test: the ladder data. `walks_vx / vy / wz` start at 0.10
  (wz 0.20) and pass at a 0.9 share, so a policy can stand still at the smallest commands and pass; this one counts exactly those rungs.
  0.5 = a policy that walks at every rung where the `walks_*` rules start counting (|c| = 0.10, wz 0.20–0.30 → 10 of 18 = 0.56) passes,
  one that also stands at ±0.10 does not. Today: the 36 s1_pilot final records 0.00–0.28 (median 0.06), the 12 stored v4 records ≤ 0.33:
  nobody passes. Whether the study enforces it is the study's choice.
- **enc_no_falls** (definition, ≤ 0): falls + hard fails in the stand / walk of any pattern, P5 included. An offset push not held is shown,
  not counted (the v4 rule for Test 2: it costs survival and push points).
- **enc_stand_drift** (provisional, ≤ 5 cm) and **enc_stand_lifts** (provisional, ≤ 1): the 15 s stand under P5.
Env: `EXAM_ENC_STAND_DRIFT_CM`, `EXAM_ENC_STAND_LIFTS`, `EXAM_WALKS_LOW_SHARE`.

### 13.4 Validation (`exam3_val*`, 12 full Exam #3 runs of 5 checkpoints, seed 1, n 10, ff on, 4 rigs; 2026-11-13)
Calibration `e3-2026-11-13` (`exam_calibrate.py --e3 --write`, `results/exam3_calibration.json` = the v4 table + the five new keys, p10 → p90
of the 12 records): enc_walk_tilt 1.46 → 2.83°, enc_impact 0.141 → 0.353 BW, enc_smooth 0.0007 → 0.0076 rad, enc_push_energy 782 → 1277 J
(widened to half the median), enc_power −13.6 → −5.5 W (every policy uses LESS power walking under offsets: a rank, not a cost — weak row).
Checkpoint means (runs); noise = one run's sd pooled within checkpoints (7 dof):

| policy | enc training | runs | Exam #3 | v4, same runs | Test 6 points /10.43 | P5 stand drift cm · lifts (each run) | offset pushes held · score | Test 2, same 8 pushes: held · score | walks_low | enc constraints passed (runs) |
|---|---|---|---|---|---|---|---|---|---|---|
| lm7k_micro_legs_squeeze | none | 2 | 61.5 | 59.0 | 8.1 | 1.1 · 0, 0.5 · 0 | 8/8 · 0.56 | 8/8 · 0.55 | 0.19 | 2/2 |
| s1_pilot_t37 | none | 2 | 60.9 | 61.9 | 5.4 | 0.6 · 1, 54.6 · 14 | 8/8 · 0.55 | 8/8 · 0.56 | 0.08 | 1/2 |
| lm7n_enc_survive + nodecay (one checkpoint) | ±0.010 | 4 | 57.6 | 56.4 | 8.2 | 2.2 · 0, 1.8 · 0, 1.8 · 0, 1.8 · 0 | 8/8 · 0.61 | 8/8 · 0.60 | 0.01 | 4/4 |
| lm7l_arch_armsway | none | 2 | 55.3 | 59.6 | 2.5 | 37.6 · 8, 68.2 · 12 | 8/8 · 0.60 | 8/8 · 0.66 | 0.03 | 0/2 |
| lm7m_enc_ladder | ±0.003 | 2 | 48.8 | 53.3 | 4.2 | 39.5 · 11, 28.4 · 6 | 7.5/8 · 0.51 | 7/8 · 0.47 | 0.00 | 0/2 |
| noise (one run's sd) | | | 2.0 | 1.2 | 1.1 | | 0.04 | 0.03 | 0.05 | |

- **The P5 stand is the part that tells the band-trained policy apart, and it is bimodal for the others.** Over all 25 P5 stands on record
  (these 12 + the 13 of 12.5): lm7n ±0.010 passes both limits in **8 of 8** (drift 1.8–2.8 cm, 0 lifts); lm7m ±0.003 0 of 4 (12.8–39.5 cm,
  6–11 lifts); armsway 1 of 4; t37 1 of 4; squeeze 4 of 5 (0.2–1.1 cm, 0 lifts — and once 42.2 cm, 10 lifts). An untrained policy either
  stands as still as the trained one or creeps away stepping; one 15 s stand samples that coin once. So: the limits (5 cm in the gap
  2.8 .. 11.1 cm; 1 lift: the passing mode has 0–2, the failing one 3–14) are right, the band-trained policy passes with margin, but **a
  single run does not reject an untrained policy reliably** (squeeze 80 % pass). Fix if wanted: enter P5 several times (the mode seems set
  at the pattern switch), or require the pass in each of N runs.
- **The offset pushes do not separate anyone.** Every policy holds 8 of 8 (lm7m 7–8, as in Test 2), and the push score under P5 equals the
  same pushes' score on perfect encoders within the noise (0.51–0.61 vs 0.47–0.66, sd 0.04): a 1.0 m/s kick is not made harder by P5.
- **Exam #3 vs v4 on the same runs**: +2.5 (squeeze), +1.2 (lm7n), −1.0 (t37), −4.3 (armsway), −4.5 (lm7m: − the P5 stand and one offset
  push fall = survival halved in one run). The band-trained checkpoint moves from 4th to 3rd of the five. One run's sd rises from 1.2 to
  2.0 points: the P5 stand's coin (up to 3 points) and an offset-push fall (5 points of survival) are in the total now.
- `walks_low` fails everywhere (0.00–0.28), as in the study's records. Injection check: 0.08–0.24 mrad in all 12 runs. `--exam 2` on the
  final code: section 13.6.

### 13.5 fleetdeck
`fleetdeck/exam2.py section_enc`: "Test 6 — encoder offsets" (per-pattern table + the offset pushes above Test 2's same pushes), the new
constraints as badges with the others, "Exam #3" in the head, the score title and the constraints line (fleetdeck CONTEXT.md, round 2026-11-13b).

### 13.6 Versions and the default
`exam.sh --exam 3` = Exam #3; `--exam 2` = Exam #2 v4 exactly (no Test 6, `score_v4`, `exam_version` 2, no `enc_at` ever sent); `--exam 1` =
the 2026-10-29 exam. **The default is the one value `EXAM=` in `tools/exam.sh`: still 2 on 2026-11-13**, because the s1_pilot watcher calls
`exam.sh` without `--exam` and its 8 running trials must be scored with v4; the owner's decision is 3 — flip it once the study's stored
flags carry `--exam 2`.

