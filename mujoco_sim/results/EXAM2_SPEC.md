# Exam #2 — the refined standardized walk exam (2026-10-30)

Status: **implemented 2026-10-30** (operator: "I like what you have proposed + my refinements. Start implementing."), first smoke
on lm7f_kit_alive5; g/b values marked *provisional* wait for the calibration run (section 6). `tools/exam.sh` runs Exam #2 by default
(`--exam 1` = the 2026-10-29 exam). Decisions of 2026-10-30 (section 9 answered): quality-based score, the weights below, 4 s rungs,
pushes 0.5 / 1.0 / 1.5 m/s standing and walking at vx 0.3, P-law homing turns with counted rescues, starts all around the desk with
around-desk planning, the stance drawing. Open: the training band (section 8), seeds per exam (section 9).
Exam #1 = `tools/exam.sh` as of 2026-10-29 (Test 0 handover, Test 1 staircase, Test 2 homing, score 40/20/20/10/10; catalogue in
`EXAM_METRICS.md`). Exam #2 keeps the rig (lock-step headless MuJoCo, the real Bridge + MovementModule stack, the comx06 armature
hand790 body, p14h_stancewide as the balance policy) and changes what is tested and how it is scored.

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

## 5. Stance drawing
Possible, two levels:
- **Top view (done):** the two soles at true size (26 × 8.6 cm) from the recorded separation, stagger and toe angles.
- **Front / side view (Exam #2):** record the full joint state through the 20 s stand, average it, run MuJoCo forward kinematics and
  draw the robot as a stick figure (or render the model in that mean pose), with the 20 s spread as a ghost and the support polygon +
  CoM projection in the top view.

## 6. Noise floor and calibration
The lock-step rig repeats a seed bit for bit, so re-running the same seed shows no noise; the noise is in **which starts the seed
draws**. One seed = one sample of 10 homing starts. Example: lm7f_kit_alive5's settled errors (4.3, 8.6, 7.2, 9.3, 8.6, 6.2, 1.5,
5.0, 12.9, 6.9 cm) have a mean of 7.05 cm and a std of 3.1 cm; the standard error of the mean is $\sigma/\sqrt{n}$ = 3.1/√10 = 1.0 cm.
Exam #1's placement = 20 (1 − ē / 10 cm), so 1 cm = 2 points: two policies 2 points apart on placement are a coin toss.

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
- **Wall time:** one policy, full Exam #2 with n 10 homing = 742–746 s on a pinned rig (2 rigs in parallel, 2026-10-31).

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
