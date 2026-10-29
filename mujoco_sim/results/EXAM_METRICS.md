# The standardized walk exam — metric catalogue

One list of every number the exam records, how it is computed, its units, what it flags and which eye verdict it corresponds to.
The exam = `tools/exam.sh` (N lock-step headless rigs): per policy **Test 1 staircase** (`tools/staircase.sh`, commander `stairs`) then
**Test 2 homing** (`tools/homing_trials.sh --n 10 --seed 1`). Same seed = same starts; on the lock-step rig a run is reproducible bit for
bit (det12/det13, seqA/seqB). Records: `homing_<policy>.jsonl` (one line per trial), `stairs_<policy>.jsonl` (one line per staircase),
the 10 Hz CSVs, and the per-policy `exam/<tag>_<date>.json` written into the milestone (fleetdeck draws the sheets from it).
Numbers are FLAGS and a ranking aid; the operator's eye on the rig is the verdict (RL Bible rule 34). Status column: live = recorded by the current code (the stance metrics since 2026-10-29 midday); planned = not yet.

## Test 0 — the HANDOVER (LIVE 2026-10-29 evening: tools/handover.sh, run first by exam.sh; `--no-handover` skips it; operator: "Maybe the test starts with balance and then goes into locomotion and then walks to the table and the rest of the exam")
Why: the 10-29 hardware test failed at the switch (gforce_airtime stepped backwards and drew the legs in right after p14h_stancewide -> walk;
footbonus6_still stumbled at the handover), and the exam never measured it: its stand starts after a teleport, with a walk policy that has
stood for 40 s. Test 0 runs FIRST, from the real entry state: the balance policy (CURRENT_BALANCE) stands 10 s holding an arm pose, the
commander switches to the walk at a zero command, and the next 8 s are recorded; then the walk -> balance leg (stop, switch back, 10 s).
Arm set (one leg per pose, the same seed): default arms; RIGHT ARM UP + LEFT ARM AT THE SIDE (ActionModule go_to_start: right hand
(0.25, 0.49, 0.35), left hand (0.05, 0.315, -0.056), torso frame, y outward); both arms forward at desk height; both arms up.

| metric | definition | units | flags | eye verdict it maps to | status |
|---|---|---|---|---|---|
Implementation: three queue items per policy (`handover X Y YAW POSE`), each = segments `bal_stand` (10 s, arms ramped to the pose over
2 s on /BridgeModule/joint_set_arms), `walk_after` (8 s from the frame the walk became active), `bal_after` (10 s from the switch back).
Every segment carries the staircase's per-segment metrics (lifts, drift_xy, dx_body, yaw_change_deg, z_min, sep_mean / sep_trend,
stagger, toe, jitter); the record adds `switch_wait_s` (request -> active, the gate's delay) and the 10 Hz rows. Arms: the right-up pose is
the joint-space median of p14h_stancewide's bank cluster held at the go_to_start IK target; both_forward is an approximate joint pose.

| metric (segment walk_after unless noted) | flags | eye verdict it maps to |
|---|---|---|
| lifts | any = "it steps at the handover" | "stumbles at handover" |
| sep_trend | < −0.01 m = the inward draw | "legs rotated inwards" |
| dx_body (forward, left) | forward < −0.01 m = the backward creep | "steps backwards and keeps moving backwards" |
| yaw_change_deg | \|x\| > 3° = the stand turns at the switch | "legs rotated inwards" (the feet yaw with the turn) |
| z_min vs bal_stand | a dip > 0.01 m | "sags at the switch" |
| bal_after: the same | the reverse leg (walk -> balance) | "shifts right when the balance takes over" |
| switch_wait_s | > 1 s = the gate held the switch (knee load / gyro) | — |

First run (t0dev, 2026-10-29, the two policies that failed on the robot that day): after the switch the separation fell 2-9 cm in every
pose (gforce_airtime both_forward 0.342 -> 0.249 m; footbonus6_still right_up -7.3 cm), the stand turned -4..-9 deg, gforce_airtime lifted
a foot 5-6 times with the arms at default / right-up; the balance's switch back widened the feet again (+6..+9 cm). The exam reproduces the
hardware failure (inward legs at the handover); torso-rate rms is not in the record yet.

## Test 1 — staircase (sim truth, free floor at (−3, 0, 0), 113 s of sim time)
Segments: stand 10 s; vx 0.05 / 0.10 / 0.15 (8 s each), −0.10 (6 s); vy +0.10 / −0.10 (6 s); wz +0.3 / −0.3 / +0.5 / −0.5 / +0.8 / −0.8
(6 s each); 2 s stands between, 3 s at the end. Steady window = the segment minus its first 1.5 s.

| metric | definition | units | flags | eye verdict it maps to | status |
|---|---|---|---|---|---|
| gain (per command segment) | $g = \bar v_{true} / v_{cmd}$ over the steady window, sim-truth body velocity (vx, vy or wz) | — | < 0.8 under-tracks, ≈ 0 stands under the command, > 1.2 overshoots | "tracks 0.05 / 0.10", "doesn't move at 0.05", "doesn't turn below 0.8" | live |
| v_true_std | std of the sim-truth velocity over the steady window | m/s, rad/s | large = surging / stop-and-go gait | "step, step, wait" | live |
| drift_xy (stand segments) | planar displacement start → end of a zero-command segment | m | > 0.05 m over the 10 s stand = the stand walks | "sways and needs to step", "legs draw inward until they step" | live |
| dx_body | the same displacement in the segment-start body frame (forward, left) | m | sign = direction of the creep | "forward creep", "walks right" | live |
| yaw_change_deg | unwrapped heading change over the segment | deg | stand: > 3° = the stand turns; vx segments: heading drift while walking | "turns clockwise while standing", "heading drift" | live |
| lifts | contact → air transitions per foot (floor force > 1 N) | count | stand: any lift = stepping while standing; turns: the stepping-turn count | "needs to step sometimes", "stampedes" | live |
| pivot_rad_s | mean \|foot yaw rate\| of a LOADED foot over the segment | rad/s | turns: ≥ 0.3 = the sliding turn; ≈ 0 = a stepping turn (the lm7g_pivot question) | "slides its feet when rotating" | live |
| loaded_frac | share of (foot, sample) pairs in contact | — | < 0.5 while standing = hopping | — | live |
| swing_height | max foot rise above its segment-start height while in the air | m | < 0.02 shuffles, > 0.10 stamps | "lift looks good", "shuffle" | live |
| z_min | lowest base height in the segment | m | < 0.55 = fell (the staircase aborts: `fell`) | "fell", "squats" | live |
| fell | first segment and time the base went below 0.55 m | — | any = the policy fell during the script | — | live |
| **feet separation** | lateral distance between the two ankle bodies in the base frame, mean and start→end trend per segment | m | trend < −0.03 m over the stand = the legs draw inward; mean < 0.18 narrow stance | "legs draw inward", "legs apart, good" | live |
| **stagger** | fore-aft offset between the feet in the base frame (left minus right), mean and \|mean\| | m | \|stagger\| > 0.08 while standing = one foot ahead of the other | "one leg in front of the other until it needs to step" | live |
| **toe angle L / R** | foot yaw relative to the base heading (+ = toe out), mean per segment | deg | \|toe\| > 10° = toe-out / toe-in stance; L ≠ R = asymmetric | "feet parallel", "toed out", "rotate inward" | live |
| **joint jitter** | std of the high-passed (sample-to-sample difference) knee and hip pitch angles over the segment, per side | deg | stand: > 0.5° = the "twerking" oscillation; walking: compare rows at equal speed | "wiggles up and down", "twerking" | live |
| footprint | the two feet's positions in the base frame over the stand (scatter) — drawn, not scored | m | — | the stance picture | planned (fleetdeck draws separation/stagger/toe over time instead) |

## Test 2 — homing (teleport 3–5 m from the table edge, goto, balance switch at the stop, 10 s settle; 10 trials, seed 1)

| metric | definition | units | flags | eye verdict it maps to | status |
|---|---|---|---|---|---|
| result | ARRIVED / FAIL_timeout (90 s) / FAIL_switch_timeout / STOPPED | — | arrivals / n | "walks off", "never arrives" | live |
| err_at_switch | to-go error (along, lateral, heading) when the switch is requested; planar = $\sqrt{a^2 + l^2}$ | m, deg | the WALK policy's placement | "precise stop" | live |
| final_err | the same after the balance policy's 10 s settle | m, deg | the stand's placement; pass = ≤ 0.05 m and ≤ 2° | "stands where it stopped" | live |
| settle_drift | final − switch, per component; printed as the robot's offset (sign flipped: fwd / left / turned-left +) | m, deg | what the hold + switch + settle ADDED; lateral mean = the engagement shift (p16d C7) | "shifts right when the balance policy takes over" | live |
| start_vs_teleport | the pose at goto minus the teleport pose | m, deg | the policy's reaction to the drop (2 s of standing) | "spins when dropped" | live |
| phase_s | seconds per phase (FAR, ALIGN, APPROACH, PULSE, HOLD, SWITCHING, SETTLE) | s | long ALIGN / many PULSE = the turn is the bottleneck | "takes forever to turn" | live |
| path_m / straight_m / path_eff | path length (0.5 s points), straight distance, their ratio | m, — | eff < 0.8 = detours (turn-in-place arcs, re-aligns) | — | live |
| v_far | commanded / measured / max speed over the FAR cruise (cmd ≥ 0.2 m/s) | m/s | measured/commanded ≈ the cruise gain | "walks slower than asked" | live |
| track (gains) | measured/commanded over steady-command ticks: vx (≥ 0.1), vy L / R (≥ 0.08), wz in place L / R (= 0.8), wz walking | — | L ≠ R = the clockwise bias (footbonus6_still: wz L 0.3 / R 0.65) | "turns right better than left" | live |
| vx_rms | rms of (cmd − measured) vx over steady ticks | m/s | — | — | live |
| n_turns / t_turn_s | in-place turn episodes and their total time | count, s | — | — | live |
| n_pulses / n_realign | full-command heading pulses at the stop; re-approaches after a hold drift | count | many pulses = the small-command turn is dead | "doesn't turn at small commands" | live |
| t_switch / t_total | sim seconds from goto to the switch / to the record | s | time score | — | live |
| trajectory (CSV, 10 Hz) | t, phase, raw + filtered pose, commanded / measured / truth velocity, errors, distance to the pre-pose | — | the map / velocity / error sheets | — | live (2 Hz copy in the milestone JSON: live) |

## Score (tools/exam_report.py, 0–100; a ranking aid, the weights are a choice)
tracking 40 = mean over the 12 staircase command segments of min(gain, 1) (a fall = 0) · placement 20 = 1 − min(settled planar mean / 0.10, 1)
· stillness 20 = ½(1 − min(stand-10 s drift / 0.10, 1)) + ½(1 − min(|stand after the switch, lateral| / 0.10, 1)) · turn 10 = (1 − min(pivot / 0.5, 1))
× min(|gain| at 0.8, 1) · survival 10 = arrivals / n (0 if fallen). Planned: a stance term from feet separation trend, |stagger| and joint jitter
once their ranges are known from the first run that records them.

## Reads so far (lm7f, 2026-10-28/29)
Agreement with the eye at the ends (gforce_airtime top; angmom rows + full bottom; footbonus6_still the stillest stand). The eye saw
three things the exam did not measure: vertical "twerking" (posture) — base-z std is normal (2.7 mm standing), the oscillation is in the
joints → joint jitter; the inward leg draw (clear9, footbonus6, drall) → feet separation; "one leg in front of the other" (airtime) → stagger.

## First stance numbers (lm7f_kit_gforce_resets_posture, one staircase, 2026-10-29)
stand 10 s: separation 0.296 m with a −2.9 cm trend (the legs draw in), stagger −7.3 cm (right foot ahead), toe L 0.4° / R 3.4°, knee jitter
0.52 deg/sample, hip 0.21 — the "twerking" row reads 0.52; walking 0.10 m/s: knee jitter 1.05 (the gait's own swing, compare rows at equal speed).
The ranges for a stance score come from the first full wave that records these.
