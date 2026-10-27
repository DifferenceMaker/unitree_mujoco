# HOMING sim2sim — results 2026-10-27

Chain under test: localiser pose → `homing_commander.py` → `/MovementModule/cmd_vel` → WALK policy → arrival at the table → balance
switch (`/MovementModule/policy`) → stand. Rig: `run_mujoco_homing.sh` (MuJoCo desk scene, table edge centre at (0.35, 0), approach
heading 0°; walk `lm7f_kit_footbonus6_still` @105397, balance `p14h_stancewide`; pose = MuJoCo ground truth + the LIO-SAM noise measured
on the robot the same morning, 1 cm / 0.1° jitter; no pushes). Trials: `tools/homing_trials.sh --n 10` (teleport to a random start
3–5 m from the table, ±60° around the approach line, random heading; goal = table edge; stop = torso front 0.35 m off the edge, i.e.
the base 0.50 m off; arrival tolerance 3 cm / 2°; hold; switch; 10 s settle). Raw records: `results/homing_trials_2026-10-27_*.jsonl`.

## Numbers

| batch | n | arrived | at the stop (planar / heading) | after 10 s (planar mean, max / heading mean, max) | time to switch |
|---|---|---|---|---|---|
| v3, balance switch (`_v3_switch`) | 10 | 10/10 | 1.7–6.3 cm, ≤ 3.0° | 7.5 cm, 12 cm / 3.7°, 6.2° | 18–60 s (mean 26) |
| v3.1, NO switch — the walk policy stands (`_v31_noswitch`) | 5 | 5/5 | 0.8–3.6 cm, ≤ 1.4° | 6.1 cm, 8.4 cm / **10.7°, 16.7°** | — |
| v3.1, balance switch (`_v31_switch`) | 10 | 10/10 | 0.7–4.2 cm, ≤ 3.8° | 8.4 cm, 16 cm / 3.9°, 11.5° | 16–30 s (mean 21) |
| v3.1, switch, hold 0.3 s (`_v31_switch_hold03`) | 10 | 10/10 | 0.6–4.1 cm, ≤ 2.4° | 7.6 cm, 17 cm / 4.7°, 11.3° | 15–25 s (mean 20) |

Walk from 3–5 m: FAR phase ≈ 0.44 m/s average (3 m in 7 s); whole approach 16–30 s; the two 40–60 s cases were starts facing away
(a full in-place turn at 0.36 rad/s left).

## What the numbers say

1. **The commander does its job**: at the stop it places the base within 0.7–6 cm and ≤ 4° in 25/25 trials (median ≈ 3 cm / 1°), on a
   pose with the localiser's noise. Arrival tolerance 3 cm / 2° is reachable with the kit's 0.05–0.15 m/s band and the pulse turns.
2. **The drift happens after the stop, and it is the WALK policy's zero-command stand**: with no switch at all the robot moves 4–8 cm,
   always to the RIGHT, and its heading turns 2…17° clockwise (right) within 10 s. The balance policy, once engaged, holds the heading
   far better (3.9° mean vs 10.7°) but keeps the lateral shift it inherited from the hold + gate (≈ 1.3 s under the walk policy):
   the robot stands **2…15 cm right of the approach line, mean 7 cm** in 20/20 switch trials. Nothing in the commander or the
   localiser produces this bias. Shortening the hold before the switch request from 1.0 s to 0.3 s changes nothing (7.6 cm mean,
   0.6…17 cm right, 10/10 right) — the shift is the balance policy's engagement from the walk stance plus the walk policy's own stand,
   not the waiting time. SIGN NOTE (corrected 2026-10-27 evening): the jsonl `lateral_m` / `heading_deg` are the commander's TO-GO
   errors (+ = the stop line is to the robot's left / the robot must turn left), so the "+7 cm" first read as "left" is the robot
   standing 7 cm RIGHT of the line and turned right — confirmed on the raw trajectory (fy = −0.067 m at approach heading 0). The
   clockwise bias matches the turn map (right turns 2x faster than left) and the drop reaction (the policy spins right after a
   teleport, −4…−13° in the 2 s before the goto).
3. **Turn asymmetry of the walk policy** (measured on this rig, in place): +0.5 → 0.02 rad/s (no left turn), −0.5 → −0.10, +0.8 → +0.36,
   −0.8 → −0.66; walking at 0.15 m/s both directions turn at ≈ 0.3 rad/s on a ±0.5 command. The commander turns in place only at the
   full ±0.8, arcs at 0.15 m/s + 0.5 for medium bearing errors, and trims heading only while translating. The training-side item is in
   `scripts/queue/lm7f/CONCLUSIONS.md` (the lm7d "+ side hard" seen again).
4. **Handover geometry**: the WORK/anchor handover should take the pose AFTER the balance policy settles (not at the switch), or the stop
   pose carries a lateral pre-compensation of the measured drift (knob `HOMING_LAT_BIAS` is not implemented — the number is a property
   of this walk policy and should be fixed on the policy side: the standing drift at zero command and the balance activation from the
   walk stance = the pose-bank mirror for the balance line).

## Instrumented batch (v3.2, 2026-10-27 14:19, seed 1, hold 1.0 s) — the ins and outs

The commander now logs a 10 Hz trajectory per trial (measured + filtered pose, commanded / measured / sim-true velocity, errors to
the stop pose, phase) and the record carries the waypoints (pose at every phase change), phase durations, path length / efficiency,
cruise speed, command-tracking gains (measured/commanded over steady-command ticks), pulses, re-alignments, the stand after the
switch and the drop→goto reaction. Files: `results/homing_trials_2026-10-27_v32_instrumented_seed1.jsonl`,
`results/homing_traj_2026-10-27_v32_instrumented_seed1/trial_*.csv`, the three plots
`results/homing_2026-10-27_footbonus6_still_seed1_instrumented_{map,velocity,errors}.png`, one benchmark row in `results/HOMING_BENCH.md`.
Offsets below are the ROBOT's offset from the stop pose (fwd + = towards the table, left +, heading + = turned left).

| trial | start (x, y, yaw°) | t_sw s | @switch planar / fwd / left / head | settled planar / fwd / left / head | stand after the switch fwd / left / turned | path m (eff) | cruise cmd/meas | g_vx | g_vy L/R | g_wz L/R | pulses / turn segments | drop→goto |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | −2.38, 3.69, 83 | 32.6 | 0.028 / +0.022 / −0.016 / −1.3° | 0.085 / +0.041 / −0.075 / −5.3° | +0.019 / −0.059 / −4.0° | 5.06 (0.87) | 0.49/0.44 | 0.87 | 0.28/0.72 | 0.31/0.70 | 5 / 9 | 4 cm, −11.0° |
| 2 | −3.64, 0.65, −101 | 24.6 | 0.019 / +0.019 / +0.002 / −2.0° | 0.085 / +0.036 / −0.076 / −2.4° | +0.017 / −0.078 / −0.4° | 4.11 (0.88) | 0.49/0.41 | 0.88 | 0.38/? | 0.26/? | 2 / 5 | 2 cm, −0.5° |
| 3 | −3.45, 1.18, 76 | 17.8 | 0.025 / +0.008 / −0.024 / −1.9° | 0.113 / −0.001 / −0.113 / −0.6° | −0.009 / −0.089 / +1.3° | 3.75 (0.95) | 0.49/0.42 | 0.86 | ?/? | 0.59/0.46 | 0 / 2 | 3 cm, −3.7° |
| 4 | −1.71, −3.20, 34 | 38.1 | 0.053 / +0.040 / −0.035 / −1.4° | 0.067 / +0.039 / −0.055 / +4.1° | −0.001 / −0.020 / +5.5° | 4.73 (0.75) | 0.49/0.43 | 0.84 | 0.41/? | ?/0.65 | 10 / 22 | 4 cm, −7.2° |
| 5 | −2.24, −2.97, 11 | 24.5 | 0.055 / +0.039 / +0.038 / −2.1° | 0.037 / +0.034 / −0.015 / −7.8° | −0.005 / −0.053 / −5.8° | 4.42 (0.82) | 0.49/0.42 | 0.87 | ?/? | 0.09/0.57 | 2 / 8 | 5 cm, −2.7° |
| 6 | −2.78, 1.98, −175 | 23.2 | 0.039 / +0.008 / −0.038 / −0.8° | 0.189 / +0.014 / −0.189 / −1.2° | +0.006 / −0.151 / −0.3° | 4.01 (0.85) | 0.49/0.42 | 0.88 | ?/0.74 | 0.31/? | 0 / 2 | 2 cm, +0.1° |
| 7 | −3.42, −2.76, 27 | 18.3 | 0.026 / −0.004 / +0.026 / +0.3° | 0.020 / +0.012 / −0.016 / −0.5° | +0.016 / −0.042 / −0.9° | 4.61 (0.93) | 0.49/0.43 | 0.87 | 0.61/? | ?/0.65 | 1 / 3 | 4 cm, −6.3° |
| 8 | −1.96, −3.32, −59 | 25.9 | 0.022 / +0.001 / −0.022 / +2.2° | 0.057 / −0.009 / −0.057 / +3.4° | −0.010 / −0.035 / +1.2° | 4.62 (0.81) | 0.49/0.44 | 0.89 | 0.22/? | 0.27/0.77 | 1 / 4 | 1 cm, −1.0° |
| 9 | −3.03, 0.08, −153 | 19.3 | 0.015 / +0.004 / −0.014 / +2.8° | 0.060 / +0.046 / −0.038 / +2.9° | +0.042 / −0.024 / +0.1° | 3.27 (0.90) | 0.48/0.43 | 0.91 | ?/? | 0.42/? | 1 / 2 | 2 cm, −0.4° |
| 10 | −2.81, −2.97, −92 | 47.7 | 0.024 / +0.024 / 0.000 / −2.3° | 0.112 / +0.043 / −0.104 / −4.0° | +0.019 / −0.104 / −1.7° | 5.28 (0.75) | 0.49/0.44 | 0.89 | 0.39/? | 0.30/0.76 | 6 / 48 | 3 cm, −0.6° |

Batch: 10/10 arrived; at the switch planar median 2.6 cm (max 5.5), heading median 2.0° (max 2.8); settled planar mean 8.3 cm
(max 18.9), heading mean 3.2° (max 7.8); time to the switch mean 27 s (18–48). `?` = too few steady ticks for a gain.

What the instrumentation adds:

5. **The stand after the switch moves the robot RIGHT in 10/10 trials** (2–15 cm, mean 6.6 cm) and is still creeping at +10 s
   (errors plot: the mean left offset goes −1 → −6 cm in the first 5 s and −7.5 cm at 10 s). The heading after the switch is mixed
   in sign (−5.8…+5.5°, mean −0.5°); the fwd offset is stable (+2.5 cm mean). So the one-sided bias is lateral, not angular.
6. **Cruise and tracking**: commanded 0.49 m/s, achieved 0.42–0.44 (gain 0.84–0.91, vx RMS 0.06–0.08 incl. the estimator lag);
   the measured velocity (pose differenced over 0.4 s) and the sim truth agree, so the localiser-only velocity estimate is usable
   on the robot. Path efficiency 0.85 mean; 0.75 for the two trials with pulse trains.
7. **Every left-side command is weaker**: wz in place L 0.32 / R 0.65 (measured/commanded), vy L 0.22–0.61 / R 0.72–0.74, and the
   drop reaction is clockwise in 9/10 (−0.4…−11° in the 2 s between the teleport and the goto). The policy has a clockwise/right
   bias in every channel.
8. **Where the time goes**: FAR 9–17 s, ALIGN 0.4–4.5 s, APPROACH 4–23 s. Trials 4 and 10 spent 16 / 23 s in APPROACH with 10 / 6
   pulses and 22 / 48 in-place turn segments — the residual heading error at the stop was to the LEFT, and a 0.25 s +0.8 pulse
   produces almost nothing (velocity plot, trial 10: a train of +0.8 commands with a flat measured wz). Commander item: for a left
   residual use a short walking arc (0.15 m/s + 0.5 turns 0.3 rad/s) or a longer left pulse; the right residuals clear in 1–2 pulses.

## Launch (what to run)

```bash
cd ~/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim
bash run_mujoco_homing.sh                     # sim window + stack; HOMING_WALK / HOMING_BALANCE / HOMING_VMAX / HOMING_HOLD_S / HOMING_BALANCE="" (no switch)
bash tools/homing_trials.sh --n 10 --seed 1 [--tag NAME] [--no-plot]   # the experiment: per-trial lines, two tables (placement; trajectory + tracking),
                                              # a row in results/HOMING_BENCH.md, plots in logs/homing_plots/ (map, velocity, errors)
echo "teleport -4 0.5 20" > logs/.sim_stdin; echo "goto 0.35 0 0" >> logs/.homing_cmd      # one trial by hand
bash run_mujoco_homing.sh stop
# re-plot any batch later: ~/miniconda3/envs/isaacsim/bin/python tools/homing_plot.py --jsonl logs/homing_trials.jsonl --n 10 --traj logs/homing_traj --out results/<name>
```
Per-trial trajectory CSVs: `logs/homing_traj/trial_<n>_<stamp>.csv` (t, phase, x y yaw, filtered fx fy fyaw, cmd_vx cmd_vy cmd_wz,
measured mvx mvy mwz, source ovx ovy owz = sim truth / the LIO twist, along lat head_deg = to-go errors, d_pre). As a benchmark: the
same `--seed` gives the same starts for every walk policy (`HOMING_WALK=<slug> bash run_mujoco_homing.sh`), one bench row per batch.
Code: `Aspired_Robot_Project-homing` (branch MM_loco_switch_homing) `MovementModule/Utils/homing_commander.py`; rig pieces in this repo
(`run_mujoco_homing.sh`, `tools/homing_trials.sh`, `tools/_nodes_in_container.sh` MODE h, sim `teleport` command in `simulate/src/main.cc`).
On the robot the same node runs with `--pose lio` (`/lio_sam_ros2/mapping/re_location_odometry`) and the goal from
`LocalizationModule/config/workstations.yaml` (T1: table (1.337, −0.099), heading −1.68°).
