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
   always to the LEFT, and its heading drifts +2…+17° (left) within 10 s. The balance policy, once engaged, holds the heading far
   better (3.9° mean vs 10.7°) but keeps the lateral shift it inherited from the hold + gate (≈ 1.3 s under the walk policy):
   lateral **+2…+15 cm left, mean +7 cm** in 20/20 switch trials. Nothing in the commander or the localiser produces this bias.
   Shortening the hold before the switch request from 1.0 s to 0.3 s changes nothing (7.6 cm mean, lateral +0.6…+17 cm, 10/10 left) —
   the shift is the balance policy's engagement from the walk stance plus the walk policy's own stand, not the waiting time.
3. **Turn asymmetry of the walk policy** (measured on this rig, in place): +0.5 → 0.02 rad/s (no left turn), −0.5 → −0.10, +0.8 → +0.36,
   −0.8 → −0.66; walking at 0.15 m/s both directions turn at ≈ 0.3 rad/s on a ±0.5 command. The commander turns in place only at the
   full ±0.8, arcs at 0.15 m/s + 0.5 for medium bearing errors, and trims heading only while translating. The training-side item is in
   `scripts/queue/lm7f/CONCLUSIONS.md` (the lm7d "+ side hard" seen again).
4. **Handover geometry**: the WORK/anchor handover should take the pose AFTER the balance policy settles (not at the switch), or the stop
   pose carries a lateral pre-compensation of the measured drift (knob `HOMING_LAT_BIAS` is not implemented — the number is a property
   of this walk policy and should be fixed on the policy side: the standing drift at zero command and the balance activation from the
   walk stance = the pose-bank mirror for the balance line).

## Launch (what to run)

```bash
cd ~/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim
bash run_mujoco_homing.sh                     # sim window + stack; HOMING_WALK / HOMING_BALANCE / HOMING_VMAX / HOMING_HOLD_S / HOMING_BALANCE="" (no switch)
bash tools/homing_trials.sh --n 10 --seed 1   # the experiment; summary table at the end; records -> logs/homing_trials.jsonl
echo "teleport -4 0.5 20" > logs/.sim_stdin; echo "goto 0.35 0 0" >> logs/.homing_cmd      # one trial by hand
bash run_mujoco_homing.sh stop
```
Code: `Aspired_Robot_Project-homing` (branch MM_loco_switch_homing) `MovementModule/Utils/homing_commander.py`; rig pieces in this repo
(`run_mujoco_homing.sh`, `tools/homing_trials.sh`, `tools/_nodes_in_container.sh` MODE h, sim `teleport` command in `simulate/src/main.cc`).
On the robot the same node runs with `--pose lio` (`/lio_sam_ros2/mapping/re_location_odometry`) and the goal from
`LocalizationModule/config/workstations.yaml` (T1: table (1.337, −0.099), heading −1.68°).
