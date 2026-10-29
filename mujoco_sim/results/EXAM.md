# Standardized walk exam (tools/exam.sh) -- one block per run, newest last

## EXAM lm7f19 — 2026-10-28 19:45 — 17 policies, seed 1, homing n 10, balance p14h_stancewide, 7 instances, wall 37 min 14 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/lm7f19_2026-10-28_19-08-03

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_gforce_airtime | **55.8** | 23.8/7.2/14.6/0.2/10.0 | 10/10 | 0.064 | -0.034 / 0.2 | 0.88 | 0.50/0.69 | 0.91 | 0.74/1.06/1.07/0.90 | 0.61/0.93 | -0.01/0.04 | 0.16/0.34 | 0.68/0.74 | 0.020 / -5.0 / 0 | 0.49 | [21, 22] | 0.029 | - | 1119 |
| 2 | lm7f_kit_alive10 | **54.6** | 23.5/4.5/15.0/1.7/10.0 | 10/10 | 0.078 | -0.041 / -3.9 | 0.88 | 0.47/0.66 | 0.85 | 1.09/1.33/1.12/0.58 | 0.55/0.99 | 0.16/-0.02 | 0.07/0.47 | 0.63/0.59 | 0.009 / 1.0 / 4 | 0.36 | [22, 21] | 0.037 | - | 633 |
| 3 | lm7f_kit_gforce_resets_posture | **54.4** | 27.3/6.7/10.5/0.0/10.0 | 10/10 | 0.067 | -0.052 / 2.7 | 0.88 | 0.52/0.68 | 0.85 | 1.31/1.22/0.99/0.72 | 0.74/1.04 | -0.05/0.46 | 0.38/0.52 | 0.62/0.74 | 0.043 / -8.9 / 1 | 0.72 | [21, 14] | 0.024 | - | 678 |
| 4 | lm7f_kit_gforce_drall | **52.5** | 19.0/7.2/13.3/3.1/10.0 | 10/10 | 0.064 | -0.040 / -2.9 | 0.89 | 0.55/0.43 | 0.88 | 0.76/0.97/1.01/0.59 | 0.16/0.22 | -0.06/0.24 | 0.24/0.33 | 0.82/0.38 | 0.028 / -16.6 / 8 | 0.24 | [23, 6] | 0.030 | - | 659 |
| 5 | lm7f_kit_footbonus6_still | **52.0** | 17.5/5.8/15.8/3.0/10.0 | 10/10 | 0.071 | -0.040 / -0.6 | 0.88 | 0.31/0.67 | 0.86 | -0.02/1.04/1.01/0.34 | 0.74/0.95 | -0.03/0.18 | 0.01/0.01 | 0.33/0.66 | 0.002 / 2.6 / 0 | 0.19 | [7, 14] | 0.031 | - | 621 |
| 6 | lm7f_kit_gforce_resets | **51.9** | 24.3/3.2/14.3/0.0/10.0 | 10/10 | 0.084 | -0.039 / -2.2 | 0.88 | 0.55/0.66 | 0.85 | 0.89/1.05/1.09/0.74 | 0.34/0.89 | 0.04/0.05 | 0.43/0.39 | 0.78/0.76 | 0.018 / 0.9 / 0 | 0.61 | [18, 17] | 0.029 | - | 847 |
| 7 | lm7f_kit_alive5 | **51.2** | 23.0/4.7/11.5/2.0/10.0 | 10/10 | 0.076 | -0.051 / -1.6 | 0.86 | 0.42/0.64 | 0.87 | 1.38/1.24/1.08/0.66 | 0.76/0.74 | -0.10/0.24 | -0.02/0.13 | 0.67/0.69 | 0.034 / 1.1 / 0 | 0.35 | [22, 16] | 0.032 | - | 724 |
| 8 | lm7f_kit_gforce | **49.1** | 27.7/5.5/5.9/0.0/10.0 | 10/10 | 0.072 | -0.041 / -0.4 | 0.89 | 0.53/0.74 | 0.89 | 1.30/1.28/1.15/0.46 | 0.70/0.94 | 0.12/0.43 | 0.49/0.55 | 0.80/0.84 | 0.112 / 4.2 / 4 | 0.66 | [22, 22] | 0.028 | - | 753 |
| 9 | lm7f_kit_hips4 | **49.1** | 23.1/2.8/12.0/1.2/10.0 | 10/10 | 0.086 | -0.058 / -1.6 | 0.87 | 0.44/0.62 | 0.89 | 1.00/1.17/1.06/0.76 | 0.66/1.07 | -0.01/-0.02 | 0.09/0.24 | 0.55/0.64 | 0.023 / 0.5 / 2 | 0.40 | [19, 18] | 0.040 | - | 711 |
| 10 | lm7f_kit_gforce_pc_soft | **46.0** | 23.8/5.4/5.6/1.3/10.0 | 10/10 | 0.073 | -0.053 / -2.8 | 0.91 | 0.47/0.49 | 0.82 | 1.42/1.26/1.17/0.87 | 0.58/0.93 | 0.12/0.24 | 0.11/0.28 | 0.56/0.45 | 0.091 / -14.3 / 6 | 0.37 | [12, 11] | 0.031 | - | 845 |
| 11 | lm7f_kit_stand10 | **45.6** | 23.1/7.4/4.7/0.4/10.0 | 10/10 | 0.063 | -0.053 / -1.8 | 0.87 | 0.49/0.59 | 0.89 | 0.68/1.09/1.05/1.09 | 0.77/0.62 | -0.05/0.22 | 0.25/0.21 | 0.57/0.61 | 0.224 / -8.8 / 10 | 0.47 | [18, 19] | 0.035 | - | 640 |
| 12 | lm7f_kit_gforce_activation | **44.8** | 23.4/4.6/6.0/0.8/10.0 | 9/9 | 0.077 | -0.040 / 0.3 | 0.92 | 0.45/0.67 | 0.85 | 1.19/1.24/1.18/0.63 | 0.92/0.51 | 0.00/0.28 | 0.09/0.31 | 0.55/0.74 | 0.185 / -15.2 / 5 | 0.44 | [17, 19] | 0.031 | - | 821 |
| 13 | lm7f_kit_clear9 | **42.4** | 25.5/3.5/3.4/0.0/10.0 | 10/10 | 0.083 | -0.066 / -0.8 | 0.86 | 0.55/0.74 | 0.90 | 0.89/0.94/0.94/0.82 | 0.74/1.01 | 0.02/0.17 | 0.24/0.23 | 0.80/0.86 | 0.108 / -4.9 / 6 | 0.63 | [24, 22] | 0.034 | - | 703 |
| 14 | lm7f_kit_airtime | **39.1** | 23.6/0.0/3.4/2.1/10.0 | 10/10 | 0.101 | -0.066 / -1.6 | 0.92 | 0.49/0.59 | 0.88 | 1.24/1.21/1.21/0.56 | 0.68/0.94 | -0.10/-0.00 | 0.13/0.41 | 0.75/0.60 | 0.146 / -7.9 / 3 | 0.34 | [22, 13] | 0.035 | - | 589 |
| 15 | lm7f_kit_gforce_resets_angmom_posture | **17.6** | 10.7/0.0/7.0/0.0/0.0 | 0/9 | - | - / - | - | -/- | - | -0.02/0.01/0.01/-0.01 | 0.01/0.01 | -0.03/0.51 | 0.47/0.67 | 0.77/0.77 | 0.030 / -4.2 / 0 | 1.06 | [12, 14] | 0.000 | - | 1073 |
| 16 | lm7f_kit_gforce_resets_angmom | **16.6** | 11.2/0.0/5.4/0.0/0.0 | 0/8 | - | - / - | - | -/- | - | -0.06/0.00/-0.00/-0.01 | -0.06/0.04 | 0.29/0.33 | 0.62/0.53 | 0.81/0.74 | 0.046 / -3.4 / 0 | 0.98 | [9, 14] | 0.000 | - | 1000 |
| 17 | lm7f_kit_full | **15.2** | 9.8/0.0/5.4/0.0/0.0 | 0/7 | - | - / - | - | -/- | - | -0.02/0.01/0.01/0.01 | -0.06/0.06 | -0.02/0.22 | 0.52/0.59 | 0.79/0.71 | 0.046 / -3.6 / 1 | 0.98 | [9, 11] | 0.002 | - | 1113 |

## EXAM lm7f19b — 2026-10-28 19:56 — 2 policies, seed 1, homing n 10, balance p14h_stancewide, 2 instances, wall 9 min 35 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/lm7f19b_2026-10-28_19-46-43

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_gforce_bank | **48.0** | 22.5/6.6/7.2/1.6/10.0 | 10/10 | 0.067 | -0.030 / -5.4 | 0.89 | 0.45/0.71 | 0.87 | 1.09/1.11/1.06/0.61 | 0.70/0.33 | 0.01/0.30 | 0.16/0.43 | 0.45/0.77 | 0.098 / -0.4 / 3 | 0.37 | [10, 16] | 0.027 | - | 567 |
| 2 | lm7f_kit_footbonus6 | **47.4** | 24.4/7.1/5.6/0.3/10.0 | 10/10 | 0.065 | -0.051 / -2.3 | 0.85 | 0.57/0.69 | 0.88 | 1.00/1.05/1.03/0.75 | 0.46/1.35 | -0.03/0.20 | 0.22/0.28 | 0.84/0.58 | 0.092 / -16.4 / 3 | 0.48 | [23, 18] | 0.032 | - | 565 |

## EXAM lm7f19 — 2026-10-28 19:56 — 19 policies, seed 1, homing n 10, balance p14h_stancewide, 7 instances, wall 46 min 49 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/lm7f19_2026-10-28_19-08-03

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_gforce_airtime | **55.8** | 23.8/7.2/14.6/0.2/10.0 | 10/10 | 0.064 | -0.034 / 0.2 | 0.88 | 0.50/0.69 | 0.91 | 0.74/1.06/1.07/0.90 | 0.61/0.93 | -0.01/0.04 | 0.16/0.34 | 0.68/0.74 | 0.020 / -5.0 / 0 | 0.49 | [21, 22] | 0.029 | - | 1119 |
| 2 | lm7f_kit_alive10 | **54.6** | 23.5/4.5/15.0/1.7/10.0 | 10/10 | 0.078 | -0.041 / -3.9 | 0.88 | 0.47/0.66 | 0.85 | 1.09/1.33/1.12/0.58 | 0.55/0.99 | 0.16/-0.02 | 0.07/0.47 | 0.63/0.59 | 0.009 / 1.0 / 4 | 0.36 | [22, 21] | 0.037 | - | 633 |
| 3 | lm7f_kit_gforce_resets_posture | **54.4** | 27.3/6.7/10.5/0.0/10.0 | 10/10 | 0.067 | -0.052 / 2.7 | 0.88 | 0.52/0.68 | 0.85 | 1.31/1.22/0.99/0.72 | 0.74/1.04 | -0.05/0.46 | 0.38/0.52 | 0.62/0.74 | 0.043 / -8.9 / 1 | 0.72 | [21, 14] | 0.024 | - | 678 |
| 4 | lm7f_kit_gforce_drall | **52.5** | 19.0/7.2/13.3/3.1/10.0 | 10/10 | 0.064 | -0.040 / -2.9 | 0.89 | 0.55/0.43 | 0.88 | 0.76/0.97/1.01/0.59 | 0.16/0.22 | -0.06/0.24 | 0.24/0.33 | 0.82/0.38 | 0.028 / -16.6 / 8 | 0.24 | [23, 6] | 0.030 | - | 659 |
| 5 | lm7f_kit_footbonus6_still | **52.0** | 17.5/5.8/15.8/3.0/10.0 | 10/10 | 0.071 | -0.040 / -0.6 | 0.88 | 0.31/0.67 | 0.86 | -0.02/1.04/1.01/0.34 | 0.74/0.95 | -0.03/0.18 | 0.01/0.01 | 0.33/0.66 | 0.002 / 2.6 / 0 | 0.19 | [7, 14] | 0.031 | - | 621 |
| 6 | lm7f_kit_gforce_resets | **51.9** | 24.3/3.2/14.3/0.0/10.0 | 10/10 | 0.084 | -0.039 / -2.2 | 0.88 | 0.55/0.66 | 0.85 | 0.89/1.05/1.09/0.74 | 0.34/0.89 | 0.04/0.05 | 0.43/0.39 | 0.78/0.76 | 0.018 / 0.9 / 0 | 0.61 | [18, 17] | 0.029 | - | 847 |
| 7 | lm7f_kit_alive5 | **51.2** | 23.0/4.7/11.5/2.0/10.0 | 10/10 | 0.076 | -0.051 / -1.6 | 0.86 | 0.42/0.64 | 0.87 | 1.38/1.24/1.08/0.66 | 0.76/0.74 | -0.10/0.24 | -0.02/0.13 | 0.67/0.69 | 0.034 / 1.1 / 0 | 0.35 | [22, 16] | 0.032 | - | 724 |
| 8 | lm7f_kit_gforce | **49.1** | 27.7/5.5/5.9/0.0/10.0 | 10/10 | 0.072 | -0.041 / -0.4 | 0.89 | 0.53/0.74 | 0.89 | 1.30/1.28/1.15/0.46 | 0.70/0.94 | 0.12/0.43 | 0.49/0.55 | 0.80/0.84 | 0.112 / 4.2 / 4 | 0.66 | [22, 22] | 0.028 | - | 753 |
| 9 | lm7f_kit_hips4 | **49.1** | 23.1/2.8/12.0/1.2/10.0 | 10/10 | 0.086 | -0.058 / -1.6 | 0.87 | 0.44/0.62 | 0.89 | 1.00/1.17/1.06/0.76 | 0.66/1.07 | -0.01/-0.02 | 0.09/0.24 | 0.55/0.64 | 0.023 / 0.5 / 2 | 0.40 | [19, 18] | 0.040 | - | 711 |
| 10 | lm7f_kit_gforce_bank | **48.0** | 22.5/6.6/7.2/1.6/10.0 | 10/10 | 0.067 | -0.030 / -5.4 | 0.89 | 0.45/0.71 | 0.87 | 1.09/1.11/1.06/0.61 | 0.70/0.33 | 0.01/0.30 | 0.16/0.43 | 0.45/0.77 | 0.098 / -0.4 / 3 | 0.37 | [10, 16] | 0.027 | - | 567 |
| 11 | lm7f_kit_footbonus6 | **47.4** | 24.4/7.1/5.6/0.3/10.0 | 10/10 | 0.065 | -0.051 / -2.3 | 0.85 | 0.57/0.69 | 0.88 | 1.00/1.05/1.03/0.75 | 0.46/1.35 | -0.03/0.20 | 0.22/0.28 | 0.84/0.58 | 0.092 / -16.4 / 3 | 0.48 | [23, 18] | 0.032 | - | 565 |
| 12 | lm7f_kit_gforce_pc_soft | **46.0** | 23.8/5.4/5.6/1.3/10.0 | 10/10 | 0.073 | -0.053 / -2.8 | 0.91 | 0.47/0.49 | 0.82 | 1.42/1.26/1.17/0.87 | 0.58/0.93 | 0.12/0.24 | 0.11/0.28 | 0.56/0.45 | 0.091 / -14.3 / 6 | 0.37 | [12, 11] | 0.031 | - | 845 |
| 13 | lm7f_kit_stand10 | **45.6** | 23.1/7.4/4.7/0.4/10.0 | 10/10 | 0.063 | -0.053 / -1.8 | 0.87 | 0.49/0.59 | 0.89 | 0.68/1.09/1.05/1.09 | 0.77/0.62 | -0.05/0.22 | 0.25/0.21 | 0.57/0.61 | 0.224 / -8.8 / 10 | 0.47 | [18, 19] | 0.035 | - | 640 |
| 14 | lm7f_kit_gforce_activation | **44.8** | 23.4/4.6/6.0/0.8/10.0 | 9/9 | 0.077 | -0.040 / 0.3 | 0.92 | 0.45/0.67 | 0.85 | 1.19/1.24/1.18/0.63 | 0.92/0.51 | 0.00/0.28 | 0.09/0.31 | 0.55/0.74 | 0.185 / -15.2 / 5 | 0.44 | [17, 19] | 0.031 | - | 821 |
| 15 | lm7f_kit_clear9 | **42.4** | 25.5/3.5/3.4/0.0/10.0 | 10/10 | 0.083 | -0.066 / -0.8 | 0.86 | 0.55/0.74 | 0.90 | 0.89/0.94/0.94/0.82 | 0.74/1.01 | 0.02/0.17 | 0.24/0.23 | 0.80/0.86 | 0.108 / -4.9 / 6 | 0.63 | [24, 22] | 0.034 | - | 703 |
| 16 | lm7f_kit_airtime | **39.1** | 23.6/0.0/3.4/2.1/10.0 | 10/10 | 0.101 | -0.066 / -1.6 | 0.92 | 0.49/0.59 | 0.88 | 1.24/1.21/1.21/0.56 | 0.68/0.94 | -0.10/-0.00 | 0.13/0.41 | 0.75/0.60 | 0.146 / -7.9 / 3 | 0.34 | [22, 13] | 0.035 | - | 589 |
| 17 | lm7f_kit_gforce_resets_angmom_posture | **17.6** | 10.7/0.0/7.0/0.0/0.0 | 0/9 | - | - / - | - | -/- | - | -0.02/0.01/0.01/-0.01 | 0.01/0.01 | -0.03/0.51 | 0.47/0.67 | 0.77/0.77 | 0.030 / -4.2 / 0 | 1.06 | [12, 14] | 0.000 | - | 1073 |
| 18 | lm7f_kit_gforce_resets_angmom | **16.6** | 11.2/0.0/5.4/0.0/0.0 | 0/8 | - | - / - | - | -/- | - | -0.06/0.00/-0.00/-0.01 | -0.06/0.04 | 0.29/0.33 | 0.62/0.53 | 0.81/0.74 | 0.046 / -3.4 / 0 | 0.98 | [9, 14] | 0.000 | - | 1000 |
| 19 | lm7f_kit_full | **15.2** | 9.8/0.0/5.4/0.0/0.0 | 0/7 | - | - / - | - | -/- | - | -0.02/0.01/0.01/0.01 | -0.06/0.06 | -0.02/0.22 | 0.52/0.59 | 0.79/0.71 | 0.046 / -3.6 / 1 | 0.98 | [9, 11] | 0.002 | - | 1113 |

## EXAM tp5 — 2026-10-28 20:20 — 10 policies, seed 1, homing n 10, balance p14h_stancewide, 5 instances, wall 22 min 51 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/tp5_2026-10-28_19-57-30

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_gforce_airtime | **53.8** | 23.1/5.9/14.7/0.0/10.0 | 10/10 | 0.070 | -0.042 / 0.2 | 0.89 | 0.48/0.69 | 0.91 | 0.86/1.07/1.05/0.91 | 0.59/0.76 | -0.01/0.06 | 0.02/0.29 | 0.67/0.78 | 0.011 / 0.9 / 0 | 0.50 | [21, 22] | 0.031 | - | 574 |
| 2 | lm7f_kit_footbonus6 | **52.1** | 25.2/8.3/7.4/1.1/10.0 | 10/10 | 0.058 | -0.033 / -1.8 | 0.85 | 0.58/0.69 | 0.89 | 1.01/1.06/1.03/0.84 | 0.58/1.19 | -0.09/0.12 | 0.08/0.34 | 0.84/0.75 | 0.092 / -16.4 / 3 | 0.43 | [25, 21] | 0.030 | - | 487 |
| 3 | lm7f_kit_alive10 | **49.8** | 21.5/2.2/14.0/2.1/10.0 | 10/10 | 0.089 | -0.052 / -0.3 | 0.87 | 0.54/0.67 | 0.81 | 0.11/1.32/1.12/0.68 | 0.78/0.94 | 0.00/-0.04 | 0.04/0.48 | 0.74/0.70 | 0.009 / 1.0 / 4 | 0.35 | [23, 15] | 0.038 | - | 700 |
| 4 | lm7f_kit_alive5 | **47.6** | 22.1/2.9/10.7/1.9/10.0 | 10/10 | 0.086 | -0.059 / -0.1 | 0.87 | 0.42/0.62 | 0.88 | 1.38/1.24/1.08/0.66 | 0.72/0.80 | -0.03/0.15 | -0.01/0.12 | 0.44/0.73 | 0.034 / 1.1 / 0 | 0.34 | [19, 19] | 0.032 | - | 521 |
| 5 | lm7f_kit_footbonus6_still | **47.0** | 19.4/1.4/13.0/3.2/10.0 | 10/10 | 0.093 | -0.068 / -0.1 | 0.87 | 0.33/0.63 | 0.86 | -0.02/1.04/1.00/0.54 | 0.78/0.94 | -0.05/0.15 | 0.01/0.18 | 0.42/0.81 | 0.002 / 2.6 / 0 | 0.24 | [12, 19] | 0.031 | - | 627 |
| 6 | lm7f_kit_gforce | **44.0** | 24.1/4.6/5.3/0.0/10.0 | 10/10 | 0.077 | -0.047 / -2.2 | 0.89 | 0.51/0.70 | 0.87 | 1.43/1.27/1.16/0.35 | 0.49/1.02 | 0.19/0.35 | 0.40/0.24 | 0.41/0.80 | 0.112 / 4.2 / 4 | 0.67 | [21, 24] | 0.024 | - | 499 |
| 7 | lm7f_kit_gforce_activation | **43.8** | 23.4/5.6/4.1/0.8/10.0 | 10/10 | 0.072 | -0.059 / -0.7 | 0.92 | 0.45/0.70 | 0.82 | 1.26/1.20/1.20/0.68 | 0.88/0.71 | -0.02/0.26 | 0.02/0.35 | 0.42/0.71 | 0.156 / -11.1 / 5 | 0.43 | [15, 17] | 0.034 | - | 626 |
| 8 | lm7f_kit_clear9 | **40.0** | 25.1/0.0/4.9/0.0/10.0 | 10/10 | 0.103 | -0.051 / -4.3 | 0.87 | 0.58/0.74 | 0.88 | 0.89/0.94/0.93/0.74 | 0.61/1.09 | 0.08/0.16 | 0.23/0.32 | 0.76/0.86 | 0.123 / -6.8 / 4 | 0.71 | [23, 21] | 0.035 | - | 491 |
| 9 | lm7f_kit_airtime | **39.7** | 23.5/0.2/3.6/2.3/10.0 | 10/10 | 0.099 | -0.064 / 3.5 | 0.92 | 0.46/0.56 | 0.87 | 1.24/1.21/1.21/0.54 | 0.58/0.96 | -0.08/0.18 | 0.13/0.44 | 0.59/0.63 | 0.146 / -7.9 / 3 | 0.31 | [13, 14] | 0.035 | - | 531 |
| 10 | lm7f_kit_full | **15.9** | 10.0/0.0/5.9/0.0/0.0 | 0/10 | - | - / - | - | -/- | - | -0.01/0.01/0.01/0.01 | -0.06/0.07 | 0.01/0.21 | 0.59/0.53 | 0.81/0.74 | 0.041 / 0.3 / 2 | 0.93 | [10, 13] | 0.000 | - | 852 |

## EXAM detx_B — 2026-10-28 20:31 — 2 policies, seed 1, homing n 10, balance p14h_stancewide, 2 instances, wall 9 min 20 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/detx_B_2026-10-28_20-21-58

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_gforce_airtime | **55.3** | 24.0/5.7/15.2/0.4/10.0 | 10/10 | 0.072 | -0.037 / 0.0 | 0.88 | 0.51/0.73 | 0.91 | 0.89/1.02/1.06/0.82 | 0.61/1.07 | -0.01/0.06 | 0.07/0.33 | 0.63/0.79 | 0.011 / 0.9 / 0 | 0.47 | [21, 24] | 0.028 | - | 560 |
| 2 | lm7f_kit_footbonus6 | **48.1** | 24.5/6.7/6.8/0.1/10.0 | 10/10 | 0.066 | -0.040 / -2.0 | 0.85 | 0.56/0.62 | 0.88 | 1.00/1.05/1.03/0.75 | 0.46/1.35 | -0.03/0.20 | 0.22/0.31 | 0.80/0.62 | 0.092 / -16.4 / 3 | 0.50 | [23, 18] | 0.032 | - | 458 |

## EXAM detx_A — 2026-10-28 20:31 — 2 policies, seed 1, homing n 10, balance p14h_stancewide, 2 instances, wall 9 min 41 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/detx_A_2026-10-28_20-21-43

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_gforce_airtime | **59.0** | 24.1/8.6/15.9/0.4/10.0 | 10/10 | 0.057 | -0.030 / -2.6 | 0.88 | 0.48/0.70 | 0.91 | 0.89/1.02/1.06/0.82 | 0.61/1.07 | -0.01/0.06 | 0.07/0.33 | 0.63/0.80 | 0.011 / 0.9 / 0 | 0.47 | [21, 23] | 0.028 | - | 570 |
| 2 | lm7f_kit_footbonus6 | **47.3** | 24.4/6.1/6.4/0.3/10.0 | 10/10 | 0.069 | -0.043 / -2.6 | 0.84 | 0.57/0.65 | 0.89 | 1.00/1.05/1.03/0.75 | 0.46/1.35 | -0.03/0.20 | 0.22/0.28 | 0.84/0.58 | 0.092 / -16.4 / 3 | 0.48 | [23, 18] | 0.032 | - | 573 |

## EXAM seqA — 2026-10-29 08:35 — 1 policies, seed 1, homing n 10, balance p14h_stancewide, 1 instances, wall 7 min 17 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/seqA_2026-10-29_08-28-01

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_gforce_airtime | **58.9** | 24.0/9.4/15.1/0.3/10.0 | 10/10 | 0.053 | -0.038 / -1.5 | 0.89 | 0.50/0.70 | 0.89 | 0.89/1.02/1.06/0.82 | 0.61/1.07 | -0.01/0.06 | 0.07/0.33 | 0.63/0.80 | 0.011 / 0.9 / 0 | 0.48 | [21, 22] | 0.028 | - | 436 |

## EXAM seqB — 2026-10-29 08:42 — 1 policies, seed 1, homing n 10, balance p14h_stancewide, 1 instances, wall 7 min 14 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/seqB_2026-10-29_08-35-18

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_gforce_airtime | **58.9** | 24.0/9.4/15.1/0.3/10.0 | 10/10 | 0.053 | -0.038 / -1.5 | 0.89 | 0.50/0.70 | 0.89 | 0.89/1.02/1.06/0.82 | 0.61/1.07 | -0.01/0.06 | 0.07/0.33 | 0.63/0.80 | 0.011 / 0.9 / 0 | 0.48 | [21, 22] | 0.028 | - | 434 |

## EXAM dety_A — 2026-10-29 09:22 — 2 policies, seed 1, homing n 10, balance p14h_stancewide, 2 instances, wall 8 min 47 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/dety_A_2026-10-29_09-13-58

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_gforce_airtime | **53.2** | 23.0/6.5/13.6/0.0/10.0 | 10/10 | 0.067 | -0.040 / -0.4 | 0.88 | 0.49/0.70 | 0.90 | 0.77/0.98/1.07/0.97 | 0.74/0.62 | -0.01/0.06 | -0.03/0.31 | 0.67/0.79 | 0.024 / 0.9 / 0 | 0.50 | [22, 22] | 0.033 | - | 479 |
| 2 | lm7f_kit_footbonus6 | **47.4** | 24.8/5.7/5.5/1.3/10.0 | 10/10 | 0.071 | -0.060 / -0.4 | 0.85 | 0.57/0.69 | 0.89 | 0.90/1.05/1.04/0.82 | 0.50/1.13 | -0.02/0.45 | 0.05/0.31 | 0.80/0.62 | 0.085 / -10.6 / 4 | 0.41 | [26, 17] | 0.029 | - | 519 |

## EXAM dety_B — 2026-10-29 09:23 — 2 policies, seed 1, homing n 10, balance p14h_stancewide, 2 instances, wall 8 min 55 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/dety_B_2026-10-29_09-14-08

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_gforce_airtime | **55.1** | 23.8/6.7/14.7/0.0/10.0 | 10/10 | 0.067 | -0.044 / -0.1 | 0.88 | 0.49/0.72 | 0.89 | 0.79/1.09/1.03/0.84 | 0.64/0.88 | -0.01/0.03 | 0.19/0.34 | 0.68/0.73 | 0.008 / 1.5 / 0 | 0.52 | [27, 19] | 0.029 | - | 535 |
| 2 | lm7f_kit_footbonus6 | **44.9** | 24.2/3.9/5.9/1.0/10.0 | 10/10 | 0.081 | -0.057 / -2.6 | 0.85 | 0.56/0.63 | 0.87 | 0.91/1.06/1.03/0.85 | 0.50/0.92 | 0.05/0.15 | 0.04/0.28 | 0.84/0.70 | 0.085 / -10.6 / 4 | 0.44 | [24, 20] | 0.029 | - | 515 |

## EXAM det14_A — 2026-10-29 09:23 — 1 policies, seed 1, homing n 10, balance p14h_stancewide, 1 instances, wall 8 min 49 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/det14_A_2026-10-29_09-14-18

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_footbonus6_still | **44.1** | 22.0/3.6/5.8/2.7/10.0 | 10/10 | 0.082 | -0.066 / -2.3 | 0.87 | 0.37/0.69 | 0.86 | 0.57/1.13/0.99/0.66 | 0.68/1.02 | -0.08/0.22 | 0.01/0.26 | 0.51/0.71 | 0.077 / -10.5 / 2 | 0.28 | [20, 17] | 0.032 | - | 529 |

## EXAM det15_A — 2026-10-29 10:41 — 1 policies, seed 1, homing n 10, balance p14h_stancewide, 1 instances, wall 11 min 6 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/det15_A_2026-10-29_10-29-57

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_footbonus6_still | **52.3** | 19.1/6.0/14.9/2.3/10.0 | 10/10 | 0.070 | -0.049 / -0.9 | 0.88 | 0.30/0.69 | 0.85 | -0.02/1.07/1.04/0.59 | 0.66/0.91 | -0.07/0.09 | 0.12/0.20 | 0.36/0.80 | 0.002 / 2.8 / 0 | 0.30 | [8, 18] | 0.036 | - | 666 |

## EXAM detz_B — 2026-10-29 10:41 — 2 policies, seed 1, homing n 10, balance p14h_stancewide, 2 instances, wall 11 min 19 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/detz_B_2026-10-29_10-29-47

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_gforce_airtime | **54.0** | 21.8/5.9/15.9/0.4/10.0 | 10/10 | 0.070 | -0.037 / 0.8 | 0.88 | 0.48/0.68 | 0.91 | 0.73/1.06/1.08/0.80 | 0.62/0.48 | -0.01/0.09 | -0.03/0.34 | 0.69/0.77 | 0.004 / 0.3 / 0 | 0.47 | [22, 21] | 0.028 | - | 659 |
| 2 | lm7f_kit_footbonus6 | **51.0** | 24.9/5.1/10.1/0.9/10.0 | 10/10 | 0.074 | -0.027 / -4.2 | 0.84 | 0.58/0.69 | 0.88 | 0.91/1.06/1.05/0.85 | 0.66/1.09 | -0.09/0.24 | 0.13/0.28 | 0.83/0.57 | 0.072 / -10.2 / 3 | 0.44 | [24, 17] | 0.030 | - | 671 |

## EXAM detz_A — 2026-10-29 10:41 — 2 policies, seed 1, homing n 10, balance p14h_stancewide, 2 instances, wall 12 min 4 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/detz_A_2026-10-29_10-29-37

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_gforce_airtime | **54.4** | 21.7/6.1/16.1/0.5/10.0 | 10/10 | 0.070 | -0.035 / 1.4 | 0.88 | 0.49/0.71 | 0.91 | 0.73/1.06/1.08/0.79 | 0.39/0.71 | 0.02/0.08 | -0.02/0.35 | 0.72/0.72 | 0.004 / 0.3 / 0 | 0.47 | [20, 18] | 0.028 | - | 672 |
| 2 | lm7f_kit_footbonus6 | **51.4** | 24.9/4.9/10.9/0.7/10.0 | 10/10 | 0.075 | -0.056 / -0.7 | 0.84 | 0.56/0.64 | 0.88 | 1.03/1.06/1.03/0.83 | 0.44/1.08 | -0.16/0.27 | 0.28/0.27 | 0.82/0.57 | 0.035 / -13.7 / 5 | 0.45 | [24, 17] | 0.030 | - | 716 |

## EXAM det15_B — 2026-10-29 11:56 — 1 policies, seed 1, homing n 10, balance p14h_stancewide, 1 instances, wall 9 min 13 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/det15_B_2026-10-29_11-47-43

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_footbonus6_still | **54.9** | 20.0/6.7/15.1/3.1/10.0 | 10/10 | 0.066 | -0.047 / 0.6 | 0.87 | 0.35/0.66 | 0.87 | -0.02/1.06/1.02/0.58 | 0.74/1.09 | -0.03/0.15 | 0.01/0.21 | 0.53/0.76 | 0.002 / 2.8 / 0 | 0.26 | [14, 17] | 0.031 | - | 553 |

## EXAM detw_B — 2026-10-29 12:30 — 2 policies, seed 1, homing n 10, balance p14h_stancewide, 2 instances, wall 9 min 9 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/detw_B_2026-10-29_12-21-40

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_gforce_airtime | **55.1** | 22.1/7.4/15.6/0.0/10.0 | 10/10 | 0.063 | -0.040 / -0.0 | 0.88 | 0.50/0.69 | 0.91 | 0.78/1.09/1.03/0.91 | 0.61/0.53 | -0.01/0.04 | 0.03/0.30 | 0.66/0.77 | 0.004 / 0.3 / 0 | 0.50 | [24, 21] | 0.029 | - | 485 |
| 2 | lm7f_kit_footbonus6 | **52.2** | 23.8/7.0/10.6/0.8/10.0 | 10/10 | 0.065 | -0.042 / -0.6 | 0.85 | 0.55/0.69 | 0.88 | 0.98/1.06/1.06/0.78 | 0.53/0.88 | -0.06/0.11 | 0.22/0.31 | 0.85/0.49 | 0.052 / -14.6 / 2 | 0.44 | [22, 12] | 0.032 | - | 541 |

## EXAM detw_A — 2026-10-29 12:32 — 2 policies, seed 1, homing n 10, balance p14h_stancewide, 2 instances, wall 10 min 51 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/detw_A_2026-10-29_12-21-30

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_gforce_airtime | **56.1** | 23.0/6.8/15.8/0.5/10.0 | 10/10 | 0.066 | -0.037 / 1.7 | 0.88 | 0.51/0.68 | 0.90 | 0.78/1.09/1.03/0.74 | 0.61/0.81 | -0.00/0.12 | -0.03/0.35 | 0.70/0.80 | 0.004 / 0.3 / 0 | 0.46 | [20, 21] | 0.029 | - | 518 |
| 2 | lm7f_kit_footbonus6 | **53.3** | 24.1/8.2/10.0/1.0/10.0 | 10/10 | 0.059 | -0.028 / -0.0 | 0.83 | 0.56/0.61 | 0.88 | 0.98/1.04/1.04/0.80 | 0.55/0.60 | -0.11/0.25 | 0.14/0.30 | 0.82/0.79 | 0.072 / -10.2 / 3 | 0.44 | [23, 24] | 0.030 | - | 643 |

## EXAM detv_A — 2026-10-29 16:11 — 2 policies, seed 1, homing n 10, balance p14h_stancewide, 2 instances, wall 7 min 52 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/detv_A_2026-10-29_16-03-39

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_footbonus6 | **59.7** | 25.7/10.8/12.0/1.2/10.0 | 10/10 | 0.046 | -0.012 / -2.8 | 0.84 | 0.55/0.69 | 0.89 | 1.02/1.07/1.02/0.75 | 0.83/1.15 | -0.07/0.16 | 0.12/0.26 | 0.83/0.75 | 0.068 / -19.2 / 4 | 0.43 | [23, 21] | 0.030 | - | 362 |
| 2 | lm7f_kit_gforce_airtime | **51.1** | 20.6/6.7/13.8/0.0/10.0 | 10/10 | 0.067 | -0.046 / 0.4 | 0.88 | 0.48/0.69 | 0.90 | 0.08/1.01/1.07/0.89 | 0.61/0.73 | -0.04/0.04 | 0.01/0.40 | 0.64/0.78 | 0.015 / -4.4 / 0 | 0.53 | [24, 21] | 0.031 | - | 471 |

## EXAM detv_B — 2026-10-29 16:11 — 2 policies, seed 1, homing n 10, balance p14h_stancewide, 2 instances, wall 7 min 56 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/detv_B_2026-10-29_16-03-49

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_gforce_airtime | **47.9** | 17.8/5.7/14.4/0.0/10.0 | 10/10 | 0.071 | -0.048 / 0.1 | 0.88 | 0.50/0.71 | 0.91 | 0.05/0.04/0.76/0.88 | 0.78/1.04 | -0.02/0.07 | 0.09/0.27 | 0.64/0.78 | 0.008 / 3.1 / 0 | 0.52 | [21, 20] | 0.031 | - | 359 |
| 2 | lm7f_kit_footbonus6 | **42.4** | 25.6/1.2/5.0/0.6/10.0 | 10/10 | 0.094 | -0.079 / -0.1 | 0.84 | 0.56/0.66 | 0.89 | 0.91/1.06/1.05/0.85 | 0.66/1.09 | -0.09/0.20 | 0.15/0.34 | 0.83/0.74 | 0.072 / -10.2 / 3 | 0.46 | [23, 20] | 0.030 | - | 468 |

## EXAM land_lm7g — 2026-10-29 17:02 — 32 policies, seed 1, homing n 10, balance p14h_stancewide, 7 instances, wall 44 min 41 s

Score = 40 tracking + 20 placement + 20 stillness + 10 turn + 10 survival (tools/exam_report.py). Gains = achieved / commanded (homing: pose-differenced; staircase: sim truth). Stand left: + = the robot stands LEFT of the stop line (sign = side). Pivot = mean |foot yaw rate| of a loaded foot during the 0.5 rad/s turns (sliding turn high, stepping turn ~0). Records: /home/aspired-comp-2/Projects/robot_projects/repos/unitree_mujoco/mujoco_sim/logs/exam/land_lm7g_2026-10-29_16-14-10

| # | policy | score | trk/plc/still/turn/surv | homing arrived | settled planar m | stand left m / turned deg | gain vx | wz L/R | eff | stairs vx .05/.10/.15/-.10 | vy L/R | wz .3 L/R | wz .5 L/R | wz .8 L/R | stand10 drift m / yaw deg / lifts | pivot rad/s | lifts .8 | swing m | fell | t s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | lm7f_kit_alive5 | **55.7** | 22.8/5.9/14.6/2.4/10.0 | 10/10 | 0.070 | -0.035 / -2.7 | 0.86 | 0.39/0.64 | 0.87 | 1.57/1.31/1.10/0.72 | 0.77/0.79 | -0.08/0.11 | -0.02/0.16 | 0.56/0.74 | 0.019 / 1.0 / 1 | 0.32 | [19, 19] | 0.039 | - | 650 |
| 2 | lm7g_still_alive10 | **54.5** | 21.2/8.6/14.7/0.0/10.0 | 10/10 | 0.057 | -0.032 / 4.2 | 0.91 | 0.45/0.68 | 0.88 | -0.02/1.68/1.19/0.81 | 0.59/1.08 | -0.04/0.33 | -0.00/0.32 | 0.57/0.72 | 0.021 / -5.6 / 1 | 0.60 | [17, 15] | 0.091 | - | 414 |
| 3 | lm7f_kit_gforce_bank | **52.7** | 22.7/4.4/13.8/1.8/10.0 | 10/10 | 0.078 | -0.044 / -3.9 | 0.89 | 0.40/0.72 | 0.88 | 1.14/1.11/1.05/0.64 | 0.62/0.42 | 0.02/0.37 | 0.18/0.37 | 0.47/0.73 | 0.018 / -6.6 / 2 | 0.35 | [11, 20] | 0.029 | - | 407 |
| 4 | lm7f_kit_gforce_resets | **52.4** | 24.9/2.6/14.9/0.0/10.0 | 10/10 | 0.087 | -0.044 / 1.0 | 0.88 | 0.54/0.65 | 0.89 | 0.95/1.03/1.10/0.80 | 0.28/0.90 | 0.03/0.06 | 0.48/0.39 | 0.79/0.80 | 0.007 / 0.8 / 1 | 0.56 | [20, 18] | 0.030 | - | 414 |
| 5 | lm7g_energy_gated | **52.1** | 19.0/7.4/15.2/1.5/9.0 | 9/10 | 0.063 | -0.045 / -1.0 | 0.91 | 0.52/0.64 | 0.86 | -1.20/1.07/1.10/0.57 | 0.55/0.73 | -0.09/0.07 | 0.10/0.20 | 0.76/0.73 | 0.004 / 2.8 / 0 | 0.40 | [18, 16] | 0.027 | - | 564 |
| 6 | lm7f_kit_gforce_airtime | **51.8** | 19.9/6.4/14.9/0.6/10.0 | 10/10 | 0.068 | -0.041 / -0.2 | 0.88 | 0.49/0.68 | 0.90 | 0.08/0.72/1.07/0.81 | 0.65/0.99 | -0.01/0.02 | -0.01/0.31 | 0.63/0.74 | 0.010 / 2.0 / 0 | 0.45 | [22, 19] | 0.030 | - | 635 |
| 7 | lm7f_kit_alive10 | **51.7** | 23.6/1.1/15.3/1.7/10.0 | 10/10 | 0.095 | -0.039 / -3.4 | 0.88 | 0.51/0.67 | 0.85 | 1.50/1.27/1.13/0.70 | 0.53/0.72 | 0.14/0.25 | -0.05/0.28 | 0.75/0.73 | 0.007 / 1.1 / 1 | 0.39 | [20, 21] | 0.033 | - | 428 |
| 8 | lm7f_kit_gforce_resets_posture | **51.6** | 25.3/6.5/9.8/0.0/10.0 | 10/10 | 0.067 | -0.038 / -0.1 | 0.88 | 0.51/0.67 | 0.86 | 1.32/1.18/1.09/0.79 | 0.57/0.61 | -0.07/0.44 | 0.39/0.50 | 0.56/0.70 | 0.065 / -9.3 / 1 | 0.77 | [22, 13] | 0.024 | - | 423 |
| 9 | lm7f_kit_footbonus6 | **51.1** | 25.2/6.5/8.1/1.3/10.0 | 10/10 | 0.068 | -0.027 / -0.2 | 0.85 | 0.56/0.61 | 0.88 | 1.05/1.07/1.05/0.81 | 0.57/1.17 | -0.07/0.20 | 0.15/0.33 | 0.82/0.69 | 0.093 / -16.5 / 2 | 0.41 | [24, 17] | 0.031 | - | 533 |
| 10 | lm7f_kit_hips4 | **50.1** | 22.1/4.4/12.2/1.4/10.0 | 10/10 | 0.078 | -0.061 / -2.4 | 0.88 | 0.44/0.60 | 0.87 | 0.84/1.21/1.10/0.69 | 0.66/0.99 | 0.05/-0.03 | 0.02/0.17 | 0.53/0.70 | 0.018 / -1.4 / 1 | 0.39 | [19, 20] | 0.036 | - | 545 |
| 11 | lm7g_fric | **48.9** | 21.6/2.2/12.9/2.2/10.0 | 10/10 | 0.089 | -0.061 / -2.2 | 0.89 | 0.41/0.65 | 0.87 | 0.99/1.08/1.09/0.57 | 0.48/0.97 | -0.06/0.02 | -0.01/0.18 | 0.54/0.73 | 0.011 / -0.2 / 3 | 0.33 | [14, 20] | 0.022 | - | 684 |
| 12 | lm7f_kit_footbonus6_still | **48.2** | 20.7/7.3/7.2/3.0/10.0 | 10/10 | 0.064 | -0.028 / -1.6 | 0.87 | 0.31/0.64 | 0.85 | 0.07/1.07/1.02/0.65 | 0.86/1.06 | -0.06/0.14 | 0.01/0.24 | 0.45/0.79 | 0.128 / 4.0 / 1 | 0.26 | [15, 20] | 0.036 | - | 424 |
| 13 | lm7g_fric_pc | **48.2** | 21.8/5.3/9.7/1.4/10.0 | 10/10 | 0.074 | -0.046 / -0.8 | 0.87 | 0.44/0.63 | 0.89 | 1.11/1.14/1.11/0.58 | 0.58/0.47 | 0.10/0.25 | 0.09/0.42 | 0.48/0.55 | 0.058 / -7.1 / 4 | 0.36 | [12, 14] | 0.030 | - | 536 |
| 14 | lm7g_trunk_still | **47.9** | 20.7/5.6/10.8/0.8/10.0 | 10/10 | 0.072 | -0.063 / -0.3 | 0.90 | 0.51/0.61 | 0.88 | 0.76/1.11/1.10/0.61 | 0.42/1.09 | -0.06/0.07 | -0.07/0.06 | 0.62/0.69 | 0.028 / -5.4 / 2 | 0.44 | [18, 15] | 0.030 | - | 621 |
| 15 | lm7g_pivot | **46.7** | 22.1/4.6/5.2/4.8/10.0 | 10/10 | 0.077 | -0.048 / -0.6 | 0.92 | 0.55/0.60 | 0.87 | 1.13/1.19/1.14/0.72 | 0.37/0.54 | -0.08/0.07 | 0.12/0.34 | 0.77/0.70 | 0.190 / -11.4 / 8 | 0.18 | [22, 19] | 0.035 | - | 419 |
| 16 | lm7f_kit_gforce_pc_soft | **46.6** | 23.1/5.9/6.1/1.5/10.0 | 10/10 | 0.070 | -0.039 / -0.7 | 0.91 | 0.47/0.46 | 0.83 | 1.41/1.27/1.18/0.82 | 0.63/0.95 | -0.09/0.22 | 0.17/0.26 | 0.46/0.42 | 0.148 / -11.6 / 4 | 0.33 | [13, 8] | 0.030 | - | 570 |
| 17 | lm7f_kit_gforce | **45.3** | 26.2/3.7/5.4/0.0/10.0 | 10/10 | 0.082 | -0.046 / -1.3 | 0.89 | 0.54/0.72 | 0.88 | 1.42/1.28/1.15/0.31 | 0.54/0.99 | 0.11/0.43 | 0.39/0.38 | 0.86/0.85 | 0.113 / 3.9 / 4 | 0.68 | [23, 23] | 0.026 | - | 544 |
| 18 | lm7g_trunk | **43.8** | 21.7/2.7/7.7/1.7/10.0 | 10/10 | 0.086 | -0.042 / 0.5 | 0.90 | 0.50/0.55 | 0.89 | 1.21/1.27/1.15/0.86 | -0.09/0.82 | 0.06/0.25 | 0.19/0.20 | 0.54/0.59 | 0.081 / -7.1 / 4 | 0.35 | [15, 14] | 0.027 | - | 402 |
| 19 | lm7f_kit_gforce_drall | **43.0** | 18.8/5.3/6.2/2.7/10.0 | 10/10 | 0.073 | -0.060 / -0.3 | 0.90 | 0.54/0.42 | 0.87 | 0.71/0.91/0.97/0.64 | 0.17/0.23 | -0.05/0.33 | 0.19/0.36 | 0.77/0.35 | 0.078 / -5.9 / 6 | 0.26 | [24, 7] | 0.031 | - | 433 |
| 20 | lm7g_bank | **42.6** | 20.6/0.9/9.4/1.7/10.0 | 10/10 | 0.095 | -0.068 / -1.5 | 0.85 | 0.50/0.52 | 0.87 | 0.89/1.05/1.06/0.68 | 0.27/1.28 | 0.05/-0.01 | 0.21/0.09 | 0.54/0.42 | 0.038 / -7.6 / 3 | 0.32 | [15, 7] | 0.025 | - | 644 |
| 21 | lm7f_kit_clear9 | **41.7** | 26.0/2.0/3.7/0.0/10.0 | 10/10 | 0.090 | -0.064 / -2.4 | 0.86 | 0.52/0.74 | 0.89 | 1.02/0.99/0.94/0.61 | 0.68/1.24 | 0.18/0.20 | 0.29/0.30 | 0.74/0.86 | 0.099 / -9.2 / 4 | 0.66 | [23, 25] | 0.033 | - | 393 |
| 22 | lm7g_pc | **41.7** | 17.5/2.4/10.5/1.3/10.0 | 10/10 | 0.088 | -0.074 / 1.2 | 0.90 | 0.49/0.65 | 0.88 | -0.10/0.56/1.04/0.79 | 0.40/0.67 | -0.04/0.03 | 0.15/0.27 | 0.61/0.76 | 0.022 / 4.4 / 0 | 0.41 | [16, 14] | 0.106 | - | 415 |
| 23 | lm7g_drall | **40.9** | 17.6/6.3/4.5/2.5/10.0 | 10/10 | 0.069 | -0.055 / -2.8 | 0.92 | 0.23/0.58 | 0.83 | 0.65/1.06/1.11/0.48 | 0.17/0.37 | 0.20/0.07 | 0.20/0.14 | 0.38/0.62 | 0.202 / -3.5 / 3 | 0.25 | [10, 12] | 0.030 | - | 714 |
| 24 | lm7f_kit_airtime | **39.6** | 23.6/0.0/4.1/1.9/10.0 | 10/10 | 0.103 | -0.059 / 1.7 | 0.92 | 0.46/0.58 | 0.87 | 1.26/1.23/1.18/0.58 | 0.63/0.92 | 0.02/0.18 | 0.07/0.39 | 0.64/0.67 | 0.108 / -4.1 / 2 | 0.35 | [18, 14] | 0.035 | - | 404 |
| 25 | lm7g_airtime_clear9 | **39.6** | 19.8/3.8/6.0/0.0/10.0 | 10/10 | 0.081 | -0.040 / 0.3 | 0.92 | 0.52/0.70 | 0.82 | 1.24/1.29/1.19/0.33 | 0.53/0.78 | -0.06/-0.07 | 0.04/0.07 | 0.43/0.77 | 0.134 / 1.6 / 0 | 0.70 | [19, 24] | 0.032 | - | 720 |
| 26 | lm7g_still_stand15 | **38.2** | 24.8/0.6/3.9/0.0/8.9 | 8/9 | 0.097 | -0.061 / -3.4 | 0.91 | 0.50/0.71 | 0.85 | 0.99/1.20/1.17/0.47 | 0.76/1.38 | 0.07/0.17 | 0.28/0.30 | 0.67/0.75 | 0.206 / 1.2 / 3 | 0.60 | [17, 15] | 0.023 | - | 628 |
| 27 | lm7f_kit_gforce_activation | **38.1** | 24.5/0.4/2.5/0.7/10.0 | 10/10 | 0.098 | -0.075 / 2.5 | 0.92 | 0.42/0.64 | 0.83 | 1.06/1.17/1.20/0.73 | 0.68/0.83 | -0.04/0.41 | 0.15/0.36 | 0.48/0.72 | 0.179 / -6.1 / 6 | 0.44 | [18, 18] | 0.026 | - | 557 |
| 28 | lm7f_kit_stand10 | **36.5** | 25.1/0.0/1.0/0.4/10.0 | 10/10 | 0.105 | -0.090 / -2.6 | 0.88 | 0.49/0.62 | 0.86 | 0.78/1.07/1.00/1.08 | 0.82/0.65 | 0.01/0.38 | 0.39/0.32 | 0.60/0.60 | 0.266 / -5.2 / 12 | 0.47 | [22, 19] | 0.033 | - | 390 |
| 29 | lm7g_energy_cot | **35.9** | 7.5/2.8/15.5/1.1/9.0 | 9/10 | 0.086 | -0.026 / 2.9 | 0.93 | 0.40/0.39 | 0.84 | -0.12/-0.00/1.04/-0.02 | 0.08/0.08 | -0.00/0.00 | 0.01/0.27 | 0.44/0.37 | 0.019 / 6.4 / 0 | 0.37 | [7, 7] | 0.057 | - | 711 |
| 30 | lm7f_kit_gforce_resets_angmom | **19.9** | 12.4/0.0/7.5/0.0/0.0 | 0/10 | - | - / - | - | -/- | - | -0.05/-0.01/-0.00/0.03 | -0.05/0.07 | 0.30/0.66 | 0.62/0.53 | 0.80/0.70 | 0.025 / -4.2 / 0 | 0.99 | [12, 14] | 0.000 | - | 709 |
| 31 | lm7f_kit_full | **17.4** | 10.3/0.0/7.1/0.0/0.0 | 0/10 | - | - / - | - | -/- | - | -0.00/0.02/0.01/-0.01 | -0.05/0.06 | 0.01/0.24 | 0.61/0.63 | 0.81/0.71 | 0.029 / -3.2 / 2 | 0.93 | [12, 12] | 0.004 | - | 993 |
| 32 | lm7f_kit_gforce_resets_angmom_posture | **15.3** | 7.9/0.0/7.4/0.0/0.0 | 0/10 | - | - / - | - | -/- | - | -0.01/-0.02/0.01/-0.04 | -0.03/0.01 | -0.03/0.28 | 0.06/0.54 | 0.72/0.76 | 0.026 / -4.3 / 0 | 0.68 | [12, 14] | 0.001 | - | 697 |

