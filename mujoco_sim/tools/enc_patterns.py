#!/usr/bin/env python3
"""enc_patterns.py -- the FIXED encoder zero-offset patterns of Test 6 of EXAM #3 (2026-11-13, results/EXAM2_SPEC.md sections 12 + 13).

Why a file of its own: the patterns are the exam. They must be the same numbers for every policy and every run (nothing is drawn at
exam time), and three readers need them: the commander in the rig (MovementModule/Utils/homing_commander.py sends them to the sim as
"enc_at <sim_t> <27 values>"), the scorer (tools/exam_score.py names them in the record) and a person reading a record.

Convention (the robot's, BridgeModule/main/encoder_offsets.json; the training's, unitree_rl_lab mdp/dr.py randomize_encoder_offset):
    reported angle = true angle + offset            (rad, per motor, SDK order = the H1-2 MJCF order)
The sim publishes q_true + offset and its PD runs on that reading, tau = kp (q_cmd - (q_true + offset)) + ..., which is the same as
applying the target at q_cmd - offset. Worked: offset +0.010 rad on an ankle pitch held at a target of -0.200 rad -> the joint settles
where the encoder READS -0.200, i.e. truly at -0.210 rad (0.57 deg further), and the policy never sees the difference.

The 13 joints = the training band's (lm7l_env_cfg.SW_ENC01_JOINTS: every leg joint + the torso); the arms stay 0. BAND = the trained
half-width (SW_ENC01_HALF = 0.010 rad = 0.57 deg).

Patterns (P0 first: the paired baseline every other pattern is compared with, same policy, same run):
  P0 zero          perfect encoders
  P1 resid_band    + the robot's measured residual per joint group (RESID, study_env_cfg.py), CLIPPED to the band: ankle pitch, hip
                   roll and ankle roll sit AT the band's edge (0.010), hip yaw 0.005, hip pitch / knee 0.004 keep their measured size.
                   The stop tests measure pair sums / differences, not a sign per joint, so "+" = every joint in its own positive
                   direction: a sagittal part (pitch joints, the same on both legs) + a lateral part (roll / yaw joints the same sign
                   on both legs = both legs rolled to one side).
  P2 resid_mirror  P1 mirrored left <-> right = the same error on the other side: pitch joints unchanged, roll / yaw joints sign-
                   flipped (and the sides swapped; RESID is side-symmetric, so only the signs move). (P1 + P2) / 2 reads the sagittal
                   response, (P1 - P2) / 2 the lateral one.
  P3 rand_a, P4 rand_b   two fixed draws, uniform +-BAND over the 13 joints: numpy 2.4.5 default_rng(SEED).uniform(-BAND, BAND, 13)
                   twice in a row, rounded to 0.1 mrad. HARD-CODED below (a numpy update must not change the exam); --check re-draws
                   and compares.
  P5 resid_full    STRESS, outside the trained band: + the measured residuals unscaled (ankle pitch 0.0305 rad = 1.75 deg = 3 x the
                   band). In Exam #3 this is the pattern that is SCORED: its 15 s stand (base drift, foot lifts: the constraints
                   enc_stand_drift / enc_stand_lifts and stability points) and the 8 offset pushes run under it. The stress flag below
                   is how the commander and the scorer find it; keep it the LAST pattern (the pushes follow its walk).

    python3 tools/enc_patterns.py            # print the table (mrad)
    python3 tools/enc_patterns.py --check    # re-draw P3 / P4 from SEED, check the mirror and the band; exit 1 on a mismatch
"""
import sys

JOINTS = ["L_hip_yaw", "L_hip_pitch", "L_hip_roll", "L_knee", "L_ankle_pitch", "L_ankle_roll",
          "R_hip_yaw", "R_hip_pitch", "R_hip_roll", "R_knee", "R_ankle_pitch", "R_ankle_roll", "torso",
          "L_sh_pitch", "L_sh_roll", "L_sh_yaw", "L_elbow", "L_wr_roll", "L_wr_pitch", "L_wr_yaw",
          "R_sh_pitch", "R_sh_roll", "R_sh_yaw", "R_elbow", "R_wr_roll", "R_wr_pitch", "R_wr_yaw"]   # SDK motor order = the MJCF order (27)
N_MOTOR = len(JOINTS)
ENC_JOINTS = list(range(13))     # the trained band's joints: 12 leg joints + the torso
BAND = 0.010                     # rad: the trained half-width (lm7l_env_cfg.SW_ENC01_HALF)
SEED = 20261113                  # P3 / P4: numpy default_rng(SEED), two consecutive uniform(-BAND, BAND, 13) draws

# the robot's measured residual per joint group, rad (unitree_rl_lab-ik .../robots/h1_2/study_env_cfg.py RESID, stop tests 10-07 / 11-06);
# a copy on purpose: the exam must not move when the training table is edited -- --check does not compare them, the spec names the date
RESID = {"hip_yaw": 0.005, "hip_pitch": 0.004, "hip_roll": 0.020, "knee": 0.004, "ankle_pitch": 0.0305, "ankle_roll": 0.013, "torso": 0.0}
# a left <-> right mirror keeps a pitch angle and flips a roll / yaw angle (both legs' joint axes point the same way in the MJCF:
# yaw z, pitch y, roll x)
MIRROR_SIGN = {"hip_yaw": -1.0, "hip_pitch": 1.0, "hip_roll": -1.0, "knee": 1.0, "ankle_pitch": 1.0, "ankle_roll": -1.0, "torso": -1.0}

_RAND_A = [-0.0063, 0.0068, -0.0058, -0.0065, 0.0009, 0.0042, -0.0001, -0.0090, -0.0042, -0.0066, 0.0061, -0.0040, -0.0097]
_RAND_B = [0.0058, 0.0052, -0.0007, 0.0056, 0.0098, 0.0018, 0.0082, -0.0068, -0.0039, 0.0084, -0.0064, -0.0063, 0.0008]


def _group(name):
    return name if name == "torso" else name[2:]


def _full(v13):
    return [round(float(v), 5) for v in v13] + [0.0] * (N_MOTOR - 13)


def _resid(clip):
    return _full([min(RESID[_group(JOINTS[i])], clip) for i in ENC_JOINTS])


def mirror(v):
    """the same error on the other side: left <-> right swapped, roll / yaw joints sign-flipped (27 values in, 27 out)"""
    out = [0.0] * N_MOTOR
    for i, n in enumerate(JOINTS):
        if n == "torso":
            out[i] = MIRROR_SIGN["torso"] * v[i]
        else:
            j = JOINTS.index(("R_" if n[0] == "L" else "L_") + n[2:])
            out[i] = MIRROR_SIGN.get(n[2:], 1.0) * v[j] if i < 13 else v[j]   # the arms carry no offset in any pattern (kept for completeness)
    return [round(x + 0.0, 5) for x in out]   # + 0.0: no "-0.0" on the wire


# (key, name, 27 offsets rad, stress): the ORDER is the exam's order -- P0 first (the baseline), the stress pattern last
PATTERNS = [
    ("P0", "zero", [0.0] * N_MOTOR, False),
    ("P1", "resid_band", _resid(BAND), False),
    ("P2", "resid_mirror", mirror(_resid(BAND)), False),
    ("P3", "rand_a", _full(_RAND_A), False),
    ("P4", "rand_b", _full(_RAND_B), False),
    ("P5", "resid_full", _resid(1.0), True),
]
BY_KEY = {p[0]: p for p in PATTERNS}


def meta():
    """what a record carries about the patterns (the numbers themselves, so a record is readable without this file)"""
    return {"band": BAND, "seed": SEED, "joints": JOINTS[:13],
            "patterns": [{"key": k, "name": n, "stress": s, "offsets": v[:13], "max_abs": max(abs(x) for x in v),
                          "norm": round(sum(x * x for x in v) ** 0.5, 5)} for k, n, v, s in PATTERNS]}


def wire(key):
    """the pattern as the sim's command argument: 'zero' or 27 numbers"""
    v = BY_KEY[key][2]
    return "zero" if not any(v) else " ".join(f"{x:.5f}" for x in v)


def check():
    ok = True
    try:
        import numpy as np
        rng = np.random.default_rng(SEED)
        for nm, hard in (("P3 rand_a", _RAND_A), ("P4 rand_b", _RAND_B)):
            draw = [round(float(x), 4) for x in rng.uniform(-BAND, BAND, 13)]
            same = draw == hard
            ok &= same
            print(f"{nm}: {'matches' if same else 'DIFFERS FROM'} numpy {np.__version__} default_rng({SEED})" + ("" if same else f"\n   drawn {draw}\n   file  {hard}"))
    except ImportError:
        print("numpy missing: P3 / P4 not re-drawn (the hard-coded numbers are the exam)")
    for k, n, v, s in PATTERNS:
        if len(v) != N_MOTOR or any(v[13:]):
            ok = False; print(f"{k}: needs {N_MOTOR} values with zero arms")
        if not s and max(abs(x) for x in v) > BAND + 1e-9:
            ok = False; print(f"{k}: outside the band {BAND}")
    if mirror(BY_KEY["P2"][2]) != BY_KEY["P1"][2]:
        ok = False; print("P2 mirrored is not P1")
    print("check " + ("OK" if ok else "FAILED"))
    return ok


if __name__ == "__main__":
    if "--check" in sys.argv:
        sys.exit(0 if check() else 1)
    print(f"encoder offset patterns, mrad (reported = true + offset); band +-{1000 * BAND:g} mrad, seed {SEED}")
    print(f"{'joint':15s}" + "".join(f"{k + ' ' + n:>17s}" for k, n, _, _ in PATTERNS))
    for i in ENC_JOINTS:
        print(f"{JOINTS[i]:15s}" + "".join(f"{1000 * v[i]:17.1f}" for _, _, v, _ in PATTERNS))
    print(f"{'|max| / norm':15s}" + "".join(f"{f'{1000 * max(abs(x) for x in v):.1f} / {1000 * sum(x * x for x in v) ** 0.5:.1f}':>17s}" for _, _, v, _ in PATTERNS))
