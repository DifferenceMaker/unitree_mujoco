"""exam_score.py -- THE score of the standardized exam, in one place (used by exam_report.py and exam_record.py).

Score (0-100, every term bounded; a ranking aid, the weights are a choice -- results/EXAM_METRICS.md):
  tracking  40  mean over the staircase's command segments of min(gain, 1) (gain = achieved / commanded, sim truth; a fall = 0)
  placement 20  1 - min(settled planar mean / 0.10 m, 1)                  (homing)
  stillness 20  half: 1 - min(stand-10 s drift / 0.10 m, 1)  (staircase)   half: 1 - min(|stand after the switch, lateral| / 0.10 m, 1) (homing)
  turn      10  (1 - min(pivot / 0.5 rad/s, 1)) x min(mean |gain| at 0.8, 1)  (slide = 0, no turn = 0, stepping turn ~1)
  survival  10  homing arrivals / n, times 0 if the staircase fell
"""
import json
import os
import statistics as st

SEGS = ["vx05", "vx10", "vx15", "vxm10", "vyL10", "vyR10", "wzL03", "wzR03", "wzL05", "wzR05", "wzL08", "wzR08"]


def mean(v, d=None):
    return st.mean(v) if v else d


def clamp01(x):
    return max(0.0, min(1.0, x))


def summarize(homing, stairs):
    """the derived numbers of one policy: homing means, staircase gains, stance numbers (None where absent)"""
    r = {}
    ok = [x for x in homing if x.get("result") == "ARRIVED"]
    r["n"] = len(homing); r["arrived"] = len(ok)
    if ok:
        r["planar"] = mean([x["final_err"]["planar_m"] for x in ok])
        r["planar_sw"] = st.median([x["err_at_switch"]["planar_m"] for x in ok if x.get("err_at_switch")] or [float("nan")])
        r["head"] = mean([abs(x["final_err"]["heading_deg"]) for x in ok])
        sd = [x["settle_drift"] for x in ok if x.get("settle_drift")]
        r["stand_left"] = mean([-d["lateral_m"] for d in sd]); r["stand_turn"] = mean([-d["heading_deg"] for d in sd])
        tr = [x.get("track") or {} for x in ok]
        r["g_vx"] = mean([t["vx"][0] for t in tr if t.get("vx")]); r["g_wzL"] = mean([t["wz_inplace_left"][0] for t in tr if t.get("wz_inplace_left")])
        r["g_wzR"] = mean([t["wz_inplace_right"][0] for t in tr if t.get("wz_inplace_right")])
        r["eff"] = mean([x["path_eff"] for x in ok if x.get("path_eff")]); r["t_mean"] = mean([x["t_total"] for x in ok])
    segs = {s["name"]: s for s in (stairs or {}).get("segments", [])} if stairs else {}
    stands = [s for s in (stairs or {}).get("segments", []) if s["name"] == "stand"] if stairs else []
    r["fell"] = (stairs or {}).get("fell") if stairs else None
    r["stairs"] = (stairs or {}).get("result") if stairs else None
    g = {}
    for k in SEGS:
        s = segs.get(k); gv = None
        if s and s.get("gain"): gv = next(iter(s["gain"].values()))
        g[k] = gv
    r["gains"] = g
    if stands:
        s0 = stands[0]
        r["stand10_drift"] = s0.get("drift_xy"); r["stand10_yaw"] = s0.get("yaw_change_deg"); r["stand10_lifts"] = sum(s0.get("lifts") or [0, 0])
        r["stand_lifts_all"] = sum(sum(s.get("lifts") or [0, 0]) for s in stands)
        for k in ("sep_mean", "sep_trend", "stagger_mean", "toe_L", "toe_R", "jitter_knee", "jitter_hip"):   # stance metrics (2026-10-29)
            if s0.get(k) is not None: r["stand10_" + k] = s0[k]
    if segs.get("vx10"):
        r["vx10_yaw"] = segs["vx10"].get("yaw_change_deg"); r["vx10_left"] = (segs["vx10"].get("dx_body") or [None, None])[1]
        for k in ("jitter_knee", "jitter_hip", "sep_mean", "stagger_mean"):
            if segs["vx10"].get(k) is not None: r["vx10_" + k] = segs["vx10"][k]
    piv = [segs[k]["pivot_rad_s"] for k in ("wzL05", "wzR05", "wzL08", "wzR08") if segs.get(k) and segs[k].get("pivot_rad_s") is not None]
    r["pivot05"] = mean(piv); r["lifts05"] = [sum(segs[k].get("lifts") or [0, 0]) for k in ("wzL08", "wzR08") if segs.get(k)]
    r["swing"] = max([max(segs[k].get("swing_height") or [0, 0]) for k in ("vx10", "vx15") if segs.get(k)] or [None])
    return r


def score_policy(homing, stairs):
    r = summarize(homing, stairs)
    g = r["gains"]
    trk = mean([clamp01(v) if v is not None and v > 0 else 0.0 for v in g.values()], 0.0) if stairs else 0.0
    fell = bool(r["fell"]) or (stairs is not None and stairs.get("result") != "DONE")
    if fell: trk = 0.0
    ok = r["arrived"] > 0
    plc = 1 - min((r.get("planar") or 1.0) / 0.10, 1.0) if ok else 0.0
    stl = 0.5 * (1 - min((r.get("stand10_drift") if r.get("stand10_drift") is not None else 1.0) / 0.10, 1.0)) + \
          0.5 * (1 - min(abs(r.get("stand_left") if r.get("stand_left") is not None else 1.0) / 0.10, 1.0))
    g08 = [abs(g[k]) for k in ("wzL08", "wzR08") if g.get(k) is not None]
    trn = (1 - min((r.get("pivot05") if r.get("pivot05") is not None else 0.5) / 0.5, 1.0)) * (min(mean(g08, 0.0), 1.0) if g08 else 0.0)
    srv = (r["arrived"] / r["n"] if r["n"] else 0.0) * (0.0 if fell else 1.0)
    parts = {"tracking": round(40 * trk, 1), "placement": round(20 * plc, 1), "stillness": round(20 * stl, 1), "turn": round(10 * trn, 1), "survival": round(10 * srv, 1)}
    r["parts"] = parts; r["score"] = round(sum(parts.values()), 1)
    return r


# ═══ EXAM #2 (2026-10-30, results/EXAM2_SPEC.md): score by QUALITY, the tests are instruments ═══
# Every metric becomes 0..1 through one ramp between a good value g (full marks) and a bad value b (zero); g > b works too (higher is
# better). A metric row = (quality, test, key, label, value, unit, g, b, weight); points = weight x ramp. Weights per quality (operator
# 2026-10-30): survival 10, stability 30, tracking 20, gait & body care 10, energy 10, pushes 10, homing 10. "prov" = g/b are first
# guesses until the calibration run (spec section 6) sets them from real distributions.
QUALITIES = {"survival": 10, "stability": 30, "tracking": 20, "gait": 10, "energy": 10, "pushes": 10, "homing": 10}
BODY_MASS = 77.27
W_BODY = BODY_MASS * 9.81
LEVER = 0.5   # m: a yaw rate counts as LEVER x wz of linear speed in the tracking error (c0 0.1 m/s <-> 0.2 rad/s)


def ramp(x, g, b):
    if x is None or x != x:
        return 0.0
    return clamp01((b - x) / (b - g))


E_FLOOR = 0.1      # m/s: the rung error's denominator floor c0, v2 / v3
E_FLOOR_V4 = 0.05  # v4 (2026-11-09, small-command precision): c = 0.05 achieved 0.03 -> e = 0.02 / 0.1 = 0.2 (v3) vs 0.02 / 0.05 = 0.4 (v4)


def rung_error(cmd, v, floor=E_FLOOR):
    """tracking error of one rung: ||v - c|| / max(||c||, floor) on (vx, vy, LEVER wz) -- overshoot and off-axis motion count.
    floor = 0.1 m/s in v2 / v3 (= 0.2 rad/s on a wz rung), 0.05 m/s in v4 (= 0.1 rad/s; the LEVER stays)"""
    c = (cmd[0], cmd[1], LEVER * cmd[2]); w = (v[0], v[1], LEVER * v[2])
    nc = (c[0] ** 2 + c[1] ** 2 + c[2] ** 2) ** 0.5
    return ((w[0] - c[0]) ** 2 + (w[1] - c[1]) ** 2 + (w[2] - c[2]) ** 2) ** 0.5 / max(nc, floor)


def _axis(cmd):
    k = max(range(3), key=lambda i: abs(cmd[i]))
    return ("vx", "vy", "wz")[k] if abs(cmd[k]) > 1e-9 else None


def ladder_parts(lad):
    """the ladder record split: the 20 s stand, the rungs (by axis), the stops (with the rung before), the falls"""
    segs = [s for s in (lad or {}).get("segments", []) if s.get("name") != "reset"]
    stand = next((s for s in segs if s["name"] == "stand"), None)
    rungs = [s for s in segs if s["name"] not in ("stand", "stop", "bal_settle", "settle", "walk_pre", "walk_stand") and s.get("cmd") and _axis(s["cmd"])]
    stops = []
    for i, s in enumerate(segs):
        if s["name"] == "stop":
            prev = segs[i - 1] if i > 0 else None
            stops.append((s, prev["name"] if prev else None, _axis(prev["cmd"]) if prev and prev.get("cmd") else None))
    return stand, rungs, stops


EXPECTED_RUNGS = {"vx": 30, "vy": 12, "wz": 32}
EXPECTED_STOPS = 6
EXPECTED_PUSHES = 24      # v2 / v3: 3 levels x 4 sides x stand / walk
EXPECTED_PUSHES_V4 = 48   # v4 (2026-11-09): 3 levels x 8 directions (the sides + the diagonals) x stand / walk
PUSH_DIRS_V4 = ("front", "back", "left", "right", "front_left", "front_right", "back_left", "back_right")   # = homing_commander.PUSH_DIRS
EXPECTED_SWITCHES = 6
LADDER_TOPS = {"vx": (1.00, 0.50), "vy": (0.30, 0.30), "wz": (0.80, 0.80)}   # = homing_commander.LADDER_AXES (top forward, top backward)


# Every good / bad end in ONE table (calibration 2026-10-31): key -> (g, b). results/exam2_calibration.json (tools/exam_calibrate.py)
# overrides it (only the PROV keys below; the rest are definitions the calibration keeps).
CAL_DEFAULT = {
    "stand_drift": (0.01, 0.10), "stand_yaw": (1, 10), "stand_lifts": (0, 3), "stand_rate": (0.02, 0.10), "stand_sep_trend": (0.0, 0.03),
    "stand_stagger": (0.01, 0.08), "stand_toe": (2, 10), "stand_jitter": (0.2, 1.0),
    "stop_settle": (1.0, 4.0), "stop_steps": (1, 5), "stop_resid": (0.005, 0.05), "stop_over_m": (0.10, 0.60), "stop_over_deg": (5, 30),
    "walk_tilt": (1.5, 6.0), "walk_rate": (0.15, 0.6), "walk_sag": (0.03, 0.15),
    "ho_lifts": (0, 2), "ho_drift": (0.01, 0.05), "ho_sep": (0.01, 0.05), "ho_yaw": (1, 5), "ho_dip": (0.005, 0.03), "ho_jolt": (0.1, 0.6),
    "ho_jump": (3, 20), "ho_wait": (0.5, 3.0),
    "rung_e": (0.1, 0.5),
    "impact": (1.5, 3.5), "smooth": (0.03, 0.10), "smooth_v1": (0.06, 0.15), "pivot": (0.05, 0.5), "slip": (0.02, 0.15), "steady": (0.03, 0.15),
    "cot": (0.4, 1.2), "stand_tau2": (1500, 6000),
    "push_dist": (0.05, 0.5), "push_tilt": (3, 20), "push_rec": (1, 4), "push_steps": (2, 6), "push_foot": (2.0, 6.0),   # steps: operator "max 2-3 steps"
    "hm_planar": (0.02, 0.10), "hm_head": (1, 5), "hm_sw": (0.02, 0.10), "hm_prog": (0.25, 0.05), "hm_turn": (0.9, 0.3),
    # 2026-11-01 (operator: counter-sway, swinging arms, chaotic recoveries; energy = actuation over ALL phases): first guesses -> calibrate
    "arm_sway": (0.3, 3.0), "push_counter": (1.0, 8.0), "push_sway": (0.15, 0.6), "push_sep": (2.0, 12.0), "push_arm": (3.0, 25.0),
    "push_energy": (200.0, 1000.0), "en_all_tau2": (3000.0, 9000.0), "en_all_power": (100.0, 350.0),
    # 2026-11-03 (Exam #2 v3): the 60 s stand's separation / stagger departure from its first second (operator: "how they start is how
    # they finish"), the arms' departure from the walk stance, arm self-contact seconds -- first guesses, prov until calibrated
    "stand_sep_dev_max": (0.005, 0.05), "stand_sep_dev_end": (0.005, 0.04), "stand_stag_dev_max": (0.005, 0.06), "stand_stag_dev_end": (0.005, 0.05),
    "arm_dev_mean": (2.0, 15.0), "arm_dev_max": (5.0, 30.0), "sc_dur": (0.0, 2.0),
}
# PROVISIONAL keys = the spec's first guesses (no physical anchor): the calibration sets them from the population (p10 -> p90);
# every other key is a definition the calibration keeps (and only flags when dead: > 80 % of the samples at one end)
PROV = {"stand_rate", "walk_rate", "ho_jolt", "ho_jump", "impact", "smooth", "smooth_v1", "slip", "steady", "cot", "stand_tau2",
        "push_dist", "push_tilt", "push_rec", "push_foot",
        "arm_sway", "push_counter", "push_sway", "push_sep", "push_arm", "push_energy", "en_all_tau2", "en_all_power",
        "stand_sep_dev_max", "stand_sep_dev_end", "stand_stag_dev_max", "stand_stag_dev_end", "arm_dev_mean", "arm_dev_max", "sc_dur"}
# one push = 1.0: upright and no counter-sway first, few coordinated steps, still arms (operator 2026-11-01)
PUSH_W = {"tilt": 0.15, "counter": 0.15, "sway": 0.10, "steps": 0.15, "sep": 0.10, "rec": 0.10, "dist": 0.10, "foot": 0.05, "arm": 0.10}
HO_W = {"lifts": 0.25, "drift": 0.2, "sep": 0.15, "yaw": 0.1, "dip": 0.1, "jolt": 0.1, "jump": 0.05, "wait": 0.05}   # one switch = 1.0
CAL_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results", "exam2_calibration.json")
# v4 (2026-11-09): its OWN file, so the v2 calibration that scores the v2 / v3 records stays intact; a complete table (the keys v4 did not
# re-calibrate are copied from the v2 file); absent -> the v2 file (load_cal_v4)
CAL_FILE_V4 = os.path.join(os.path.dirname(CAL_FILE), "exam2_calibration_v4.json")


def load_cal_v4():
    """(table, version) for score_v4: results/exam2_calibration_v4.json, else the v2 calibration"""
    return load_cal(CAL_FILE_V4) if os.path.exists(CAL_FILE_V4) else load_cal(CAL_FILE)


def load_cal(path=CAL_FILE):
    """(table, version): the defaults overridden by the calibration file when present"""
    cal = dict(CAL_DEFAULT); ver = "default"
    try:
        d = json.load(open(path))
        for k, v in (d.get("cal") or {}).items():
            if k in cal: cal[k] = (float(v[0]), float(v[1]))
        ver = d.get("version", "?")
    except (OSError, ValueError):
        pass
    return cal, ver


def raw_values(handover, ladders, pushes, homing, e_floor=E_FLOOR):
    """every quantity the Exam #2 score ramps, before any ramp: per-policy scalars + per-item lists (stops, switches, rungs, pushes,
    homing trials). score_v2 ramps these; tools/exam_calibrate.py reads their distribution over many policies.
    e_floor = the rung error's denominator floor (E_FLOOR v2 / v3, E_FLOOR_V4 v4)."""
    V = {"falls": []}
    for r in handover or []:
        if r.get("fell") or r.get("result") == "FELL": V["falls"].append(("handover", r.get("pose")))
    for f in (ladders or {}).get("falls") or []: V["falls"].append(("ladders", f.get("seg")))
    for r in homing or []:
        if r.get("result") == "FAIL_fell": V["falls"].append(("homing", r.get("trial")))
    stand, rungs, stops = ladder_parts(ladders)
    sd = stand or {}
    V["stand_drift"] = sd.get("drift_xy")
    V["stand_yaw"] = abs(sd["yaw_change_deg"]) if sd.get("yaw_change_deg") is not None else None
    V["stand_lifts"] = sum(sd["lifts"]) if sd.get("lifts") else None
    V["stand_rate"] = sd.get("rate_rms")
    V["stand_sep_trend"] = abs(sd["sep_trend"]) if sd.get("sep_trend") is not None else None
    V["stand_stagger"] = abs(sd["stagger_mean"]) if sd.get("stagger_mean") is not None else None
    V["stand_toe"] = mean([abs(sd[k]) for k in ("toe_L", "toe_R") if sd.get(k) is not None])
    V["stand_jitter"] = sd.get("jitter_knee")
    V["stand_tau2"] = (sd.get("energy") or {}).get("tau2_mean")
    for k in ("sep_dev_max", "sep_dev_end", "stag_dev_max", "stag_dev_end"):   # v3 (2026-11-03): the 60 s stand's departure from its start
        V["stand_" + k] = sd.get(k)
    V["stand_s"] = (ladders or {}).get("stand_s") or (round(sd["dur"]) if sd.get("dur") else 20)
    # the stand AFTER walking (2026-11-03 evening): "walk_stand" at the end of the ladders, the same numbers as the first stand
    ws = next((s for s in (ladders or {}).get("segments", []) if s.get("name") == "walk_stand"), None) or {}
    V["wstand_s"] = (ladders or {}).get("stand1_s") or (round(ws["dur"]) if ws.get("dur") else 120); V["wstand_ran"] = bool(ws) and not ws.get("fell")
    V["wstand_drift"] = ws.get("drift_xy"); V["wstand_yaw"] = abs(ws["yaw_change_deg"]) if ws.get("yaw_change_deg") is not None else None
    V["wstand_lifts"] = sum(ws["lifts"]) if ws.get("lifts") else None; V["wstand_rate"] = ws.get("rate_rms")
    V["wstand_sep_trend"] = abs(ws["sep_trend"]) if ws.get("sep_trend") is not None else None
    V["wstand_stagger"] = abs(ws["stagger_mean"]) if ws.get("stagger_mean") is not None else None
    V["wstand_toe"] = mean([abs(ws[k]) for k in ("toe_L", "toe_R") if ws.get(k) is not None]); V["wstand_jitter"] = ws.get("jitter_knee")
    for k in ("sep_dev_max", "sep_dev_end", "stag_dev_max", "stag_dev_end"): V["wstand_" + k] = ws.get(k)
    V["stand_sep_dir"], V["wstand_sep_dir"] = sd.get("sep_dir"), ws.get("sep_dir")
    V["stops"] = []
    for s, prev, ax in stops:
        st_ = s.get("stop") or {}
        V["stops"].append({"ok": bool(st_) and not s.get("fell"), "ax": ax, "settle": st_.get("settle_s"), "steps": st_.get("steps"),
                           "resid": st_.get("resid_m"), "over_m": st_.get("overshoot_m"), "over_deg": st_.get("overshoot_deg")})
    walk = [r for r in rungs if _axis(r["cmd"]) in ("vx", "vy") and max(abs(r["cmd"][0]), abs(r["cmd"][1])) <= 0.5 + 1e-9 and not r.get("fell")]
    V["walk_tilt"] = mean([r["tilt_mean"] for r in walk if r.get("tilt_mean") is not None])
    V["walk_rate"] = mean([r["rate_rms"] for r in walk if r.get("rate_rms") is not None])
    z_ref = mean([s["z_mean"] for r in handover or [] for s in r.get("segments", []) if s.get("name") == "bal_stand" and s.get("z_mean")]) or sd.get("z_mean")
    zw = mean([r["z_mean"] for r in walk if r.get("z_mean")])
    V["walk_sag"] = (z_ref - zw) if (z_ref and zw) else None
    V["switches"] = []
    for r in handover or []:
        segs = {s["name"]: s for s in r.get("segments", [])}
        for before, after, wkey in (("bal_stand", "walk_after", "to_walk"), ("walk_after", "bal_after", "to_balance")):
            a, b0 = segs.get(after), segs.get(before)
            if not a or not b0 or a.get("fell"):
                V["switches"].append({"ok": False}); continue
            V["switches"].append({"ok": True, "lifts": sum(a.get("lifts") or [9, 9]), "drift": a.get("drift_xy"),
                                  "sep": abs(a["sep_mean"] - b0["sep_mean"]) if a.get("sep_mean") is not None and b0.get("sep_mean") is not None else None,
                                  "yaw": abs(a["yaw_change_deg"]) if a.get("yaw_change_deg") is not None else None,
                                  "dip": (b0["z_mean"] - a["z_min"]) if b0.get("z_mean") and a.get("z_min") else None,
                                  "jolt": a.get("rate_peak2"), "jump": a.get("qt_jump_deg"), "wait": (r.get("switch_wait_s") or {}).get(wkey)})
    V["rungs"] = [(_axis(r["cmd"]), None if r.get("fell") or not r.get("v_true") else rung_error(r["cmd"], r["v_true"], e_floor)) for r in rungs]
    ok_r = [r for r in rungs if not r.get("fell")]
    lin = [r for r in ok_r if _axis(r["cmd"]) in ("vx", "vy")]
    V["impact"] = mean([r["impact_p95_N"] / W_BODY for r in ok_r if r.get("impact_p95_N")])
    acc = [r["qt_acc_rms"] for r in ok_r if r.get("qt_acc_rms") is not None]
    V["smooth_kind"] = "acc" if acc else "rate"
    V["smooth"] = mean(acc) if acc else mean([r["qt_rate_rms"] for r in ok_r if r.get("qt_rate_rms") is not None])
    V["pivot"] = mean([r["pivot_rad_s"] for r in lin if r.get("pivot_rad_s") is not None])
    V["slip"] = mean([r["slip_mean"] for r in lin if r.get("slip_mean") is not None])
    fast = [r for r in lin if max(abs(r["cmd"][0]), abs(r["cmd"][1])) >= 0.2 - 1e-9 and r.get("swing_height")]
    V["swing_share"] = mean([1.0 if 0.03 <= max(r["swing_height"]) <= 0.10 else 0.0 for r in fast])
    V["steady"] = mean([(r["v_true_std"][0] ** 2 + r["v_true_std"][1] ** 2) ** 0.5 for r in lin if r.get("v_true_std")])
    V["cot"] = mean([r["energy"]["P_W"] / (W_BODY * r["v_true"][0]) for r in ok_r if r["name"] in ("vx+0.30", "vx+0.50") and r.get("energy")
                     and r.get("v_true") and r["v_true"][0] > 0.05])
    V["pushes"] = []
    for s in (pushes or {}).get("segments", []):
        p = s.get("push")
        if not p: continue
        V["pushes"].append({"held": bool(p.get("held")), "dist": p.get("dist_m"), "tilt": p.get("tilt_peak_deg"),
                            "rec": p.get("recovery_s") if p.get("recovery_s") is not None else 5.0, "steps": max(p.get("steps") or 0, 0), "foot": p.get("foot_peak_bw"),
                            "counter": p.get("counter_sway_deg"), "sway": p.get("sway_rate_rms"), "sep": p.get("sep_dev_cm"), "arm": p.get("arm_peak_deg"),
                            "energy": p.get("energy_J")})
    # arm sway (2026-11-01): the arm joints' angle std in the 20 s stand, the stops and the walking rungs up to 0.5 m/s
    arm_segs = ([stand] if stand else []) + [s for s, _, _ in stops] + walk
    V["arm_sway"] = mean([s["arm_std_deg"] for s in arm_segs if s and s.get("arm_std_deg") is not None])
    # v3 (2026-11-03): the arms' departure from the walk stance (max over the 7 joints of |q - default|, deg): mean / peak over the walk
    # policy's windows (the 60 s stand, the stops, the walking rungs <= 0.5, the pushes); arm self-contact seconds over the same + the
    # handover's walk segment; joint-limit dwell (share of samples within 2 deg of a limit) per joint, walking vs standing
    walk_segs = [s for s in arm_segs if s] + [s for s in (pushes or {}).get("segments", []) if s.get("name", "").startswith("push:")]
    ad = [s["arm_dev_deg"] for s in walk_segs if s.get("arm_dev_deg")]
    V["arm_dev_mean"] = mean([max(a["mean"]) for a in ad]); V["arm_dev_max"] = max([max(a["max"]) for a in ad] or [None])
    sc_segs = walk_segs + [s for r in handover or [] for s in r.get("segments", []) if s.get("name") == "walk_after"]
    scs = [s["self_contact"] for s in sc_segs if s.get("self_contact")]
    V["sc_dur"] = sum(x["dur_s"] or 0.0 for x in scs) if scs else None
    V["sc_peak"] = max([x["peak_N"] for x in scs] or [None]); V["sc_pair"] = next((x["pair"] for x in sorted(scs, key=lambda x: -x["peak_N"]) if x["pair"]), None)
    def jl_agg(segs):
        acc, tot = None, 0.0
        for s_ in segs:
            j = s_.get("joint_limit"); d = s_.get("dur") or 0.0
            if not j or not d: continue
            if acc is None: acc = [0.0] * len(j["frac"])
            for k, v in enumerate(j["frac"]): acc[k] += v * d
            tot += d
        if not acc or not tot: return None
        fr = [v / tot for v in acc]
        return {"frac": [round(v, 4) for v in fr], "worst": sorted([[JOINT_NAMES[k] if k < len(JOINT_NAMES) else f"j{k}", round(v, 4)] for k, v in enumerate(fr) if v > 0],
                                                                    key=lambda x: -x[1])[:3]}
    V["jl_walk"] = jl_agg([r for r in rungs if not r.get("fell")] + [s for s, _, _ in stops]); V["jl_stand"] = jl_agg([stand] if stand else [])
    # energy over ALL phases (operator 2026-11-01: "how much the motors get actuated in sum ... for all test phases"): Tests 0-2
    E = [s.get("energy") for r in handover or [] for s in r.get("segments", [])] + [s.get("energy") for s in (ladders or {}).get("segments", [])] \
        + [s.get("energy") for s in (pushes or {}).get("segments", [])]
    E = [e for e in E if e and e.get("P_W") is not None and e.get("mech_J") is not None and e["P_W"] > 0]
    dur = sum(e["mech_J"] / e["P_W"] for e in E)
    V["en_all_power"] = sum(e["mech_J"] for e in E) / dur if dur > 0 else None
    V["en_all_tau2"] = sum(e["tau2"] for e in E) / dur if dur > 0 else None
    V["homing"] = []
    for r in homing or []:
        if r.get("result") != "ARRIVED":
            V["homing"].append({"ok": False}); continue
        e, sw_ = r.get("final_err") or {}, r.get("err_at_switch") or {}
        V["homing"].append({"ok": True, "planar": e.get("planar_m"), "head": abs(e["heading_deg"]) if e.get("heading_deg") is not None else None,
                            "sw": sw_.get("planar_m"), "prog": (r.get("straight_m") or 0.0) / r["t_switch"] if r.get("t_switch") else None,
                            "turn": r.get("turn_gain"), "rescue": r.get("n_rescue") or 0})
    return V


# ── key columns (2026-11-02, the robot's toe scuff): the ladder swing heights at 0.15 / 0.30 and Test 5 "stumble" -- NOT in the score;
# EXAM2_SPEC Test 5 carries the proposed weight and ramp
def ladder_swing(ladders, names=("vx+0.15", "vx+0.30")):
    """{rung name: [L, R] swing height (m) from the flat ladder, None where the rung is missing / fell}"""
    segs = {s["name"]: s for s in (ladders or {}).get("segments", []) if s.get("name") != "reset"}
    return {n: (segs[n].get("swing_height") if segs.get(n) and not segs[n].get("fell") else None) for n in names}


def seg_swing(seg):
    """the swing-engine dict of a segment: 'swing' (ladders + Test 5 since 2026-11-02 evening) or 'stumble' (the first Test 5 records)"""
    return (seg or {}).get("swing") or (seg or {}).get("stumble")


def ladder_scuffs(ladders, names=("vx+0.15", "vx+0.30")):
    """the toe-drag key column (2026-11-02): {rung: {scuffed, swings [L, R], catches, trips, share}} from the FLAT ladder's swing engine;
    None where the rung is missing / fell / predates the engine (records before 2026-11-02 evening)"""
    segs = {s["name"]: s for s in (ladders or {}).get("segments", []) if s.get("name") != "reset"}
    out = {}
    for n in names:
        q = seg_swing(segs.get(n)) if segs.get(n) and not segs[n].get("fell") else None
        if not q or q.get("swings") is None:
            out[n] = None; continue
        ns = sum(q["swings"])
        out[n] = {"scuffed": q.get("scuffed", 0), "swings": q["swings"], "catches": q.get("catches", 0), "trips": q.get("trips", 0),
                  "share": (q.get("scuffed", 0) / ns) if ns else None, "apex_mean": q.get("swing_apex_mean"), "apex_min": q.get("swing_apex_min")}
    return out


def scuff_cell(sc):
    """'scuffed/swings' for the table ('5/33'); '—' before the engine"""
    return "—" if not sc else f"{sc['scuffed']}/{sum(sc['swings'])}"


def stumble_summary(stumble):
    """the Test 5 headline numbers per rung (None when the record is missing): {rung: {apex [L,R] m, apex_min, trips, scuffed, catches,
    swings, tilt_max, fell, on_field}} + falls, field (seed, checksum)"""
    if not stumble:
        return None
    out = {"falls": len(stumble.get("falls") or []), "field": stumble.get("field"), "result": stumble.get("result"), "rungs": {}}
    for s in stumble.get("segments", []):
        q = seg_swing(s)
        if not q or not s.get("name", "").startswith("vx"):
            continue
        out["rungs"][s["name"]] = {"apex": q.get("swing_apex"), "apex_min": q.get("swing_apex_min"), "apex_mean": q.get("swing_apex_mean"),
                                   "trips": q.get("trips"), "scuffed": q.get("scuffed"), "catches": q.get("catches"), "swings": q.get("swings"),
                                   "tilt_max": s.get("tilt_max"), "fell": bool(s.get("fell")), "on_field": q.get("on_field_frac"),
                                   "ground_rms": q.get("ground_rms"), "drift_lat": (s.get("dx_body") or [None, None])[1], "yaw": s.get("yaw_change_deg")}
    return out


# ═══ EXAM #2 v3 (2026-11-03): HARD FAILS (binary, no calibration) + the measured-only rows ═══
# a. height: z_stand = the walk policy's mean base height over the clean 60 s stand; walking (ladder rungs + stops, the handover's walk
#    segment, the homing trials) fails when the 1 s running mean of z drops below z_stand - Z_RUN_DROP or any sample below z_stand -
#    Z_SAMPLE_DROP; a push recovery fails only below z_stand - Z_PUSH_DROP (a deep dip can be a good recovery); z_stand < Z_STAND_MIN fails.
#    (34 walk policies: stand z 0.988-0.994; walk drop median 6.7 / p90 9.6 / max 18.5 cm; push drop median 15.3 / p90 19.7 / max 30.1 cm.)
#    The floor is a fail only -- no bonus for straighter legs (operator).
# b. tilt: training ends an episode at 0.8 rad = 45.8 deg -> tilt >= TILT_FAIL_DEG (env EXAM_TILT_FAIL_DEG) in any window = a fall; the
#    peak tilt per test is printed so the operator can lower it.
# c. contacts: a non-foot body on the floor > CONTACT_FAIL_N = a fall (the commander already resets the robot on it and records the body
#    + force; older records are read through their "crawl" falls). Arm self-contact (arm vs torso / pelvis / legs / the other arm) is
#    MEASURED: seconds above 20 N (sc_dur, prov ramp) + the peak and the pair.
# d. joint limits: MEASURED only (our policies hold ankle-roll / hip-yaw limits on purpose): dwell share per joint, walking and standing.
# e. arms: mean / max departure from the walk stance (prov ramps) + a FAIL when a wrist stays above its shoulder or across the body
#    midline for more than ARM_FAIL_S in a walk-policy window.
# A hard fail = that window counts as a fall for scoring (the rung / push / trial / switch scores 0) and ONE entry per (test, criterion)
# joins meta.score.falls as [test, "criterion: detail"] -- the record keeps the raw numbers.
TILT_FAIL_DEG = float(os.environ.get("EXAM_TILT_FAIL_DEG", "45.8"))
Z_RUN_DROP, Z_SAMPLE_DROP, Z_PUSH_DROP, Z_STAND_MIN = 0.10, 0.25, 0.40, 0.90
CONTACT_FAIL_N, ARM_FAIL_S = 20.0, 0.5
JOINT_NAMES = ["L_hip_yaw", "L_hip_pitch", "L_hip_roll", "L_knee", "L_ankle_pitch", "L_ankle_roll", "R_hip_yaw", "R_hip_pitch", "R_hip_roll", "R_knee",
               "R_ankle_pitch", "R_ankle_roll", "torso", "L_sh_pitch", "L_sh_roll", "L_sh_yaw", "L_elbow", "L_wr_roll", "L_wr_pitch", "L_wr_yaw",
               "R_sh_pitch", "R_sh_roll", "R_sh_yaw", "R_elbow", "R_wr_roll", "R_wr_pitch", "R_wr_yaw"]


def hard_fails(handover, ladders, pushes, homing, tilt_fail_deg=None):
    """Evaluate the v3 hard fails on DEEP COPIES of the records: the offending windows are marked (a rung / handover segment gets
    fell = "hard:<crit>", a push held = False, a homing trial result = "FAIL_hard") and returned with the fail list and the info
    (z_stand, peak tilt per test, the largest height drops). The caller scores the marked copies."""
    import copy
    tf = TILT_FAIL_DEG if tilt_fail_deg is None else tilt_fail_deg
    H, L, P, M = copy.deepcopy(handover or []), copy.deepcopy(ladders), copy.deepcopy(pushes), copy.deepcopy(homing or [])
    stand, rungs, stops = ladder_parts(L)
    z_stand = (stand or {}).get("z_mean")
    info = {"z_stand": z_stand, "tilt_fail_deg": tf, "tilt_peak": {}, "z_drop_walk": None, "z_drop_push": None, "z_drop_sample": None, "details": []}
    F = []   # (test, crit, seg, why)

    def tilt(test, seg_name, tmax):
        if tmax is None: return False
        info["tilt_peak"][test] = max(info["tilt_peak"].get(test) or 0.0, tmax)
        if tmax >= tf:
            F.append((test, "tilt", seg_name, f"{seg_name} tilt {tmax:.1f} >= {tf:g} deg")); return True
        return False

    def height(test, seg_name, z_run_min, z_min, push=False):
        if z_stand is None: return False
        bad = None
        if push:
            if z_min is not None and z_min < z_stand - Z_PUSH_DROP: bad = f"{seg_name} z {z_min:.3f} < {z_stand - Z_PUSH_DROP:.3f} (stand {z_stand:.3f} - {Z_PUSH_DROP})"
            if z_min is not None: info["z_drop_push"] = max(info["z_drop_push"] or 0.0, z_stand - z_min)
        else:
            if z_run_min is not None and z_run_min < z_stand - Z_RUN_DROP: bad = f"{seg_name} 1 s mean z {z_run_min:.3f} < {z_stand - Z_RUN_DROP:.3f} (stand {z_stand:.3f} - {Z_RUN_DROP})"
            elif z_min is not None and z_min < z_stand - Z_SAMPLE_DROP: bad = f"{seg_name} z {z_min:.3f} < {z_stand - Z_SAMPLE_DROP:.3f} (stand {z_stand:.3f} - {Z_SAMPLE_DROP})"
            if z_run_min is not None: info["z_drop_walk"] = max(info["z_drop_walk"] or 0.0, z_stand - z_run_min)
            if z_min is not None: info["z_drop_sample"] = max(info["z_drop_sample"] or 0.0, z_stand - z_min)
        if bad: F.append((test, "height", seg_name, bad)); return True
        return False

    def contact(test, seg_name, fc):
        if fc and (fc.get("peak_N") or 0.0) > CONTACT_FAIL_N:
            F.append((test, "contact", seg_name, f"{seg_name} {fc.get('body_peak') or fc.get('body')} on the floor {fc['peak_N']:.0f} N at {fc.get('t')} s")); return True
        return False

    def arms(test, seg_name, seg):
        up, cr = seg.get("arm_up_s") or [0, 0], seg.get("arm_cross_s") or [0, 0]
        if max(up) > ARM_FAIL_S:
            F.append((test, "arms", seg_name, f"{seg_name} wrist above the shoulder {max(up):.2f} s ({'L' if up[0] >= up[1] else 'R'})")); return True
        if max(cr) > ARM_FAIL_S:
            F.append((test, "arms", seg_name, f"{seg_name} wrist across the midline {max(cr):.2f} s ({'L' if cr[0] >= cr[1] else 'R'})")); return True
        return False

    if z_stand is not None and z_stand < Z_STAND_MIN:
        F.append(("ladders", "height", "stand", f"stand z {z_stand:.3f} < {Z_STAND_MIN}"))
        if stand: stand["fell"] = stand.get("fell") or "hard:height"
    if stand:
        for fn in (lambda: tilt("ladders", "stand", stand.get("tilt_max")), lambda: contact("ladders", "stand", stand.get("floor_contact")), lambda: arms("ladders", "stand", stand)):
            if fn(): stand["fell"] = stand.get("fell") or "hard"
    extra = [s for s in (L or {}).get("segments", []) if s.get("name") in ("walk_pre", "walk_stand")]   # the stand after walking + its walk
    for sg in rungs + [s for s, _, _ in stops] + extra:
        if sg.get("fell"): continue
        n = sg["name"]
        if any([height("ladders", n, sg.get("z_run_min"), sg.get("z_min")), tilt("ladders", n, sg.get("tilt_max")), contact("ladders", n, sg.get("floor_contact")), arms("ladders", n, sg)]):
            sg["fell"] = "hard"
    for r in H:
        for sg in r.get("segments", []):
            n = f"{r.get('pose')}/{sg['name']}"
            hit = tilt("handover", n, sg.get("tilt_max"))
            if sg["name"] == "walk_after" and not sg.get("fell"):
                hit = any([hit, height("handover", n, sg.get("z_run_min"), sg.get("z_min")), contact("handover", n, sg.get("floor_contact")), arms("handover", n, sg)])
            if hit and not sg.get("fell"): sg["fell"] = "hard"
    for sg in (P or {}).get("segments", []):
        pz = sg.get("push")
        if not pz or not pz.get("held"): continue
        n = sg["name"]
        if any([height("pushes", n, None, sg.get("z_min"), push=True), tilt("pushes", n, sg.get("tilt_max")), contact("pushes", n, sg.get("floor_contact")), arms("pushes", n, sg)]):
            pz["held"] = False; sg["fell"] = "hard"
    for r in M:
        w = r.get("watch") or {}
        n = f"trial {r.get('trial')}"
        if r.get("result") != "ARRIVED": continue
        if any([height("homing", n, w.get("z_run_min"), None), tilt("homing", n, w.get("tilt_peak")), contact("homing", n, w.get("floor_contact")), arms("homing", n, w)]):
            r["result"] = "FAIL_hard"
    info["details"] = [{"test": t, "crit": c, "seg": sg, "why": w} for t, c, sg, w in F]
    # one falls entry per (test, criterion): the count and the first offender
    groups = {}
    for t, c, sg, w in F:
        groups.setdefault((t, c), []).append(w)
    falls = [[t, f"{c}: {len(ws)} window{'s' if len(ws) > 1 else ''} ({ws[0]})"] for (t, c), ws in groups.items()]
    return H, L, P, M, falls, info


def score_v3(handover, ladders, pushes, homing, n_homing=10, cal=None, tilt_fail_deg=None, _v4=False, _out=None):
    """Exam #2 v3 (2026-11-03): score_v2's table with the hard fails applied first (the offending windows score as falls), the 60 s
    stand's separation / stagger departure, the arms' stance departure, arm self-contact seconds and the measure-only rows (joint-limit
    dwell, height drops, peak tilt). Same keys as v2 for fleetdeck (+ "v3" with the hard-fail info); version 3.
    _v4: the v4 table (score_v4: e floor 0.05, 48 pushes) -- call score_v4, not this flag."""
    H, L, P, M, hard, info = hard_fails(handover, ladders, pushes, homing, tilt_fail_deg)
    if _out is not None:   # score_v4 reads the hard-fail-marked copies (the constraints count a hard-failed rung as not walked)
        _out.update({"L": L, "P": P})
    sc = score_v2(H, L, P, M, n_homing=n_homing, cal=cal, _v3=info, _v4=_v4)
    sc["version"] = 3
    # survival: every hard-fail group outside the pushes halves it too (the pushes' own score already lost those pushes)
    nh = sum(1 for t, _ in hard if t != "pushes")
    for m in sc["metrics"]:
        if m["key"] == "falls":
            ran = bool(L and L.get("segments"))
            n_all = m["value"] + nh
            m["value"] = n_all; m["s"] = round((0.5 ** n_all) if ran else 0.0, 3); m["pts"] = round(m["w"] * m["s"], 3)
            m["label"] = "falls (Tests 0, 1, 3) incl. hard fails"
    sc["falls"] = sc["falls"] + hard
    sc["parts"] = {q: round(sum(m["pts"] for m in sc["metrics"] if m["quality"] == q), 1) for q in QUALITIES}
    sc["tests"] = {}
    for m in sc["metrics"]:
        sc["tests"][m["test"]] = round(sc["tests"].get(m["test"], 0.0) + m["pts"], 1)
    sc["score"] = round(sum(sc["parts"].values()), 1)
    sc["v3"] = {"hard": hard, "info": info, "thresholds": {"z_run_drop": Z_RUN_DROP, "z_sample_drop": Z_SAMPLE_DROP, "z_push_drop": Z_PUSH_DROP,
                                                           "z_stand_min": Z_STAND_MIN, "tilt_fail_deg": info["tilt_fail_deg"], "contact_N": CONTACT_FAIL_N, "arm_fail_s": ARM_FAIL_S}}
    return sc


def score_v2(handover, ladders, pushes, homing, n_homing=10, cal=None, _v3=None, _v4=False):
    """handover: list of Test 0 records; ladders / pushes: the Test 1 / Test 2 records (or None); homing: the Test 3 trial records.
    cal: a (g, b) table (default: load_cal() = CAL_DEFAULT + results/exam2_calibration.json)"""
    C, cver = (cal, "given") if cal is not None else load_cal()
    R = lambda x, k: ramp(x, *C[k])
    e_floor = E_FLOOR_V4 if _v4 else E_FLOOR; n_push = EXPECTED_PUSHES_V4 if _v4 else EXPECTED_PUSHES   # v4 (2026-11-09)
    V = raw_values(handover, ladders, pushes, homing, e_floor=e_floor)
    M = []

    def add(q, test, key, label, x, unit, w, prov=False, s=None, ck=None):
        g, b = C[ck or key] if (ck or key) in C else (1, 0)
        s = ramp(x, g, b) if s is None else clamp01(s)
        M.append({"quality": q, "test": test, "key": key, "label": label, "value": None if x is None else round(float(x), 4), "unit": unit,
                  "g": g, "b": b, "w": w, "s": round(s, 3), "pts": round(w * s, 3), "prov": prov and cver == "default"})

    falls = V["falls"]
    # ── survival 10: halves with every fall (Tests 0, 1, 3; a push-test fall belongs to the push score)
    ran = bool(ladders and ladders.get("segments"))   # a Test 1 that never ran (no record, or FAIL_switch_start) = the exam did not run
    add("survival", "all", "falls", "falls (Tests 0, 1, 3)", len(falls), "", 10, s=(0.5 ** len(falls)) if ran else 0.0, ck="none")
    # ── stability 30 = 20 s stand 7 + stops 5 + walking torso 6 + handover 12
    # stability 30 (2026-11-01) = stand 6 + stops 4 + walking 5 + handover 12 + arms locked 3
    # v3 (2026-11-03 evening): TWO stands share the stand budget -- "stand" (the 60 s clean-engagement stand, stand_* rows) and "walk_stand"
    # (the 120 s stand AFTER walking at 0.30 for 10 s, wstand_* rows, the stand the operator sees in his sim2sim); each row at half the
    # v2 weight, the twins use the same g / b. The BASE drift rows are labelled so, the feet rows say "feet separation".
    h = 0.5 if _v3 is not None else 1.0
    for pre, lab in ((("stand", f"{V['stand_s']:g} s stand"), ("wstand", f"{V['wstand_s']:g} s stand after walking")) if _v3 is not None else (("stand", f"{V['stand_s']:g} s stand"),)):
        ck = lambda k: "stand_" + k   # the twins share the stand's calibration keys
        add("stability", "ladders", f"{pre}_drift", f"{lab}: BASE drift", V[f"{pre}_drift"], "m", 1.25 * h, ck=ck("drift"))
        add("stability", "ladders", f"{pre}_yaw", f"{lab}: heading change", V[f"{pre}_yaw"], "deg", 0.75 * h, ck=ck("yaw"))
        add("stability", "ladders", f"{pre}_lifts", f"{lab}: foot lifts", V[f"{pre}_lifts"], "", 1.25 * h, ck=ck("lifts"))
        add("stability", "ladders", f"{pre}_rate", f"{lab}: torso roll/pitch rate rms", V[f"{pre}_rate"], "rad/s", 1.0 * h, prov=True, ck=ck("rate"))
        add("stability", "ladders", f"{pre}_sep_trend", f"{lab}: feet separation trend (last 1 s - first 1 s; direction in the record's sep_dir)", V[f"{pre}_sep_trend"], "m", 0.5 * h, ck=ck("sep_trend"))
        add("stability", "ladders", f"{pre}_stagger", f"{lab}: feet fore-aft stagger", V[f"{pre}_stagger"], "m", 0.5 * h, ck=ck("stagger"))
        add("stability", "ladders", f"{pre}_toe", f"{lab}: toe angle (mean |L|, |R|)", V[f"{pre}_toe"], "deg", 0.25 * h, ck=ck("toe"))
        add("stability", "ladders", f"{pre}_jitter", f"{lab}: knee jitter", V[f"{pre}_jitter"], "deg/sample", 0.5 * h, ck=ck("jitter"))
        if _v3 is not None:   # v3 rows: the feet's departure from the stand's first second (stability 30 keeps its sum: handover 12 -> 10, arms locked 3 -> 2)
            add("stability", "ladders", f"{pre}_sep_dev_max", f"{lab}: feet separation deviation, max |sep - sep(0)|", V[f"{pre}_sep_dev_max"], "m", 1.0 * h, prov=True, ck=ck("sep_dev_max"))
            add("stability", "ladders", f"{pre}_sep_dev_end", f"{lab}: feet separation deviation, |sep(end) - sep(0)|", V[f"{pre}_sep_dev_end"], "m", 0.5 * h, prov=True, ck=ck("sep_dev_end"))
            add("stability", "ladders", f"{pre}_stag_dev_max", f"{lab}: feet stagger deviation, max |stagger - stagger(0)|", V[f"{pre}_stag_dev_max"], "m", 0.5 * h, prov=True, ck=ck("stag_dev_max"))
            add("stability", "ladders", f"{pre}_stag_dev_end", f"{lab}: feet stagger deviation, |stagger(end) - stagger(0)|", V[f"{pre}_stag_dev_end"], "m", 0.25 * h, prov=True, ck=ck("stag_dev_end"))
    if _v3 is not None:
        add("stability", "all", "arm_dev_mean", "arms at the walk stance: mean max-joint |q - default| (stand, stops, walking, pushes)", V["arm_dev_mean"], "deg", 0.5, prov=True)
        add("stability", "all", "arm_dev_max", "arms at the walk stance: peak |q - default|", V["arm_dev_max"], "deg", 0.25, prov=True)
    ss = [mean([R(x["settle"], "stop_settle"), R(x["steps"], "stop_steps"), R(x["resid"], "stop_resid"),
                R(x["over_deg"], "stop_over_deg") if x["ax"] == "wz" else R(x["over_m"], "stop_over_m")]) if x["ok"] else 0.0 for x in V["stops"]]
    sv = mean(ss + [0.0] * max(0, EXPECTED_STOPS - len(ss)), 0.0)
    add("stability", "ladders", "stops", f"stops from the ladder tops ({len(ss)}/{EXPECTED_STOPS}): settle time, steps, residual drift, overshoot",
        sv, "score", 4.0, s=sv, ck="none")
    add("stability", "ladders", "walk_tilt", "walking (|v| <= 0.5): torso tilt", V["walk_tilt"], "deg", 2.0)
    add("stability", "ladders", "walk_rate", "walking (|v| <= 0.5): torso roll/pitch rate rms", V["walk_rate"], "rad/s", 2.0, prov=True)
    add("stability", "ladders", "walk_sag", "walking: height sag vs the balance policy's stand", V["walk_sag"], "m", 1.0)
    sw = [sum(w * R(x[k], "ho_" + k) for k, w in HO_W.items()) if x["ok"] else 0.0 for x in V["switches"]]
    hs = mean(sw + [0.0] * max(0, EXPECTED_SWITCHES - len(sw)), 0.0)
    add("stability", "ladders", "arm_sway", "arms locked: arm joint angle std (stand, stops, walking <= 0.5)", V["arm_sway"], "deg", 3.0 if _v3 is None else 2.0, prov=True)
    add("stability", "handover", "handover", f"handover switches ({len(sw)}/{EXPECTED_SWITCHES}): steps, drift, inward draw, turn, dip, jolt, target jump, wait",
        hs, "score", 12.0 if _v3 is None else 10.0, s=hs, ck="none")
    # ── tracking 20 = vx 8 + vy 4 + wz 8: per rung ramp(e; rung_e); a rung not run (fall) = 0
    for ax, w in (("vx", 8.0), ("vy", 4.0), ("wz", 8.0)):
        sc = [0.0 if e is None else R(e, "rung_e") for a, e in V["rungs"] if a == ax]
        s = sum(sc) / max(EXPECTED_RUNGS[ax], len(sc))
        add("tracking", "ladders", f"track_{ax}", f"{ax} ladder: mean rung score ({len(sc)}/{EXPECTED_RUNGS[ax]} rungs), e = |v - c| / max(|c|, {e_floor:g})", s, "score", w, s=s, ck="none")
    # ── gait & body care 10 = impact 3 + action smoothness 3 + sliding 2 + swing 1 + speed steadiness 1
    add("gait", "ladders", "impact", "foot touchdown impact p95 / body weight", V["impact"], "BW", 3.0, prov=True)
    w_sm = 3.0 if _v3 is None else 2.0   # v3: 1 point of the smoothness budget goes to arm self-contact
    if V["smooth_kind"] == "acc":   # the SECOND difference of the joint targets (chatter); older records carry only the first difference
        add("gait", "ladders", "smooth", "action roughness: rms of the joint-target 2nd difference (max joint)", V["smooth"], "rad", w_sm, prov=True)
    else:
        add("gait", "ladders", "smooth", "action step rms (joint targets, max joint, per policy step)", V["smooth"], "rad", w_sm, prov=True, ck="smooth_v1")
    if _v3 is not None:   # v3 measured rows: arm self-contact seconds (scored, prov), joint-limit dwell / height drops / peak tilt (weight 0 = shown only)
        pair = f" ({' vs '.join(V['sc_pair'])}, peak {V['sc_peak']:.0f} N)" if V.get("sc_pair") else ""
        add("gait", "all", "sc_dur", f"arm self-contact > 20 N: seconds (stand, stops, walking, pushes, handover walk){pair}", V["sc_dur"], "s", 1.0, prov=True)
        for key, lab in (("jl_walk", "walking"), ("jl_stand", "standing")):
            j = V.get(key); worst = (j or {}).get("worst") or []
            add("gait", "ladders", key, f"joint-limit dwell {lab} (share within 2 deg of a limit; worst: " + (", ".join(f"{n_} {f_:.2f}" for n_, f_ in worst) or "none") + ") -- measured only",
                worst[0][1] if worst else 0.0, "share", 0.0, s=1.0, ck="none")
        inf = _v3
        zs = f"stand z {inf['z_stand']:.3f} m" if inf.get("z_stand") is not None else "stand z n/a"
        add("stability", "all", "z_drop", f"height: largest 1 s-mean drop walking / sample drop in a push ({zs}; fail at -{Z_RUN_DROP:g} / -{Z_PUSH_DROP:g}) -- measured only",
            inf.get("z_drop_walk"), "m", 0.0, s=1.0, ck="none")
        add("stability", "all", "z_drop_push", "height: largest single-sample drop in a push recovery -- measured only", inf.get("z_drop_push"), "m", 0.0, s=1.0, ck="none")
        tp = inf.get("tilt_peak") or {}
        add("stability", "all", "tilt_peak", "peak torso tilt per test (" + ", ".join(f"{k} {v:.1f}" for k, v in tp.items()) + f"); fail at {inf['tilt_fail_deg']:g} deg -- measured only",
            max(tp.values()) if tp else None, "deg", 0.0, s=1.0, ck="none")
    add("gait", "ladders", "pivot", "pivot index of a loaded foot (linear rungs)", V["pivot"], "rad/s", 1.0)
    add("gait", "ladders", "slip", "slip speed of a loaded foot (linear rungs)", V["slip"], "m/s", 1.0, prov=True)
    add("gait", "ladders", "swing", "swing height inside 3-10 cm (share of rungs >= 0.2 m/s)", V["swing_share"], "share", 1.0, s=V["swing_share"] or 0.0, ck="none")
    add("gait", "ladders", "steady", "speed steadiness: planar v std within a rung", V["steady"], "m/s", 1.0, prov=True)
    # ── energy 10 = cost of transport at vx 0.3 / 0.5 (6) + standing tau^2 (4)
    # energy 10 (2026-11-01) = actuation over all phases 4 (tau^2 2 + mechanical power 2) + push recovery 2 + cost of transport 2 + stand tau^2 2
    add("energy", "all", "en_all_tau2", "all phases (Tests 0-2): mean sum tau^2 (motor load)", V["en_all_tau2"], "N^2m^2", 2.0, prov=True)
    add("energy", "all", "en_all_power", "all phases (Tests 0-2): mean mechanical power sum |tau qdot|", V["en_all_power"], "W", 2.0, prov=True)
    pe = mean([p["energy"] for p in V["pushes"] if p["held"] and p.get("energy") is not None])
    add("energy", "pushes", "push_energy", "push recovery: mechanical energy in the 5 s after a kick (held pushes)", pe, "J", 2.0, prov=True)
    add("energy", "ladders", "cot", "cost of transport at vx 0.3 / 0.5 (mechanical |tau qdot|)", V["cot"], "", 2.0, prov=True)
    add("energy", "ladders", "stand_tau2", "20 s stand: mean sum tau^2", V["stand_tau2"], "N^2m^2", 2.0, prov=True)
    # ── pushes 10: per push (24) held -> mean of 5 ramps, a fall -> 0
    ps = [sum(w * R(p[k], "push_" + k) for k, w in PUSH_W.items()) if p["held"] else 0.0 for p in V["pushes"]]
    held = sum(1 for p in V["pushes"] if p["held"])
    pv = sum(ps) / max(n_push, len(ps))
    add("pushes", "pushes", "pushes", f"pushes held {held}/{n_push}: tilt, counter-sway, sway, steps, feet, recovery, distance, foot peak, arms",
        pv, "score", 10.0, prov=True, s=pv, ck="none")
    # ── homing 10 per trial: settled 4 + heading 1 + at the switch 1 + progress 2 + turn 2 (minus 0.1 per rescue); not arrived = 0
    hv = [0.4 * R(x["planar"], "hm_planar") + 0.1 * R(x["head"], "hm_head") + 0.1 * R(x["sw"], "hm_sw") + 0.2 * R(x["prog"], "hm_prog")
          + 0.2 * max(0.0, R(x["turn"], "hm_turn") - 0.1 * x["rescue"]) if x["ok"] else 0.0 for x in V["homing"]]
    hm = sum(hv) / max(n_homing, len(hv)) if hv else 0.0
    arr = sum(1 for x in V["homing"] if x["ok"])
    add("homing", "homing", "homing", f"homing arrived {arr}/{len(V['homing'])}: settled, heading, at the switch, progress speed, turn gain - rescues", hm, "score", 10.0, s=hm, ck="none")

    parts = {q: round(sum(m["pts"] for m in M if m["quality"] == q), 1) for q in QUALITIES}
    tests = {}
    for m in M:
        tests[m["test"]] = round(tests.get(m["test"], 0.0) + m["pts"], 1)
    return {"version": 2, "calib": cver, "score": round(sum(parts.values()), 1), "parts": parts, "tests": tests, "falls": [list(f) for f in falls],
            "metrics": M, "weights": QUALITIES, "cal": {k: list(v) for k, v in C.items()}}   # the table used (fleetdeck colours the items with it)


# ═══ EXAM #2 v4 (2026-11-09, results/EXAM2_SPEC.md section 11): 8 push directions (the sides + the diagonals, 48 pushes), the rung error
# floor 0.05 (small-command precision), the balance partner = CURRENT_BALANCE of the rig worktree (exam.sh), and the CONSTRAINTS block =
# feasibility, SEPARATE from the points (the Optuna study's constraints_func: c <= 0 = feasible). Weights unchanged (operator 2026-11-09).
WALK_MIN_CMD = {"vx": 0.10, "vy": 0.10, "wz": 0.20}   # "walks": the rungs from this |command| on (m/s, m/s, rad/s) ...
WALK_GAIN = 0.5    # ... pass when the achieved speed along the command / the command >= this (right sign; a fell / missing rung fails) ...
WALK_SHARE = 0.90  # ... and >= this share of them per axis must pass (operator's hard rule: standing must not beat walking). Definitions.


def _env_floor(s, d):
    """'vx=0.9,vy=0.8,wz08=0.5' -> {vx: 0.9, ...} over the defaults d"""
    out = dict(d)
    for kv in (s or "").split(","):
        if "=" in kv:
            k, v = kv.split("=", 1); out[k.strip()] = float(v)
    return out


# PROVISIONAL (the final values come from the pilot's noise measurement, theta = mu* - z sigma): the tracking floors of the 11-03 design
# (vx / vy = the mean capped gain min(max(gain, 0), 1) over the "walks" rungs of the axis, wz08 = the same over the two +-0.8 rad/s rungs)
# and the walking torso-tilt limit (walk_tilt = the mean torso tilt over the walking rungs |v| <= 0.5 m/s, the stability row's value), set
# 2026-11-09 above the six v4 anchors with margin (anchors 2.1..6.3 deg in v3; 59 lm7j..lm7l records 1.6..9.6 deg, median 3.6).
TRACK_FLOOR = _env_floor(os.environ.get("EXAM_TRACK_FLOOR"), {"vx": 0.9, "vy": 0.8, "wz08": 0.5})
TILT_FLOOR_DEG = float(os.environ.get("EXAM_TILT_FLOOR_DEG", "8.0"))


def ladder_commands(ax, thr=0.0):
    """the ladder's rung commands on axis ax (both directions, from LADDER_TOPS) with |c| >= thr"""
    up, dn = LADDER_TOPS[ax]
    cs = [round(0.05 * k, 2) for k in range(1, int(round(up / 0.05)) + 1)] + [round(-0.05 * k, 2) for k in range(1, int(round(dn / 0.05)) + 1)]
    return [c for c in cs if abs(c) >= thr - 1e-9]


def rung_gains(ladders):
    """{axis: {command: gain}}: gain = the sim-truth speed along the rung's own axis / the command (negative = the wrong way); None = the
    rung fell (incl. a v3 hard fail on the marked copy) or has no v_true"""
    _, rungs, _ = ladder_parts(ladders)
    G = {"vx": {}, "vy": {}, "wz": {}}
    for r in rungs:
        ax = _axis(r["cmd"]); k = ("vx", "vy", "wz").index(ax); c = r["cmd"][k]
        G[ax][round(c, 2)] = None if r.get("fell") or not r.get("v_true") else r["v_true"][k] / c
    return G


def constraints_v4(sc, ladders, pushes=None, track_floor=None, tilt_floor_deg=None, walk_min=None):
    """the v4 feasibility block: [{name, value, threshold, sense, c, pass, provisional, detail}], c <= 0 = feasible (Optuna's
    constraints_func convention: c = threshold - value for a '>=' constraint, value - threshold for '<='; a value that could not be
    measured = c 1, infeasible). sc = the score (its 'falls' and 'walk_tilt' rows); ladders / pushes = the HARD-FAIL-MARKED copies."""
    tf = dict(TRACK_FLOOR, **(track_floor or {})); tl = TILT_FLOOR_DEG if tilt_floor_deg is None else float(tilt_floor_deg)
    m = {x["key"]: x for x in sc["metrics"]}
    C = []

    def add(name, value, thr, sense, prov, detail):
        c = 1.0 if value is None else round((thr - value) if sense == ">=" else (value - thr), 6)
        C.append({"name": name, "value": None if value is None else round(float(value), 4), "threshold": thr, "sense": sense,
                  "c": round(float(c), 4), "pass": c <= 0, "provisional": prov, "detail": detail})

    # no_falls: the survival row's count = falls in Tests 0 / 1 / 3 + the v3 hard-fail groups outside the pushes (a push that knocks the
    # robot over belongs to the push score, spec section 2 -- shown in the detail, not counted)
    ran = bool(ladders and ladders.get("segments"))
    pf = sum(1 for s in (pushes or {}).get("segments", []) if s.get("push") and not s["push"].get("held"))
    nf = m["falls"]["value"] if ran and "falls" in m else None
    add("no_falls", nf, 0, "<=", False, ("Test 1 did not run" if not ran else f"{nf} falls incl. hard fails (Tests 0, 1, 3)") +
        f"; push falls {pf} (scored in pushes, not counted)")
    G = rung_gains(ladders)
    wm = WALK_MIN_CMD if walk_min is None else walk_min   # v4: the rungs from 0.10 m/s (0.2 rad/s); Exam #3 passes E3_WALK_MIN_CMD = EVERY rung
    for ax in ("vx", "vy", "wz"):   # walks: >= 90 % of the counted rungs reach gain 0.5
        cs = ladder_commands(ax, wm[ax])
        ok = [c for c in cs if (G[ax].get(c) or 0.0) >= WALK_GAIN]
        short = [f"{c:+.2f}" + ("" if G[ax].get(c) is None else f" ({G[ax][c]:.2f})") for c in cs if c not in ok]
        add(f"walks_{ax}", len(ok) / len(cs), WALK_SHARE, ">=", False,
            f"{len(ok)}/{len(cs)} rungs with |c| >= {wm[ax]:g} reach gain >= {WALK_GAIN:g}" + (f"; short: {', '.join(short)}" if short else ""))
    cap = lambda g: 0.0 if g is None else min(max(g, 0.0), 1.0)
    for ax in ("vx", "vy"):   # track_floor: the mean capped gain over the same rungs (overshoot does not buy back an undershoot)
        cs = ladder_commands(ax, WALK_MIN_CMD[ax])
        add(f"track_floor_{ax}", mean([cap(G[ax].get(c)) for c in cs]), tf[ax], ">=", True,
            f"mean min(gain, 1) over the {len(cs)} rungs with |c| >= {WALK_MIN_CMD[ax]:g} (provisional floor, 11-03 design)")
    w8 = [G["wz"].get(0.8), G["wz"].get(-0.8)]
    add("track_floor_wz08", mean([cap(g) for g in w8]), tf["wz08"], ">=", True,
        "mean min(gain, 1) at wz +-0.8 rad/s: " + " / ".join("—" if g is None else f"{g:.2f}" for g in w8) + " (provisional floor, 11-03 design)")
    wt = (m.get("walk_tilt") or {}).get("value")
    add("tilt_floor", wt, tl, "<=", True, f"mean torso tilt over the walking rungs |v| <= 0.5 m/s (deg); provisional limit {tl:g} deg (2026-11-09: anchors pass with margin)")
    return C


def score_v4(handover, ladders, pushes, homing, n_homing=10, cal=None, tilt_fail_deg=None, track_floor=None, tilt_floor_deg=None):
    """Exam #2 v4 (2026-11-09): score_v3 (hard fails, the same weights and rows) with the rung error floor E_FLOOR_V4 (0.05), the push score
    over EXPECTED_PUSHES_V4 (48: 8 directions) and the v4 calibration (results/exam2_calibration_v4.json); + 'constraints' (constraints_v4)
    and 'feasible' (all pass). Version 4; v2 / v3 records keep score_v2 / score_v3 and the v2 calibration."""
    C, cver = (cal, "given") if cal is not None else load_cal_v4()
    out = {}
    sc = score_v3(handover, ladders, pushes, homing, n_homing=n_homing, cal=C, tilt_fail_deg=tilt_fail_deg, _v4=True, _out=out)
    sc["version"] = 4; sc["calib"] = cver
    sc["constraints"] = constraints_v4(sc, out["L"], out["P"], track_floor, tilt_floor_deg)
    sc["feasible"] = all(c["pass"] for c in sc["constraints"])
    dirs = sorted({s["push"]["dir"] for s in (out["P"] or {}).get("segments", []) if s.get("push")})
    sc["v4"] = {"e_floor": E_FLOOR_V4, "expected_pushes": EXPECTED_PUSHES_V4, "push_dirs": dirs,
                "walks": {"min_cmd": WALK_MIN_CMD, "gain": WALK_GAIN, "share": WALK_SHARE},
                "track_floor": dict(TRACK_FLOOR, **(track_floor or {})), "tilt_floor_deg": TILT_FLOOR_DEG if tilt_floor_deg is None else float(tilt_floor_deg)}
    if dirs and len(dirs) < len(PUSH_DIRS_V4):   # a 4-direction (v3) push test scored as v4 loses half the push points: say so
        sc["v4"]["warning"] = f"push test has {len(dirs)} directions, v4 expects {len(PUSH_DIRS_V4)} (a v3 exam: score it with score_v3)"
    return sc


# ═══ EXAM #3 (2026-11-13, results/EXAM2_SPEC.md sections 12 + 13) = Exam #2 v4 + Test 6 ENCODER OFFSETS, scored INSIDE the qualities ═══
# Owner 2026-11-13: "No flags. This is our new test"; the offset pushes go "under survivability, stability and gait and body care. Energy."
# score_v2 / v3 / v4 above are untouched: `exam.sh --exam 2` and every stored record keep them. Exam #3 records carry meta.exam_version 3
# and meta.score.version 5 (+ meta.score.exam 3).
# Test 6: the rig's robot gets FIXED encoder zero-offset patterns (tools/enc_patterns.py: reported = true + offset; the Bridge corrects
# nothing in sim): P0 zero (the paired baseline), P1..P4 inside the trained +-0.010 rad band, P5 = the robot's measured residuals unscaled
# (the STRESS pattern, 0.0305 rad at the ankle pitch). Per pattern a 15 s stand and a 15 s measured walk at vx 0.30; then, still under P5,
# Test 2's 1.0 m/s kick from the 4 sides, standing and walking (8 OFFSET PUSHES; Test 2 holds the same 8 on perfect encoders).
# What the validation showed (section 12.5): the in-band BIAS numbers (lean, drift vs P0) do not tell a band-trained policy from an
# untrained one; the P5 STAND does (band-trained: 0 foot lifts and 2 cm of base drift in 4 of 4 runs; the others 1.5-6.5 lifts, 6-23 cm).
# So the bias numbers stay in the record (meta.score.enc.patterns[*].d) and the points / constraints read the P5 stand, the walks' cost
# under offsets and the offset pushes.
#
# POINTS: the seven quality totals stay (QUALITIES). Inside a quality Test 6 takes a fixed share E3_SHARE[q] and every v4 row of that
# quality is scaled by (W_q - share) / W_q -- the v3 way of making room (it halved the stand rows for the second stand):
#   survival 10   10 x 0.5^falls; falls now also = a fall / hard fail in any Test 6 pattern segment + an offset push not held
#   stability 30  v4 rows x 25/30 (25) + Test 6 (5): P5 stand base drift 1.5, P5 stand foot lifts 1.0, P5 stand torso rate 0.5 -- on the
#                 stand's OWN ramps (stand_drift 1 -> 10 cm, stand_lifts 0 -> 3, stand_rate: a stand is a stand) -- + the worst in-band
#                 stand's base drift 1.0 (stand_drift again) + the worst walking torso-tilt change vs P0 over P1..P5 1.0 (enc_walk_tilt)
#   gait 10       v4 rows x 8/10 (8) + Test 6 (2): touchdown impact p95 change vs P0, mean over the pattern walks 1.0 (enc_impact, BW) +
#                 action roughness change vs P0 1.0 (enc_smooth, rad)
#   energy 10     v4 rows x 8/10 (8) + Test 6 (2): recovery energy of the held offset pushes 1.0 (enc_push_energy, J) + walking
#                 mechanical power change vs P0, mean over the pattern walks 1.0 (enc_power, W)
#   pushes 10     ONE push score over 48 + 8 = 56 pushes, every push the same formula and ramps (PUSH_W, push_*): the v4 row carries
#                 10 x 48/56 = 8.57, the offset pushes 10 x 8/56 = 1.43
#   tracking 20, homing 10: unchanged.
# Worked: P5 stand drift 2.1 cm -> s = (10 - 2.1) / 9 = 0.88 -> 1.32 of 1.5 points; 6.6 cm -> 0.38 -> 0.57.
# The five new ramps (CAL_E3_DEFAULT, provisional) are calibrated like every provisional key: p10 -> p90 of the validation population,
# never narrower than half the median (tools/exam_calibrate.py --e3 -> results/exam3_calibration.json = the v4 table + these keys).
#
# CONSTRAINTS (the v4 eight, with walks_vx / vy / wz counting EVERY ladder rung -- E3_WALK_MIN_CMD, operator 2026-11-13 -- instead of the
# rungs from 0.10 m/s / 0.2 rad/s; the morning's separate walks_low constraint is gone, e3.walks_low keeps the low-rung counts as information) +:
#   enc_no_falls     (definition) falls + hard fails in the stand / walk of ANY pattern, P5 included, <= 0 (an offset push that knocks the
#                    robot over is a push: it costs survival and push points and is shown here, not counted -- the v4 rule for Test 2)
#   enc_stand_drift  (PROVISIONAL) base drift over the 15 s stand under P5 <= ENC_STAND_DRIFT_CM
#   enc_stand_lifts  (PROVISIONAL) foot lifts in that stand <= ENC_STAND_LIFTS
# Not measurable (Test 6 did not run, the P5 stand fell, or the INJECTION CHECK failed: the stack's observed joint angle minus the sim truth
# must equal the pattern within ENC_INJ_TOL, else the policy never saw the offsets) = c 1, infeasible; the Test 6 rows then score 0.
#
# REV 2 (2026-11-13, meta.exam_rev 2, meta.score.version 6 + 'rev' 2): section 13.4 found the P5 stand bimodal for an untrained policy (it
# stands still or creeps away stepping; one 15 s stand samples that coin once), so the stress pattern's stand is entered THREE times
# (enc:P5:stand; then pre2 = 8 s walk at 0.30 on perfect encoders, settle2 = P5 lands again, stand2; the same again for 3) and the three P5
# rows + enc_stand_drift / enc_stand_lifts read the WORST of the three PER METRIC: drift = max, lifts = max, rate = max. A fall / hard
# fail in any of the repeated segments is a fall of P5 (survival, enc_no_falls) and makes the P5 stand "not measured"; each stand runs the
# injection check. Weights, ramps, calibration: rev 1's. Worked: drifts 1.8 / 2.2 / 42.0 cm -> 42.0 cm -> s = 0 of 1.5 (rev 1 would have
# read 1.8 cm -> (10 - 1.8) / 9 x 1.5 = 1.37). Rev 1 records (one stand) score exactly as before (version 5).
E3_REV2_P5_STANDS = 3
ENC_STAND_DRIFT_CM = float(os.environ.get("EXAM_ENC_STAND_DRIFT_CM", "5.0"))   # PROVISIONAL (section 13.4)
ENC_STAND_LIFTS = float(os.environ.get("EXAM_ENC_STAND_LIFTS", "1"))          # PROVISIONAL (section 13.4)
WALKS_LOW_SHARE = float(os.environ.get("EXAM_WALKS_LOW_SHARE", "0.5"))        # INFORMATION only since the all-rungs rule (e3.walks_low; no constraint)
STAND_QUIET_LIFTS = float(os.environ.get("EXAM_STAND_QUIET_LIFTS", "6"))     # PROVISIONAL (operator 2026-11-13: a stillness constraint, so a study cannot
#   buy low-speed walking with a restless stand): foot lifts in the 60 s stand + the 120 s stand after walking <= this (= the two stand_lifts
#   ramps' bad ends, 3 + 3). 2026-11-13 records: lm7n (band) 1 and 2, s1_pilot t36 4, squeeze 10, t37 12, arch_armsway 13, t21 17; 16 of 36 s1 finals pass.
E3_WALK_MIN_CMD = {"vx": 0.05, "vy": 0.05, "wz": 0.05}   # operator 2026-11-13: "all rungs should be counted" -> walks_vx / vy / wz of Exam #3 count
#   EVERY ladder rung (vx 30, vy 12, wz 32) at the same gain 0.5 and share 0.9: at most 3 / 1 / 3 rungs may fall short. The v4 rule (from
#   0.10 m/s, 0.2 rad/s) let a policy stand at every low rung and pass; it stays in score_v4 / feasible_v4 of the record.
WALKS_LOW_CMD = {"vx": (0.05, 0.10), "vy": (0.05, 0.10), "wz": (0.10, 0.15, 0.20, 0.25, 0.30)}   # |command| of the low rungs, both signs
ENC_INJ_TOL = 0.0015   # rad: |mean(observed - true) - offset| per joint over a 15 s stand (the two 50 Hz streams are one frame apart at most)
ENC_BASE = "P0"
E3_SHARE = {"stability": 5.0, "gait": 2.0, "energy": 2.0}   # Test 6's points inside a quality; the v4 rows scale by (W - share) / W
E3_PUSHES = 8                                               # the offset pushes: 1.0 m/s x 4 sides x stand / walk, under the stress pattern
E3_W = {"enc_stand_drift": 1.5, "enc_stand_lifts": 1.0, "enc_stand_rate": 0.5, "enc_creep": 1.0, "enc_walk_tilt": 1.0,   # stability 5
        "enc_impact": 1.0, "enc_smooth": 1.0,                                                                           # gait 2
        "enc_push_energy": 1.0, "enc_power": 1.0}                                                                       # energy 2
# the NEW ramps (g, b), first values; PROVISIONAL -> results/exam3_calibration.json. The P5 stand rows use stand_drift / stand_lifts / stand_rate.
CAL_E3_DEFAULT = {"enc_walk_tilt": (0.5, 2.0), "enc_impact": (0.0, 0.4), "enc_smooth": (0.0, 0.01), "enc_push_energy": (500.0, 1600.0), "enc_power": (0.0, 30.0)}
PROV_E3 = set(CAL_E3_DEFAULT)
CAL_FILE_E3 = os.path.join(os.path.dirname(CAL_FILE), "exam3_calibration.json")


def load_cal_e3():
    """(table, version) for score_e3: the v4 table + the Test 6 keys, every key overridden by results/exam3_calibration.json when present
    (that file is complete: tools/exam_calibrate.py --e3 copies the keys it does not re-derive from the v4 table)"""
    cal, ver = load_cal_v4()
    cal = dict(cal, **CAL_E3_DEFAULT); ver = f"{ver} + e3 default"
    try:
        d = json.load(open(CAL_FILE_E3))
        for k, v in (d.get("cal") or {}).items():
            if k in cal: cal[k] = (float(v[0]), float(v[1]))
        ver = d.get("version", "?")
    except (OSError, ValueError):
        pass
    return cal, ver


def _enc_hard(seg, z_stand, tf, push=False):
    """the v3 hard-fail criteria on one Test 6 segment -> a reason or None (a commander fall is already seg['fell']); push = a push
    recovery: only the deep single-sample drop fails on height (a dip can be a good recovery)"""
    if seg.get("fell"):
        return "fell" if seg["fell"] is True else str(seg["fell"])
    if (seg.get("tilt_max") or 0.0) >= tf:
        return f"tilt {seg['tilt_max']:.1f} >= {tf:g} deg"
    fc = seg.get("floor_contact")
    if fc and (fc.get("peak_N") or 0.0) > CONTACT_FAIL_N:
        return f"{fc.get('body_peak') or fc.get('body')} on the floor {fc['peak_N']:.0f} N"
    if z_stand is not None:
        if push:
            if seg.get("z_min") is not None and seg["z_min"] < z_stand - Z_PUSH_DROP:
                return f"z {seg['z_min']:.3f} < {z_stand - Z_PUSH_DROP:.3f}"
        else:
            if seg.get("z_run_min") is not None and seg["z_run_min"] < z_stand - Z_RUN_DROP:
                return f"1 s mean z {seg['z_run_min']:.3f} < {z_stand - Z_RUN_DROP:.3f}"
            if seg.get("z_min") is not None and seg["z_min"] < z_stand - Z_SAMPLE_DROP:
                return f"z {seg['z_min']:.3f} < {z_stand - Z_SAMPLE_DROP:.3f}"
    up, cr = seg.get("arm_up_s") or [0, 0], seg.get("arm_cross_s") or [0, 0]
    if max(up) > ARM_FAIL_S or max(cr) > ARM_FAIL_S:
        return f"arm envelope (up {max(up):.2f} s, cross {max(cr):.2f} s)"
    return None


def _push_row(p):
    """a push segment's 'push' dict -> the item the push score ramps (the shape raw_values builds for Test 2)"""
    return {"held": bool(p.get("held")), "dist": p.get("dist_m"), "tilt": p.get("tilt_peak_deg"),
            "rec": p.get("recovery_s") if p.get("recovery_s") is not None else 5.0, "steps": max(p.get("steps") or 0, 0), "foot": p.get("foot_peak_bw"),
            "counter": p.get("counter_sway_deg"), "sway": p.get("sway_rate_rms"), "sep": p.get("sep_dev_cm"), "arm": p.get("arm_peak_deg"),
            "energy": p.get("energy_J"), "mode": p.get("mode"), "dir": p.get("dir"), "level": p.get("level")}


def _stand_no(phase):
    """'stand' -> 1, 'stand2' -> 2, ... (rev 2's repeated stress-pattern stands); any other phase -> None"""
    if phase == "stand":
        return 1
    return int(phase[5:]) if phase.startswith("stand") and phase[5:].isdigit() and int(phase[5:]) >= 2 else None


def _rep_phase(phase):
    """rev 2 (2026-11-13): the stress pattern's repeated block 'pre2' / 'settle2' / 'stand2', 'pre3', ... -> True"""
    base = phase.rstrip("0123456789")
    return base in ("pre", "settle", "stand") and phase != base and int(phase[len(base):]) >= 2


def enc_summary(enc, tilt_fail_deg=None, stands=None):
    """Test 6 -> {ran, result, patterns: {key: {name, stress, max_abs, stand, walk (the raw segment numbers), fell [reasons], d {the paired
    differences vs P0: lean, stand_drift, walk_vy, ...}, inj}}, base (= P0's raw numbers), worst / mean {bias metric} over the in-band
    patterns, stress (the stress pattern's key), pushes [the offset pushes, _push_row + seg], falls (pattern segments, every pattern),
    stress_falls (of them under the stress pattern), push_falls, injection {ok, max_err, why}}. Missing record -> {ran: False}.
    Exam #3 rev 2 (2026-11-13): the stress pattern's stand is entered N times (phases stand, stand2, stand3, each after its own pre<k> walk on
    perfect encoders and settle<k>). Then patterns[stress]: 'stand' = the WORST of the N per metric (drift_m, lifts, rate_rms = the max; the
    other fields from the stand with the largest drift_m; None = not measured when ANY of them fell / hard-failed / is missing), 'stands' =
    each one's own numbers (+ phase, fell, inj), 'n_stands', 'stand_worst_of' {metric: phase}; a fall / hard fail in any of the repeated
    segments (pre<k> / settle<k> / stand<k>) is a fall of the pattern like one in 'stand'; every stand runs the injection check. d (the bias
    numbers vs P0) keeps reading the FIRST stand. stands = the expected N for the stress pattern (score_e3 rev 2: 3): fewer in the record =
    'missing' (an abort, counted once). A record with one stand (rev 1) gives exactly the rev-1 output."""
    tf = TILT_FAIL_DEG if tilt_fail_deg is None else tilt_fail_deg
    if not enc or not enc.get("segments"):
        return {"ran": False, "result": (enc or {}).get("result") or "MISSING"}
    meta = {p["key"]: p for p in ((enc.get("patterns") or {}).get("patterns") or [])}
    by = {}
    for s in enc["segments"]:
        e = s.get("enc") or {}
        if e.get("pattern"):
            by.setdefault(e["pattern"], {})[e["phase"]] = s
    b = by.get(ENC_BASE, {})
    z_stand = (b.get("stand") or {}).get("z_mean")
    out = {"ran": True, "result": enc.get("result"), "patterns": {}, "falls": 0, "stress_falls": 0, "z_stand": z_stand,
           "stress": next((k for k, pm in meta.items() if pm.get("stress")), None)}

    def raw(seg):   # the segment's numbers when it stood through its window, else None: the enc block + the engine's generic rows
        e = (seg or {}).get("enc") or {}
        if seg is None or not e.get("T") or _enc_hard(seg, z_stand, tf) is not None:
            return None
        en = seg.get("energy") or {}
        return dict(e, lifts=sum(seg.get("lifts") or [0]), rate_rms=seg.get("rate_rms"), tau2_mean=en.get("tau2_mean"), P_W=en.get("P_W"),
                    impact_bw=(seg["impact_p95_N"] / W_BODY) if seg.get("impact_p95_N") else None, smooth=seg.get("qt_acc_rms"),
                    drift_m=round((e["drift_fwd"] ** 2 + e["drift_lat"] ** 2) ** 0.5, 4))

    b_st, b_wk = raw(b.get("stand")), raw(b.get("walk"))
    inj_ok, inj_err, inj_why = True, 0.0, []
    ST_KEYS = ("pitch", "roll", "tilt", "z", "drift_fwd", "drift_lat", "drift_m", "drift_mm_s", "yaw_rate_deg_s", "T", "lifts", "rate_rms", "tau2_mean")
    keep = lambda r, ks: None if r is None else {k: r.get(k) for k in ks}
    for key, pm in meta.items():
        ph = by.get(key, {})
        rep = any(_rep_phase(n) for n in ph)   # rev 2: the stress pattern's repeated stands -> its segments in record order
        fell = [f"{n}: {r}" for n, r in ((n, _enc_hard(ph[n], z_stand, tf)) for n in
                                         ([n for n in ph if n in ("settle", "stand", "walk", "hold") or _rep_phase(n)] if rep else
                                          [n for n in ("settle", "stand", "walk", "hold") if n in ph])) if r]
        sn = sorted((_stand_no(n), n) for n in ph if _stand_no(n))   # [(1, 'stand'), (2, 'stand2'), ...]
        n_exp = stands if (stands and pm.get("stress")) else 0
        missing = [n for n in ("stand", "walk") if n not in ph] + [f"stand{k}" for k in range(2, n_exp + 1) if f"stand{k}" not in ph]
        multi = max(len(sn), n_exp) > 1
        st_, wk = raw(ph.get("stand")), raw(ph.get("walk"))
        d = {}
        if st_ and b_st and key != ENC_BASE:
            dp, dr = st_["pitch"] - b_st["pitch"], st_["roll"] - b_st["roll"]
            Tm = 0.5 * (st_["T"] + b_st["T"])
            d.update({"pitch": round(dp, 3), "roll": round(dr, 3), "lean": round((dp * dp + dr * dr) ** 0.5, 3),
                      "stand_drift": round(1000.0 * ((st_["drift_fwd"] - b_st["drift_fwd"]) ** 2 + (st_["drift_lat"] - b_st["drift_lat"]) ** 2) ** 0.5 / Tm, 2),
                      "stand_yaw": round(st_["yaw_rate_deg_s"] - b_st["yaw_rate_deg_s"], 3)})
        if wk and b_wk and key != ENC_BASE:
            df = lambda k: None if wk.get(k) is None or b_wk.get(k) is None else wk[k] - b_wk[k]
            rnd = lambda x, n: None if x is None else round(x, n)
            d.update({"walk_vy": round(1000.0 * (wk["vy"] - b_wk["vy"]), 1), "walk_yaw": round(wk["yaw_rate_deg_s"] - b_wk["yaw_rate_deg_s"], 3),
                      "walk_vx": round(wk["vx"] - b_wk["vx"], 4), "walk_tilt": round(wk["tilt"] - b_wk["tilt"], 3),
                      "walk_pitch": round(wk["pitch"] - b_wk["pitch"], 3), "walk_roll": round(wk["roll"] - b_wk["roll"], 3),
                      "walk_impact": rnd(df("impact_bw"), 4), "walk_smooth": rnd(df("smooth"), 6), "walk_power": rnd(df("P_W"), 2)})
        # the injection check of this pattern's stand(s): the stack observed true + offset, and the sim echoes the table it applies
        off = pm.get("offsets") or []
        injs = {}
        for _, n in (sn or [(1, "stand")]):
            e = (ph.get(n) or {}).get("enc") or {}
            inj = {"n": e.get("inj_n"), "err": None, "echo": None}
            if e.get("obs_minus_true") and len(e["obs_minus_true"]) == len(off):
                inj["err"] = round(max(abs(a - o) for a, o in zip(e["obs_minus_true"], off)), 5)
            if any(off):
                inj["echo"] = bool(e.get("eo")) and len(e["eo"]) == len(off) and max(abs(a - o) for a, o in zip(e["eo"], off)) < 2e-5
            else:
                inj["echo"] = not e.get("eo_n")   # perfect encoders: the sim sends no echo
            injs[n] = inj
            lab = key if n == "stand" else f"{key} {n}"
            if n in ph and not ph[n].get("fell"):
                if inj["err"] is None: inj_ok = False; inj_why.append(f"{lab}: no observed joint angles")
                else:
                    inj_err = max(inj_err, inj["err"])
                    if inj["err"] > ENC_INJ_TOL: inj_ok = False; inj_why.append(f"{lab}: observed - true off by {1000 * inj['err']:.1f} mrad")
                if not inj["echo"]: inj_ok = False; inj_why.append(f"{lab}: the sim's echo does not match the pattern")
        inj = next(iter(injs.values()))
        st_keep, extra = keep(st_, ST_KEYS), {}
        if multi:   # rev 2: the stress pattern's stand entered N times -> the worst per metric; each stand's own numbers in 'stands'
            rows = []
            for _, n in sn:
                r = raw(ph[n])
                row = dict({"phase": n}, **(keep(r, ST_KEYS) or {"lifts": sum(ph[n].get("lifts") or [0])}))
                row.update({"measured": r is not None, "fell": _enc_hard(ph[n], z_stand, tf), "inj": injs[n]})
                rows.append(row)
            worst_of = None
            if rows and all(row["measured"] for row in rows) and not any(_stand_no(m) for m in missing):
                wst = max(rows, key=lambda row: row["drift_m"])   # the first of equal drifts
                st_keep = {k: wst.get(k) for k in ST_KEYS}; worst_of = {"drift_m": wst["phase"]}
                for mk in ("lifts", "rate_rms"):
                    vals = [(row[mk], row["phase"]) for row in rows if row.get(mk) is not None]
                    if vals:
                        st_keep[mk], worst_of[mk] = max(vals, key=lambda t: t[0])
            else:
                st_keep = None   # one of the N fell / hard-failed / was not reached: the worst is not measured
            ns_ = [i["n"] for i in injs.values() if i["n"] is not None]; er_ = [i["err"] for i in injs.values() if i["err"] is not None]
            inj = {"n": min(ns_) if ns_ else None, "err": max(er_) if er_ else None, "echo": all(i["echo"] for i in injs.values())}
            extra = {"n_stands": max(len(sn), n_exp), "stands": rows, "stand_worst_of": worst_of}
        out["patterns"][key] = {"name": pm.get("name"), "stress": bool(pm.get("stress")), "max_abs": pm.get("max_abs"), "offsets": off,
                                "stand": st_keep,
                                "walk": keep(wk, ("vx", "vy", "vx_err", "lat_v", "yaw_rate_deg_s", "pitch", "roll", "tilt", "z", "T", "impact_bw", "smooth", "P_W", "rate_rms")),
                                "lifts_stand": max(sum(ph[n].get("lifts") or [0]) for _, n in sn) if multi and sn else sum((ph.get("stand") or {}).get("lifts") or [0]),
                                "fell": fell, "missing": missing, "d": d, "inj": inj, **extra}
        n_bad = len(fell) + (1 if missing and not fell else 0)   # a pattern the test never reached (an abort) counts once
        out["falls"] += n_bad
        if pm.get("stress"): out["stress_falls"] += n_bad
    if not any("stand" in by.get(k, {}) for k in meta):
        inj_ok = False; inj_why.append("no pattern stand was measured")
    out["injection"] = {"ok": inj_ok, "max_err": round(inj_err, 5), "tol": ENC_INJ_TOL, "why": inj_why}
    out["base"] = {"stand": out["patterns"].get(ENC_BASE, {}).get("stand"), "walk": out["patterns"].get(ENC_BASE, {}).get("walk")}
    worst, avg = {}, {}
    for mk in ("lean", "stand_drift", "stand_yaw", "walk_vy", "walk_yaw", "walk_vx", "walk_tilt"):
        vals = [(abs(p["d"][mk]), k) for k, p in out["patterns"].items() if not p["stress"] and k != ENC_BASE and mk in p["d"]]
        n_exp = sum(1 for k, p in out["patterns"].items() if not p["stress"] and k != ENC_BASE)
        worst[mk] = list(max(vals)) if vals else None
        avg[mk] = round(sum(v for v, _ in vals) / len(vals), 3) if vals else None
        out.setdefault("n_measured", {})[mk] = [len(vals), n_exp]
    out["worst"], out["mean"] = worst, avg
    # the offset pushes (the segments after the stress pattern's "hold"): Test 2's own record shape; a v3 hard fail = not held
    out["pushes"] = []
    for s in enc["segments"]:
        p = s.get("push")
        if not p: continue
        row = _push_row(p)
        why = None if not row["held"] else _enc_hard(s, z_stand, tf, push=True)
        if why: row["held"] = False
        row["why"] = why or (None if row["held"] else (s.get("fell") if isinstance(s.get("fell"), str) else "fell"))
        row["tilt_max"] = s.get("tilt_max"); row["z_min"] = s.get("z_min")
        out["pushes"].append(row)
    out["push_falls"] = sum(1 for r in out["pushes"] if not r["held"])
    out["push_pattern"] = (enc.get("timing") or {}).get("push_pattern")
    return out


def enc_values(es):
    """the Test 6 quantities the Exam #3 score ramps, before any ramp ({} keys None when not measured); exam_calibrate.py --e3 reads these"""
    V = {k: None for k in E3_W}
    if not es.get("ran"):
        return V
    P = es["patterns"]; sk = es.get("stress")
    sst = (P.get(sk) or {}).get("stand") if sk else None
    if sst:
        V["enc_stand_drift"], V["enc_stand_lifts"], V["enc_stand_rate"] = sst.get("drift_m"), sst.get("lifts"), sst.get("rate_rms")
    inb = [p["stand"]["drift_m"] for k, p in P.items() if not p["stress"] and k != ENC_BASE and p.get("stand")]
    V["enc_creep"] = max(inb) if inb else None
    dv = lambda mk: [p["d"][mk] for k, p in P.items() if k != ENC_BASE and p["d"].get(mk) is not None]
    V["enc_walk_tilt"] = max(dv("walk_tilt")) if dv("walk_tilt") else None
    V["enc_impact"] = mean(dv("walk_impact")); V["enc_smooth"] = mean(dv("walk_smooth")); V["enc_power"] = mean(dv("walk_power"))
    V["enc_push_energy"] = mean([r["energy"] for r in es.get("pushes") or [] if r["held"] and r.get("energy") is not None])
    return V


def walks_low(ladders):
    """{axis: [ok, n, [short rungs]], share}: the LOW ladder rungs (WALKS_LOW_CMD, both signs) that reach gain >= WALK_GAIN (the
    hard-fail-marked ladder record; a fell / missing rung fails)"""
    G = rung_gains(ladders); out = {}; ok_all = n_all = 0
    for ax, mags in WALKS_LOW_CMD.items():
        cs = [sg * m for m in mags for sg in (1, -1)]
        ok = [c for c in cs if (G[ax].get(round(c, 2)) or 0.0) >= WALK_GAIN]
        out[ax] = [len(ok), len(cs), [f"{c:+.2f}" + ("" if G[ax].get(round(c, 2)) is None else f" ({G[ax][round(c, 2)]:.2f})") for c in cs if c not in ok]]
        ok_all += len(ok); n_all += len(cs)
    out["share"] = ok_all / n_all
    return out


def _enc_bad(p):
    """a pattern's falls for the texts, or 'not reached'; rev 2 (n_stands) names what was not reached (rev 1: the text as it was)"""
    return ", ".join(p["fell"]) or ("not reached" + (f" ({', '.join(p['missing'])})" if p.get("n_stands") else ""))


def constraints_e3(es, ladders, drift_cm=None, lifts=None, metrics=None):
    """the Exam #3 constraints added to constraints_v4's eight (the same record format); ladders unused since walks_low left the list"""
    dc = ENC_STAND_DRIFT_CM if drift_cm is None else float(drift_cm); lf = ENC_STAND_LIFTS if lifts is None else float(lifts)
    C = []

    def add(name, value, thr, sense, prov, detail):
        c = 1.0 if value is None else round((thr - value) if sense == ">=" else (value - thr), 6)
        C.append({"name": name, "value": None if value is None else round(float(value), 4), "threshold": thr, "sense": sense,
                  "c": round(float(c), 4), "pass": c <= 0, "provisional": prov, "detail": detail})

    mv = {m["key"]: m.get("value") for m in (metrics or [])}
    sl, wl_ = mv.get("stand_lifts"), mv.get("wstand_lifts")
    add("stand_quiet", None if sl is None or wl_ is None else sl + wl_, STAND_QUIET_LIFTS, "<=", True,
        "the stands were not measured" if sl is None or wl_ is None else
        f"foot lifts: 60 s stand {sl:g} + 120 s stand after walking {wl_:g}; PROVISIONAL limit {STAND_QUIET_LIFTS:g} (2026-11-13)")
    if not es.get("ran"):
        why = f"Test 6 did not run ({es.get('result')})"
        add("enc_no_falls", None, 0, "<=", False, why); add("enc_stand_drift", None, dc, "<=", True, why); add("enc_stand_lifts", None, lf, "<=", True, why)
        return C
    inj = es["injection"]; P = es["patterns"]; sk = es.get("stress")
    fl = "; ".join(f"{k} {_enc_bad(p)}" for k, p in P.items() if p["fell"] or p["missing"])
    bad_inj = None if inj["ok"] else "INJECTION NOT VERIFIED (" + "; ".join(inj["why"][:3]) + "): the policy may never have seen the offsets"
    pf = [f"{r['mode']} {r['dir']}" for r in es.get("pushes") or [] if not r["held"]]
    add("enc_no_falls", None if bad_inj else es["falls"], 0, "<=", False,
        bad_inj or (f"{es['falls']} falls / hard fails in the stand / walk of the patterns ({', '.join(P)})" + (f": {fl}" if fl else "") +
                    f"; offset pushes not held {len(pf)}/{len(es.get('pushes') or [])}" + (f" ({', '.join(pf)})" if pf else "") + " (scored in survival + pushes, not counted here)"))
    sst = (P.get(sk) or {}).get("stand") if sk else None
    ns = int((P.get(sk) or {}).get("n_stands") or 1) if sk else 1   # rev 2: the stress stand entered N times, the worst of N is read
    if ns <= 1:
        why_none = bad_inj or (None if sst else f"the {sk} stand was not measured (fell / not reached)")
        add("enc_stand_drift", None if why_none else 100.0 * sst["drift_m"], dc, "<=", True,
            why_none or f"base drift over the {sst['T']:g} s stand under {sk} (the unscaled measured residuals), cm; PROVISIONAL limit {dc:g} cm (2026-11-13)")
        add("enc_stand_lifts", None if why_none else sst["lifts"], lf, "<=", True,
            why_none or f"foot lifts in the stand under {sk}; PROVISIONAL limit {lf:g} (2026-11-13)")
        return C
    S = P[sk].get("stands") or []
    bad = [f"{r['phase']} {r['fell'] or 'not measured'}" for r in S if not r.get("measured")] + [f"{m} not reached" for m in P[sk].get("missing") or [] if _stand_no(m)]
    why_none = bad_inj or (None if sst else f"the worst of {ns} {sk} stands was not measured ({', '.join(bad) or 'fell / not reached'})")
    dr_each = " / ".join("—" if r.get("drift_m") is None else "%.1f" % (100.0 * r["drift_m"]) for r in S)
    lf_each = " / ".join("—" if r.get("lifts") is None else "%g" % r["lifts"] for r in S)
    add("enc_stand_drift", None if why_none else 100.0 * sst["drift_m"], dc, "<=", True,
        why_none or f"base drift over the {sst['T']:g} s stand under {sk} (the unscaled measured residuals), the WORST of {ns} stands "
                    f"({dr_each} cm), cm; PROVISIONAL limit {dc:g} cm (2026-11-13)")
    add("enc_stand_lifts", None if why_none else sst["lifts"], lf, "<=", True,
        why_none or f"foot lifts in the stand under {sk}, the worst of {ns} stands ({lf_each}); PROVISIONAL limit {lf:g} (2026-11-13)")
    return C


def score_e3(handover, ladders, pushes, homing, enc=None, n_homing=10, cal=None, tilt_fail_deg=None, track_floor=None, tilt_floor_deg=None,
             rev=None, p5_stands=None):
    """EXAM #3 (2026-11-13): the v4 rows on the v4 ramps, re-weighted inside stability / gait / energy / pushes to make room for the
    Test 6 rows (see the block comment above) + the Exam #3 constraints. Keys as score_v4 (fleetdeck) + 'exam' 3, 'enc' (enc_summary +
    the offset pushes paired with Test 2), 'e3' (shares, walks_low, limits), 'score_v4' / 'feasible_v4' (the same record under the v4
    rules, for the comparison with every v4 record). version 5.
    rev 2 (Exam #3 rev 2, 2026-11-13; meta.exam_rev 2): the stress pattern's stand is entered p5_stands times (default E3_REV2_P5_STANDS = 3)
    and the P5 rows / constraints read the WORST of them (enc_summary); version 6, 'rev' 2, e3.rev / e3.p5_stands. Weights, ramps and the
    calibration are rev 1's. rev None / 1 = rev 1 exactly (version 5)."""
    R2 = int(rev or 1) >= 2
    n_exp = (int(p5_stands) if p5_stands is not None else E3_REV2_P5_STANDS) if R2 else None
    C, cver = (cal, "given") if cal is not None else load_cal_e3()
    out = {}
    v4 = score_v4(handover, ladders, pushes, homing, n_homing=n_homing, tilt_fail_deg=tilt_fail_deg, track_floor=track_floor, tilt_floor_deg=tilt_floor_deg)
    # the same rows again on the Exam #3 table (identical ramps unless exam3_calibration.json re-derived a v4 key)
    sc = score_v3(handover, ladders, pushes, homing, n_homing=n_homing, cal=C, tilt_fail_deg=tilt_fail_deg, _v4=True, _out=out)
    es = enc_summary(enc, tilt_fail_deg) if n_exp is None else enc_summary(enc, tilt_fail_deg, stands=n_exp)
    ok_inj = bool(es.get("ran")) and es["injection"]["ok"]
    X = enc_values(es) if ok_inj else {k: None for k in E3_W}
    R = lambda x, k: ramp(x, *C[k])
    M = sc["metrics"]
    for m in M:   # make room: every v4 row of an affected quality keeps its ramp and gives up the same fraction of its weight
        q = m["quality"]
        f = (QUALITIES[q] - E3_SHARE[q]) / QUALITIES[q] if q in E3_SHARE else (EXPECTED_PUSHES_V4 / (EXPECTED_PUSHES_V4 + E3_PUSHES) if q == "pushes" else 1.0)
        if f != 1.0:
            m["w"] = round(m["w"] * f, 4); m["pts"] = round(m["w"] * m["s"], 3)

    def add(q, key, label, x, unit, ck=None, s=None, w=None, prov=False):
        g, b = C[ck] if ck else (1, 0)
        s = ramp(x, g, b) if s is None else clamp01(s)
        w = E3_W[key] if w is None else w
        M.append({"quality": q, "test": "enc", "key": key, "label": label, "value": None if x is None else round(float(x), 5), "unit": unit,
                  "g": g, "b": b, "w": round(w, 4), "s": round(s, 3), "pts": round(w * s, 3), "prov": prov})

    sk = es.get("stress") or "P5"
    void = "" if ok_inj else (" -- NOT MEASURED (" + ("Test 6 did not run" if not es.get("ran") else "injection not verified") + ")")
    ns = int(((es.get("patterns") or {}).get(sk) or {}).get("n_stands") or n_exp or 1)   # rev 2: the P5 rows read the worst of ns stands
    ws = f", worst of {ns} stands" if ns > 1 else ""
    add("stability", "enc_stand_drift", f"Test 6: stand under {sk} (the unscaled measured residuals){ws}: BASE drift in 15 s" + void, X["enc_stand_drift"], "m", ck="stand_drift")
    add("stability", "enc_stand_lifts", f"Test 6: stand under {sk}{ws}: foot lifts" + void, X["enc_stand_lifts"], "", ck="stand_lifts")
    add("stability", "enc_stand_rate", f"Test 6: stand under {sk}{ws}: torso roll/pitch rate rms" + void, X["enc_stand_rate"], "rad/s", ck="stand_rate")
    add("stability", "enc_creep", "Test 6: the worst in-band stand (P1..P4): BASE drift in 15 s" + void, X["enc_creep"], "m", ck="stand_drift")
    add("stability", "enc_walk_tilt", "Test 6: walking at 0.30 under offsets: the largest torso-tilt change vs P0 (P1..P5)" + void, X["enc_walk_tilt"], "deg", ck="enc_walk_tilt", prov=True)
    add("gait", "enc_impact", "Test 6: walking under offsets: touchdown impact p95 change vs P0 (mean P1..P5)" + void, X["enc_impact"], "BW", ck="enc_impact", prov=True)
    add("gait", "enc_smooth", "Test 6: walking under offsets: action roughness change vs P0 (mean P1..P5)" + void, X["enc_smooth"], "rad", ck="enc_smooth", prov=True)
    add("energy", "enc_push_energy", f"Test 6: offset pushes under {sk}: mechanical energy in the 5 s after a kick (held pushes)" + void, X["enc_push_energy"], "J", ck="enc_push_energy", prov=True)
    add("energy", "enc_power", "Test 6: walking under offsets: mechanical power change vs P0 (mean P1..P5)" + void, X["enc_power"], "W", ck="enc_power", prov=True)
    # the offset pushes join the push score: the same per-push formula as Test 2, each of the 56 pushes the same weight
    op = (es.get("pushes") or []) if ok_inj else []
    ps = [sum(w * R(p[k], "push_" + k) for k, w in PUSH_W.items()) if p["held"] else 0.0 for p in op]
    pv = sum(ps) / max(E3_PUSHES, len(ps))
    add("pushes", "enc_pushes", f"Test 6: offset pushes under {sk} held {sum(1 for p in op if p['held'])}/{E3_PUSHES} (1.0 m/s, 4 sides, standing + walking): the Test 2 push formula" + void,
        pv, "score", s=pv, w=QUALITIES["pushes"] * E3_PUSHES / (EXPECTED_PUSHES_V4 + E3_PUSHES), prov=True)
    # pair every offset push with Test 2's same push on perfect encoders (the hard-fail-marked copy)
    t2 = {}
    for s_ in (out.get("P") or {}).get("segments", []):
        p = s_.get("push")
        if p and abs((p.get("level") or 0) - 1.0) < 1e-6: t2[(p.get("mode"), p.get("dir"))] = _push_row(p)
    for p, sc_p in zip(op, ps):
        q = t2.get((p["mode"], p["dir"]))
        p["score"] = round(sc_p, 3)
        p["t2"] = None if not q else {"held": q["held"], "score": round(sum(w * R(q[k], "push_" + k) for k, w in PUSH_W.items()) if q["held"] else 0.0, 3),
                                      **{k: q[k] for k in ("tilt", "steps", "rec", "dist", "energy", "counter", "sep")}}
    # survival: a fall / hard fail under any pattern and an offset push not held are falls like any other
    n6 = (es["falls"] + es["push_falls"]) if es.get("ran") else 0
    for m in M:
        if m["key"] == "falls":
            ran = bool(ladders and ladders.get("segments"))
            m["value"] = m["value"] + n6; m["s"] = round((0.5 ** m["value"]) if ran else 0.0, 3); m["pts"] = round(m["w"] * m["s"], 3)
            m["label"] = "falls (Tests 0, 1, 3, 6 incl. the offset pushes) incl. hard fails"
    if es.get("ran"):
        sc["falls"] = sc["falls"] + [["enc", f"{k}: {_enc_bad(p)}"] for k, p in es["patterns"].items() if p["fell"] or p["missing"]] \
            + [["enc", f"offset push {r['mode']} {r['dir']}: {r.get('why') or 'fell'}"] for r in es.get("pushes") or [] if not r["held"]]
    sc["parts"] = {q: round(sum(m["pts"] for m in M if m["quality"] == q), 1) for q in QUALITIES}
    sc["tests"] = {}
    for m in M:
        sc["tests"][m["test"]] = round(sc["tests"].get(m["test"], 0.0) + m["pts"], 1)
    sc["score"] = round(sum(sc["parts"].values()), 1)
    sc["version"] = 6 if R2 else 5; sc["exam"] = 3
    if R2: sc["rev"] = int(rev)   # Exam #3 rev 2 (2026-11-13): the repeated P5 stand; a rev-1 score has no 'rev' key
    sc["calib"] = cver
    sc["cal"] = {k: list(v) for k, v in C.items()}
    sc["enc"] = es
    sc["constraints"] = constraints_v4(sc, out["L"], out["P"], track_floor, tilt_floor_deg, walk_min=E3_WALK_MIN_CMD)   # walks_*: every rung
    # no_falls stays the v4 definition (Tests 0, 1, 3): the survival row now also counts Test 6, which has its own constraint
    for c in sc["constraints"]:
        if c["name"] == "no_falls":
            v4c = next(x for x in v4["constraints"] if x["name"] == "no_falls"); c.update({k: v4c[k] for k in ("value", "c", "pass", "detail")})
    sc["feasible_v4"] = v4["feasible"]   # the v4 rules on the same run (walks_* from 0.10 m/s / 0.2 rad/s)
    sc["constraints"] = sc["constraints"] + constraints_e3(es, out["L"], metrics=M)
    sc["feasible"] = all(c["pass"] for c in sc["constraints"])
    sc["v4"] = v4["v4"]
    sc["score_v4"] = v4["score"]; sc["parts_v4"] = v4["parts"]
    sc["e3"] = {"share": E3_SHARE, "weights": E3_W, "pushes": [EXPECTED_PUSHES_V4, E3_PUSHES], "walks_low": walks_low(out["L"]) if out.get("L") and out["L"].get("segments") else None,
                "limits": {"enc_stand_drift_cm": ENC_STAND_DRIFT_CM, "enc_stand_lifts": ENC_STAND_LIFTS, "stand_quiet_lifts": STAND_QUIET_LIFTS}, "walk_min_cmd": E3_WALK_MIN_CMD,
                "provisional": ["stand_quiet", "enc_stand_drift", "enc_stand_lifts"] + sorted(PROV_E3), "inj_tol_rad": ENC_INJ_TOL, "base": ENC_BASE, "stress": es.get("stress"),
                "values": X}
    if R2: sc["e3"].update({"rev": int(rev), "p5_stands": ns})
    return sc
