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


def rung_error(cmd, v):
    """tracking error of one rung: ||v - c|| / max(||c||, 0.1) on (vx, vy, LEVER wz) -- overshoot and off-axis motion count"""
    c = (cmd[0], cmd[1], LEVER * cmd[2]); w = (v[0], v[1], LEVER * v[2])
    nc = (c[0] ** 2 + c[1] ** 2 + c[2] ** 2) ** 0.5
    return ((w[0] - c[0]) ** 2 + (w[1] - c[1]) ** 2 + (w[2] - c[2]) ** 2) ** 0.5 / max(nc, 0.1)


def _axis(cmd):
    k = max(range(3), key=lambda i: abs(cmd[i]))
    return ("vx", "vy", "wz")[k] if abs(cmd[k]) > 1e-9 else None


def ladder_parts(lad):
    """the ladder record split: the 20 s stand, the rungs (by axis), the stops (with the rung before), the falls"""
    segs = [s for s in (lad or {}).get("segments", []) if s.get("name") != "reset"]
    stand = next((s for s in segs if s["name"] == "stand"), None)
    rungs = [s for s in segs if s["name"] not in ("stand", "stop") and s.get("cmd")]
    stops = []
    for i, s in enumerate(segs):
        if s["name"] == "stop":
            prev = segs[i - 1] if i > 0 else None
            stops.append((s, prev["name"] if prev else None, _axis(prev["cmd"]) if prev and prev.get("cmd") else None))
    return stand, rungs, stops


EXPECTED_RUNGS = {"vx": 30, "vy": 12, "wz": 32}
EXPECTED_STOPS = 6
EXPECTED_PUSHES = 24
EXPECTED_SWITCHES = 6


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
    "push_dist": (0.05, 0.5), "push_tilt": (3, 20), "push_rec": (1, 4), "push_steps": (1, 5), "push_foot": (2.0, 6.0),
    "hm_planar": (0.02, 0.10), "hm_head": (1, 5), "hm_sw": (0.02, 0.10), "hm_prog": (0.25, 0.05), "hm_turn": (0.9, 0.3),
}
# PROVISIONAL keys = the spec's first guesses (no physical anchor): the calibration sets them from the population (p10 -> p90);
# every other key is a definition the calibration keeps (and only flags when dead: > 80 % of the samples at one end)
PROV = {"stand_rate", "walk_rate", "ho_jolt", "ho_jump", "impact", "smooth", "smooth_v1", "slip", "steady", "cot", "stand_tau2",
        "push_dist", "push_tilt", "push_rec", "push_foot"}
HO_W = {"lifts": 0.25, "drift": 0.2, "sep": 0.15, "yaw": 0.1, "dip": 0.1, "jolt": 0.1, "jump": 0.05, "wait": 0.05}   # one switch = 1.0
CAL_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results", "exam2_calibration.json")


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


def raw_values(handover, ladders, pushes, homing):
    """every quantity the Exam #2 score ramps, before any ramp: per-policy scalars + per-item lists (stops, switches, rungs, pushes,
    homing trials). score_v2 ramps these; tools/exam_calibrate.py reads their distribution over many policies."""
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
    V["rungs"] = [(_axis(r["cmd"]), None if r.get("fell") or not r.get("v_true") else rung_error(r["cmd"], r["v_true"])) for r in rungs]
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
                            "rec": p.get("recovery_s") if p.get("recovery_s") is not None else 5.0, "steps": max(p.get("steps") or 0, 0), "foot": p.get("foot_peak_bw")})
    V["homing"] = []
    for r in homing or []:
        if r.get("result") != "ARRIVED":
            V["homing"].append({"ok": False}); continue
        e, sw_ = r.get("final_err") or {}, r.get("err_at_switch") or {}
        V["homing"].append({"ok": True, "planar": e.get("planar_m"), "head": abs(e["heading_deg"]) if e.get("heading_deg") is not None else None,
                            "sw": sw_.get("planar_m"), "prog": (r.get("straight_m") or 0.0) / r["t_switch"] if r.get("t_switch") else None,
                            "turn": r.get("turn_gain"), "rescue": r.get("n_rescue") or 0})
    return V


def score_v2(handover, ladders, pushes, homing, n_homing=10, cal=None):
    """handover: list of Test 0 records; ladders / pushes: the Test 1 / Test 2 records (or None); homing: the Test 3 trial records.
    cal: a (g, b) table (default: load_cal() = CAL_DEFAULT + results/exam2_calibration.json)"""
    C, cver = (cal, "given") if cal is not None else load_cal()
    R = lambda x, k: ramp(x, *C[k])
    V = raw_values(handover, ladders, pushes, homing)
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
    add("stability", "ladders", "stand_drift", "20 s stand: drift", V["stand_drift"], "m", 1.5)
    add("stability", "ladders", "stand_yaw", "20 s stand: heading change", V["stand_yaw"], "deg", 1.0)
    add("stability", "ladders", "stand_lifts", "20 s stand: foot lifts", V["stand_lifts"], "", 1.5)
    add("stability", "ladders", "stand_rate", "20 s stand: torso roll/pitch rate rms", V["stand_rate"], "rad/s", 1.0, prov=True)
    add("stability", "ladders", "stand_sep_trend", "20 s stand: feet separation trend", V["stand_sep_trend"], "m", 0.5)
    add("stability", "ladders", "stand_stagger", "20 s stand: fore-aft stagger", V["stand_stagger"], "m", 0.5)
    add("stability", "ladders", "stand_toe", "20 s stand: toe angle (mean |L|, |R|)", V["stand_toe"], "deg", 0.5)
    add("stability", "ladders", "stand_jitter", "20 s stand: knee jitter", V["stand_jitter"], "deg/sample", 0.5)
    ss = [mean([R(x["settle"], "stop_settle"), R(x["steps"], "stop_steps"), R(x["resid"], "stop_resid"),
                R(x["over_deg"], "stop_over_deg") if x["ax"] == "wz" else R(x["over_m"], "stop_over_m")]) if x["ok"] else 0.0 for x in V["stops"]]
    sv = mean(ss + [0.0] * max(0, EXPECTED_STOPS - len(ss)), 0.0)
    add("stability", "ladders", "stops", f"stops from the ladder tops ({len(ss)}/{EXPECTED_STOPS}): settle time, steps, residual drift, overshoot",
        sv, "score", 5.0, s=sv, ck="none")
    add("stability", "ladders", "walk_tilt", "walking (|v| <= 0.5): torso tilt", V["walk_tilt"], "deg", 2.5)
    add("stability", "ladders", "walk_rate", "walking (|v| <= 0.5): torso roll/pitch rate rms", V["walk_rate"], "rad/s", 2.5, prov=True)
    add("stability", "ladders", "walk_sag", "walking: height sag vs the balance policy's stand", V["walk_sag"], "m", 1.0)
    sw = [sum(w * R(x[k], "ho_" + k) for k, w in HO_W.items()) if x["ok"] else 0.0 for x in V["switches"]]
    hs = mean(sw + [0.0] * max(0, EXPECTED_SWITCHES - len(sw)), 0.0)
    add("stability", "handover", "handover", f"handover switches ({len(sw)}/{EXPECTED_SWITCHES}): steps, drift, inward draw, turn, dip, jolt, target jump, wait",
        hs, "score", 12.0, s=hs, ck="none")
    # ── tracking 20 = vx 8 + vy 4 + wz 8: per rung ramp(e; rung_e); a rung not run (fall) = 0
    for ax, w in (("vx", 8.0), ("vy", 4.0), ("wz", 8.0)):
        sc = [0.0 if e is None else R(e, "rung_e") for a, e in V["rungs"] if a == ax]
        s = sum(sc) / max(EXPECTED_RUNGS[ax], len(sc))
        add("tracking", "ladders", f"track_{ax}", f"{ax} ladder: mean rung score ({len(sc)}/{EXPECTED_RUNGS[ax]} rungs), e = |v - c| / max(|c|, 0.1)", s, "score", w, s=s, ck="none")
    # ── gait & body care 10 = impact 3 + action smoothness 3 + sliding 2 + swing 1 + speed steadiness 1
    add("gait", "ladders", "impact", "foot touchdown impact p95 / body weight", V["impact"], "BW", 3.0, prov=True)
    if V["smooth_kind"] == "acc":   # the SECOND difference of the joint targets (chatter); older records carry only the first difference
        add("gait", "ladders", "smooth", "action roughness: rms of the joint-target 2nd difference (max joint)", V["smooth"], "rad", 3.0, prov=True)
    else:
        add("gait", "ladders", "smooth", "action step rms (joint targets, max joint, per policy step)", V["smooth"], "rad", 3.0, prov=True, ck="smooth_v1")
    add("gait", "ladders", "pivot", "pivot index of a loaded foot (linear rungs)", V["pivot"], "rad/s", 1.0)
    add("gait", "ladders", "slip", "slip speed of a loaded foot (linear rungs)", V["slip"], "m/s", 1.0, prov=True)
    add("gait", "ladders", "swing", "swing height inside 3-10 cm (share of rungs >= 0.2 m/s)", V["swing_share"], "share", 1.0, s=V["swing_share"] or 0.0, ck="none")
    add("gait", "ladders", "steady", "speed steadiness: planar v std within a rung", V["steady"], "m/s", 1.0, prov=True)
    # ── energy 10 = cost of transport at vx 0.3 / 0.5 (6) + standing tau^2 (4)
    add("energy", "ladders", "cot", "cost of transport at vx 0.3 / 0.5 (mechanical |tau qdot|)", V["cot"], "", 6.0, prov=True)
    add("energy", "ladders", "stand_tau2", "20 s stand: mean sum tau^2", V["stand_tau2"], "N^2m^2", 4.0, prov=True)
    # ── pushes 10: per push (24) held -> mean of 5 ramps, a fall -> 0
    ps = [mean([R(p["dist"], "push_dist"), R(p["tilt"], "push_tilt"), R(p["rec"], "push_rec"), R(p["steps"], "push_steps"), R(p["foot"], "push_foot")])
          if p["held"] else 0.0 for p in V["pushes"]]
    held = sum(1 for p in V["pushes"] if p["held"])
    pv = sum(ps) / max(EXPECTED_PUSHES, len(ps))
    add("pushes", "pushes", "pushes", f"pushes held {held}/{EXPECTED_PUSHES}: distance, tilt, recovery time, steps, foot peak", pv, "score", 10.0, prov=True, s=pv, ck="none")
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
