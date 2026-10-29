"""exam_score.py -- THE score of the standardized exam, in one place (used by exam_report.py and exam_record.py).

Score (0-100, every term bounded; a ranking aid, the weights are a choice -- results/EXAM_METRICS.md):
  tracking  40  mean over the staircase's command segments of min(gain, 1) (gain = achieved / commanded, sim truth; a fall = 0)
  placement 20  1 - min(settled planar mean / 0.10 m, 1)                  (homing)
  stillness 20  half: 1 - min(stand-10 s drift / 0.10 m, 1)  (staircase)   half: 1 - min(|stand after the switch, lateral| / 0.10 m, 1) (homing)
  turn      10  (1 - min(pivot / 0.5 rad/s, 1)) x min(mean |gain| at 0.8, 1)  (slide = 0, no turn = 0, stepping turn ~1)
  survival  10  homing arrivals / n, times 0 if the staircase fell
"""
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
