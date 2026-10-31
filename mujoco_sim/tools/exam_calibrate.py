#!/usr/bin/env python3
"""exam_calibrate.py <record.json> ... [--version v1] [--write] [--apply] -- the Exam #2 calibration (operator 2026-10-31).

Every Exam #2 metric scores through a ramp s = (b - x) / (b - g) between a good value g (full marks) and a bad value b (zero). The
first g / b were guesses; a guess that every policy beats (or fails) gives everyone the same points and measures nothing. This tool
reads the RAW values the score ramps (exam_score.raw_values) from many exam records and shows, per ramp key, the population's
10th / 50th / 90th percentile and how many samples sit at full marks / at zero under the current table.

Rule: a PROVISIONAL key (exam_score.PROV: the spec's first guesses, no physical anchor) gets g = the population's 10th percentile
and b = its 90th percentile (oriented: higher-is-better keys the other way round), so its ramp spans where the policies actually
differ: the best tenth earns full marks, the worst tenth zero -- but never narrower than MIN_SPAN x the median (widened around the
same centre): in a tight population (walking torso rate 0.17-0.21 rad/s) p10 -> p90 would turn a 20 % difference into the whole
weight. Every other key is a definition (the operator's tolerances, physical
meaning: falls, steps, drift, the rung error ramp, the homing placement, ...) and is kept; the table flags it DEAD when > 80 % of
its samples sit at one end (the operator decides). Per-item keys (stops, switches, pushes, homing trials) use the pooled items;
per-policy keys one value per policy.
--write: results/exam2_calibration.json (exam_score.load_cal reads it) -- the version travels in every score (meta.score.calib).
--apply: re-score the given records in place with the table in results/exam2_calibration.json (written by --write) (the raw data in the records is untouched; scores are recomputable)."""
import argparse, glob, importlib.util, json, os, statistics as st, time

TOOLS = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("exam_score", os.path.join(TOOLS, "exam_score.py"))
es = importlib.util.module_from_spec(spec); spec.loader.exec_module(es)

MIN_SPAN = 0.5   # a ramp spans at least half the population median: differences under +-25 % of the typical value cannot swing it end to end
SCALAR = ["stand_drift", "stand_yaw", "stand_lifts", "stand_rate", "stand_sep_trend", "stand_stagger", "stand_toe", "stand_jitter", "stand_tau2",
          "walk_tilt", "walk_rate", "walk_sag", "impact", "smooth", "pivot", "slip", "steady", "cot"]


def pct(v, q):
    v = sorted(v); k = (len(v) - 1) * q; i = int(k); f = k - i
    return v[i] if i + 1 >= len(v) else v[i] * (1 - f) + v[i + 1] * f


def samples(recs):
    S = {}
    add = lambda k, x: S.setdefault(k, []).append(float(x)) if x is not None else None
    for r in recs:
        V = es.raw_values(r.get("handover"), r.get("ladders"), r.get("pushes"), r.get("homing"))
        for k in SCALAR:
            if k == "smooth" and V.get("smooth_kind") != "acc":
                add("smooth_v1", V.get("smooth")); continue
            add(k, V.get(k))
        for x in V["stops"]:
            if not x["ok"]: continue
            add("stop_settle", x["settle"]); add("stop_steps", x["steps"]); add("stop_resid", x["resid"])
            add("stop_over_deg", x["over_deg"]) if x["ax"] == "wz" else add("stop_over_m", x["over_m"])
        for x in V["switches"]:
            if x["ok"]:
                for k in es.HO_W: add("ho_" + k, x[k])
        for a, e in V["rungs"]:
            add("rung_e", e)
        for p in V["pushes"]:
            if p["held"]:
                for k in ("dist", "tilt", "rec", "steps", "foot"): add("push_" + k, p[k])
        for x in V["homing"]:
            if x["ok"]:
                for k in ("planar", "head", "sw", "prog", "turn"): add("hm_" + k, x[k])
    return S


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("records", nargs="+"); ap.add_argument("--version", default=time.strftime("v1-%Y-%m-%d"))
    ap.add_argument("--write", action="store_true"); ap.add_argument("--apply", action="store_true")
    a = ap.parse_args()
    files = [f for p in a.records for f in sorted(glob.glob(p))]
    recs = []
    for f in files:
        r = json.load(open(f))
        if int((r.get("meta") or {}).get("exam_version") or 1) >= 2 and r.get("ladders"):
            r["_file"] = f; recs.append(r)
    S = samples(recs)
    cur = dict(es.CAL_DEFAULT)
    new = dict(cur)
    rows = []
    for k, (g, b) in cur.items():
        v = S.get(k) or []
        if len(v) < 3:
            rows.append((k, g, b, None, None, None, len(v), None, None, g, b, "too few samples")); continue
        p10, p50, p90 = pct(v, 0.1), pct(v, 0.5), pct(v, 0.9)
        full = sum(1 for x in v if es.ramp(x, g, b) >= 0.999) / len(v); zero = sum(1 for x in v if es.ramp(x, g, b) <= 0.001) / len(v)
        dead = f"DEAD: {100 * full:.0f} % at full marks" if full > 0.8 else (f"DEAD: {100 * zero:.0f} % at zero" if zero > 0.8 else "")
        if k not in es.PROV:
            ng, nb, why = g, b, "definition kept" + (f" -- {dead}" if dead else "")
        elif p90 - p10 < 1e-9:
            ng, nb, why = g, b, "no spread"
        else:
            lo, hi, note = p10, p90, "p10 / p90"
            if hi - lo < MIN_SPAN * abs(p50):
                c = 0.5 * (lo + hi); lo, hi, note = max(0.0, c - 0.5 * MIN_SPAN * abs(p50)), c + 0.5 * MIN_SPAN * abs(p50), f"p10 / p90 widened to {MIN_SPAN:g} x median"
            ng, nb = (lo, hi) if g < b else (hi, lo)
            why = f"provisional -> {note}" + (f" (was {dead})" if dead else "")
        new[k] = (round(ng, 6), round(nb, 6))
        rows.append((k, g, b, p10, p50, p90, len(v), full, zero, new[k][0], new[k][1], why))
    f = lambda x: "–" if x is None else f"{x:.4g}"
    print(f"## Exam #2 calibration {a.version}: {len(recs)} policies ({len(files)} files)\n")
    print("| key | current g → b | n | p10 / p50 / p90 | at full / at zero (current) | proposed g → b | note |")
    print("|---|---|---|---|---|---|---|")
    for k, g, b, p10, p50, p90, n, full, zero, ng, nb, why in rows:
        print(f"| {k} | {f(g)} → {f(b)} | {n} | {f(p10)} / {f(p50)} / {f(p90)} | {'–' if full is None else f'{100 * full:.0f} % / {100 * zero:.0f} %'} | {f(ng)} → {f(nb)} | {why} |")
    if a.write:   # --apply alone re-scores with the table already in the file (never overwrites it with a subset's proposal)
        out = {"version": a.version, "date": time.strftime("%Y-%m-%d %H:%M"), "source": f"{len(recs)} Exam #2 records",
               "records": [os.path.relpath(r["_file"], os.path.expanduser("~/Projects/robot_projects/repos")) for r in recs],
               "rule": f"provisional keys (exam_score.PROV) g = p10, b = p90 of the population (oriented), span >= {MIN_SPAN} x median; the rest kept",
               "cal": {k: list(v) for k, v in new.items()}}
        json.dump(out, open(es.CAL_FILE, "w"), indent=1)
        print(f"\n-> {es.CAL_FILE}")
    if a.apply:
        cal, ver = es.load_cal()
        print("\n| policy | record | before | after |\n|---|---|---|---|")
        for r in recs:
            before = (r["meta"].get("score") or {}).get("score")
            sc = es.score_v2(r.get("handover"), r.get("ladders"), r.get("pushes"), r.get("homing"), n_homing=int(r["meta"].get("n") or 10))
            r["meta"]["score"] = sc
            path = r.pop("_file")
            json.dump(r, open(path, "w"), separators=(",", ":"))
            print(f"| {r['meta'].get('walk')} | {os.path.basename(path)} | {before} | {sc['score']} |")


if __name__ == "__main__":
    main()
