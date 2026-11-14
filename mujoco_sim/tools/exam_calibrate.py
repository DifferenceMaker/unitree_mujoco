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
--apply: re-score the given records in place with the table in results/exam2_calibration.json (written by --write) (the raw data in the records is untouched; scores are recomputable).
  Since v4 (2026-11-09) --apply re-scores each record with ITS OWN score version (meta.score.version: v2 -> score_v2, v3 -> score_v3, v4 ->
  score_v4 with the v4 table; version 5 = Exam #3 (2026-11-13) -> score_e3 with the record's own "enc" block and the Exam #3 table); before, every record was re-scored with score_v2.
--v4 (2026-11-09, Exam #2 v4): the rung errors use the v4 floor (exam_score.E_FLOOR_V4 = 0.05), the CURRENT table = the v2 calibration
  file (results/exam2_calibration.json; a key it lacks = CAL_DEFAULT) and --write goes to results/exam2_calibration_v4.json (the v2 file
  that scores the v2 / v3 records stays intact). --keys k1,k2 (fnmatch patterns): only these keys are re-derived from the given records
  (the keys whose distribution v4 changes: the pushes with the diagonals, the push share of the all-phase energy, ...); every other key is
  inherited from the v2 table. The population for a v4 key must be v4 records (a 4-direction push test has no diagonals).
--e3 (2026-11-13, Exam #3): the CURRENT table = the v4 calibration + the Test 6 keys (exam_score.CAL_E3_DEFAULT: enc_walk_tilt, enc_impact,
  enc_smooth, enc_push_energy, enc_power -- all provisional), --write goes to results/exam3_calibration.json (a complete table; the v4
  file stays intact) and by default ONLY the enc_* keys are re-derived (--keys overrides): the v4 rows keep their v4 ramps, so a row's
  score s is the same number in a v4 and an Exam #3 reading of one run. The population must be Exam #3 records (they carry "enc").
--rev N (2026-11-13, Exam #3 rev 2): keep only Exam #3 records of revision N (meta.exam_rev; none = rev 1). Without it the revision counts
  are printed (--e3) and a mixed population is flagged. --apply re-scores a version-6 record (rev 2) with score_e3(rev=meta.exam_rev,
  p5_stands=meta.det.p5_stands); a version-5 record (rev 1) exactly as before. Version 7 (rev 3, 2026-11-14: the rev-3 ladder, vx to 0.80,
  wz +-0.60) the same way, score_e3(rev=3). Rev 3's shorter ladder moves the rung-pooled samples (impact, smooth, slip, steady, ...): pass
  --rev 3 to calibrate on rev-3 records only."""
import argparse, glob, importlib.util, json, os, statistics as st, time

TOOLS = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("exam_score", os.path.join(TOOLS, "exam_score.py"))
es = importlib.util.module_from_spec(spec); spec.loader.exec_module(es)

MIN_SPAN = 0.5   # a ramp spans at least half the population median: differences under +-25 % of the typical value cannot swing it end to end
SCALAR = ["stand_drift", "stand_yaw", "stand_lifts", "stand_rate", "stand_sep_trend", "stand_stagger", "stand_toe", "stand_jitter", "stand_tau2",
          "walk_tilt", "walk_rate", "walk_sag", "impact", "smooth", "pivot", "slip", "steady", "cot",
          "arm_sway", "en_all_tau2", "en_all_power",   # + 2026-11-01
          "stand_sep_dev_max", "stand_sep_dev_end", "stand_stag_dev_max", "stand_stag_dev_end", "arm_dev_mean", "arm_dev_max", "sc_dur"]   # + v3 (sampled since v4)


def pct(v, q):
    v = sorted(v); k = (len(v) - 1) * q; i = int(k); f = k - i
    return v[i] if i + 1 >= len(v) else v[i] * (1 - f) + v[i + 1] * f


def samples(recs, e_floor=None):
    S = {}
    add = lambda k, x: S.setdefault(k, []).append(float(x)) if x is not None else None
    for r in recs:
        V = es.raw_values(r.get("handover"), r.get("ladders"), r.get("pushes"), r.get("homing"), e_floor=es.E_FLOOR if e_floor is None else e_floor)
        for k in SCALAR:
            if k == "smooth" and V.get("smooth_kind") != "acc":
                add("smooth_v1", V.get("smooth")); continue
            add(k, V.get(k))
            if k.startswith("stand_"):   # v3 twins (2026-11-03): the stand after walking (wstand_*) scores on the stand's keys -> pooled (since v4)
                add(k, V.get("w" + k))
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
                for k in ("dist", "tilt", "rec", "steps", "foot", "counter", "sway", "sep", "arm", "energy"): add("push_" + k, p.get(k))
        for x in V["homing"]:
            if x["ok"]:
                for k in ("planar", "head", "sw", "prog", "turn"): add("hm_" + k, x[k])
        if r.get("enc"):   # Exam #3 (2026-11-13): the Test 6 quantities with a ramp of their own, one value per record
            X = es.enc_values(es.enc_summary(r["enc"]))
            for k in es.CAL_E3_DEFAULT: add(k, X.get(k))
    return S


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("records", nargs="+"); ap.add_argument("--version", default=time.strftime("v1-%Y-%m-%d"))
    ap.add_argument("--write", action="store_true"); ap.add_argument("--apply", action="store_true")
    ap.add_argument("--v4", action="store_true", help="Exam #2 v4: e floor 0.05, base = the v2 table, --write -> results/exam2_calibration_v4.json")
    ap.add_argument("--keys", default="", help="--v4 / --e3: comma-separated fnmatch patterns of the keys to re-derive (default: all keys; --e3: enc_*)")
    ap.add_argument("--e3", action="store_true", help="Exam #3: base = the v4 table + the Test 6 keys, --write -> results/exam3_calibration.json, default keys enc_*")
    ap.add_argument("--rev", type=int, default=None, help="Exam #3 revision filter (2026-11-13): only Exam #3 records of this rev (1 = no meta.exam_rev, 2 = "
                    "the repeated P5 stand) enter the population / --apply; without it the rev counts are printed and a mixed population is flagged")
    a = ap.parse_args()
    if a.e3 and not a.keys: a.keys = "enc_*"
    import fnmatch
    pats = [k.strip() for k in a.keys.split(",") if k.strip()]
    pick = (lambda k: any(fnmatch.fnmatch(k, p_) for p_ in pats)) if pats else (lambda k: True)
    e_floor = es.E_FLOOR_V4 if (a.v4 or a.e3) else es.E_FLOOR
    out_file = es.CAL_FILE_E3 if a.e3 else (es.CAL_FILE_V4 if a.v4 else es.CAL_FILE)
    prov = es.PROV | es.PROV_E3 if a.e3 else es.PROV
    base_ver = "CAL_DEFAULT"
    files = [f for p in a.records for f in sorted(glob.glob(p))]
    recs = []
    for f in files:
        r = json.load(open(f))
        if int((r.get("meta") or {}).get("exam_version") or 1) >= 2 and r.get("ladders"):
            r["_file"] = f; recs.append(r)
    # Exam #3 revisions (2026-11-13): rev 2 enters the P5 stand three times -- never pool the revisions silently
    rev_of = lambda r: int(r["meta"].get("exam_rev") or 1) if int(r["meta"].get("exam_version") or 1) == 3 else None
    if a.rev is not None:
        n0 = len(recs); recs = [r for r in recs if rev_of(r) == a.rev]
        print(f"--rev {a.rev}: {len(recs)} of {n0} records kept (Exam #3 rev {a.rev} only)\n")
    revc = {}
    for r in recs:
        if rev_of(r) is not None: revc[rev_of(r)] = revc.get(rev_of(r), 0) + 1
    if a.e3 or any(k >= 2 for k in revc):
        print("Exam #3 records by revision: " + (", ".join(f"rev {k}: {v}" for k, v in sorted(revc.items())) or "none")
              + ("  -- WARNING: MIXED revisions pooled (pass --rev N to keep one)" if len(revc) > 1 else "") + "\n")
    S = samples(recs, e_floor)
    cur = dict(es.CAL_DEFAULT)
    if a.v4:   # the v4 table starts from the v2 calibration (the keys v4 does not touch keep their v2 ends)
        cur, base_ver = es.load_cal(es.CAL_FILE)
    if a.e3:   # the Exam #3 table starts from the v4 calibration + the Test 6 keys' first values
        cur, base_ver = es.load_cal_v4(); cur = dict(cur, **es.CAL_E3_DEFAULT)
    new = dict(cur)
    rows = []
    for k, (g, b) in cur.items():
        if not pick(k):
            rows.append((k, g, b, None, None, None, len(S.get(k) or []), None, None, g, b, f"inherited ({base_ver})")); continue
        v = S.get(k) or []
        if len(v) < 3:
            rows.append((k, g, b, None, None, None, len(v), None, None, g, b, "too few samples")); continue
        p10, p50, p90 = pct(v, 0.1), pct(v, 0.5), pct(v, 0.9)
        full = sum(1 for x in v if es.ramp(x, g, b) >= 0.999) / len(v); zero = sum(1 for x in v if es.ramp(x, g, b) <= 0.001) / len(v)
        dead = f"DEAD: {100 * full:.0f} % at full marks" if full > 0.8 else (f"DEAD: {100 * zero:.0f} % at zero" if zero > 0.8 else "")
        if k not in prov:
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
    print(f"## Exam #{3 if a.e3 else 2} calibration {a.version}: {len(recs)} policies ({len(files)} files){' -- %s: e floor %g, base %s, keys %s' % ('e3' if a.e3 else 'v4', e_floor, base_ver, a.keys or 'all') if (a.v4 or a.e3) else ''}\n")
    print("| key | current g → b | n | p10 / p50 / p90 | at full / at zero (current) | proposed g → b | note |")
    print("|---|---|---|---|---|---|---|")
    for k, g, b, p10, p50, p90, n, full, zero, ng, nb, why in rows:
        print(f"| {k} | {f(g)} → {f(b)} | {n} | {f(p10)} / {f(p50)} / {f(p90)} | {'–' if full is None else f'{100 * full:.0f} % / {100 * zero:.0f} %'} | {f(ng)} → {f(nb)} | {why} |")
    if a.write:   # --apply alone re-scores with the table already in the file (never overwrites it with a subset's proposal)
        out = {"version": a.version, "date": time.strftime("%Y-%m-%d %H:%M"), "source": f"{len(recs)} Exam #2 records",
               "records": [os.path.relpath(r["_file"], os.path.expanduser("~/Projects/robot_projects/repos")) for r in recs],
               "rule": f"provisional keys (exam_score.PROV) g = p10, b = p90 of the population (oriented), span >= {MIN_SPAN} x median; the rest kept",
               "cal": {k: list(v) for k, v in new.items()}}
        if a.e3: out["source"] = f"{len(recs)} Exam #3 records"; out["exam3_revs"] = {f"rev {k}": v for k, v in sorted(revc.items())}
        if a.v4 or a.e3:
            out.update({"base": base_ver, "e_floor": e_floor, "keys": a.keys or "all",
                        "recalibrated": sorted(k for k, *_r in rows if pick(k)),
                        "changed": {k: [list(cur[k]), list(new[k])] for k in new if pick(k) and tuple(new[k]) != tuple(cur[k])}})
        json.dump(out, open(out_file, "w"), indent=1)
        print(f"\n-> {out_file}")
    if a.apply:
        print("\n| policy | record | version | before | after |\n|---|---|---|---|---|")
        for r in recs:
            before = (r["meta"].get("score") or {}).get("score")
            ver = int((r["meta"].get("score") or {}).get("version") or 2)   # each record keeps its own score version (v4: the v4 table)
            fn = {2: es.score_v2, 3: es.score_v3}.get(ver, es.score_v4)
            sc = fn(r.get("handover"), r.get("ladders"), r.get("pushes"), r.get("homing"), n_homing=int(r["meta"].get("n") or 10))
            if ver in (5, 6, 7):   # Exam #3 (2026-11-13, meta.score.version 5; rev 2 = version 6; rev 3 = 7): score_e3 with the record's own "enc" (without this it would drop to v4)
                rv = int(r["meta"].get("exam_rev") or {6: 2, 7: 3}.get(ver, 1))   # rev 2: the worst of the repeated P5 stands; rev 3: + the rev-3 ladder
                kw = {"rev": rv, "p5_stands": (r["meta"].get("det") or {}).get("p5_stands")} if rv >= 2 else {}
                sc = es.score_e3(r.get("handover"), r.get("ladders"), r.get("pushes"), r.get("homing"), enc=r.get("enc"), n_homing=int(r["meta"].get("n") or 10), **kw)
            r["meta"]["score"] = sc
            path = r.pop("_file")
            json.dump(r, open(path, "w"), separators=(",", ":"))
            print(f"| {r['meta'].get('walk')} | {os.path.basename(path)} | v{sc['version']} | {before} | {sc['score']} |")


if __name__ == "__main__":
    main()
