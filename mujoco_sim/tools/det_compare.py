#!/usr/bin/env python3
"""det_compare.py -- are two exam runs of the same policy the same run? (Exam #3 rev 2, 2026-11-13)

    python3 det_compare.py <exam dir A> <exam dir B> <policy name> [--max 20]

1. the records <dir>/exam_<name>.json, compared value by value; ignored: what names the run, not what it measured (meta.stamp / tag /
   instance, the trajectory file names that carry a wall-clock second, the rig's build stamp). Prints the number of differing leaves
   and the first --max paths with both values.
2. the per-frame traces when both runs had EXAM_TRACE=1 (<dir>/trace/sim_<name>.txt = the sim's FNV hashes of ctrl / qpos / qvel per
   physics frame; <dir>/trace/mm_<name>.txt = MovementModule's obs / action / command hashes per policy step): the first differing frame.
   A run that is the same run has 0 differing leaves and identical traces over the common length.
"""
import json
import os
import sys

IGNORE = {"stamp", "tag", "instance", "traj_file", "sim_build", "exam_dir"}


def leaves(x, path=""):
    if isinstance(x, dict):
        for k in sorted(x):
            if k in IGNORE:
                continue
            yield from leaves(x[k], f"{path}.{k}" if path else k)
    elif isinstance(x, list):
        for i, v in enumerate(x):
            yield from leaves(v, f"{path}[{i}]")
        if not x:
            yield path, "[]"
    else:
        yield path, x


def first_trace_diff(pa, pb, ncols):
    """(first differing line index, line A, line B, common lines) over lines with ncols fields (a rig stopped mid-write leaves a stub)"""
    if not (os.path.exists(pa) and os.path.exists(pb)):
        return None
    n = 0
    with open(pa) as fa, open(pb) as fb:
        for la, lb in zip(fa, fb):
            if len(la.split()) < ncols or len(lb.split()) < ncols or not la.endswith("\n") or not lb.endswith("\n"):
                break   # a trace still being written (or cut by the rig stop) ends in a partial line
            if la != lb:
                return n, la.rstrip(), lb.rstrip(), n
            n += 1
    return -1, None, None, n


def main():
    a, b, name = sys.argv[1:4]
    mx = int(sys.argv[sys.argv.index("--max") + 1]) if "--max" in sys.argv else 20
    ra, rb = (json.load(open(os.path.join(d, f"exam_{name}.json"))) for d in (a, b))
    la, lb = dict(leaves(ra)), dict(leaves(rb))
    keys = sorted(set(la) | set(lb))
    diff = [k for k in keys if la.get(k, "<missing>") != lb.get(k, "<missing>")]
    sa, sb = (r["meta"].get("score") or {} for r in (ra, rb))
    print(f"records: {len(keys)} leaves, {len(diff)} differ | score A {sa.get('score')} B {sb.get('score')} | "
          f"rev A {ra['meta'].get('exam_rev')} B {rb['meta'].get('exam_rev')}")
    for k in diff[:mx]:
        print(f"   {k}: A {str(la.get(k, '<missing>'))[:80]} | B {str(lb.get(k, '<missing>'))[:80]}")
    if len(diff) > mx:
        tops = {}
        for k in diff:
            t = k.split(".")[0].split("[")[0]
            tops[t] = tops.get(t, 0) + 1
        print(f"   ... {len(diff) - mx} more; per top-level block: {tops}")
    for kind, nc in (("sim", 8), ("mm", 8)):
        r = first_trace_diff(os.path.join(a, "trace", f"{kind}_{name}.txt"), os.path.join(b, "trace", f"{kind}_{name}.txt"), nc)
        if r is None:
            print(f"{kind} trace: not in both runs")
        elif r[0] < 0:
            print(f"{kind} trace: IDENTICAL over {r[3]} common lines")
        else:
            print(f"{kind} trace: FIRST DIFFERENCE at line {r[0]} of {r[3]}+ common\n   A {r[1]}\n   B {r[2]}")
    return 0 if not diff else 1


if __name__ == "__main__":
    sys.exit(main())
