#!/usr/bin/env python3
"""homing_plot.py -- the plots of a homing batch (tools/homing_trials.sh):
  <out>_map.png       top-down map: every trial's trajectory coloured by phase, the start pose (dot + heading arrow + trial number),
                      the waypoints (ring = the pose where a phase began), the settled pose (orange arrow), the pre-pose, the stop
                      pose and the table; right: the stop zone -- the robot's offset from the stop pose at the switch (open) and
                      after the settle (filled), with the tolerance boxes.
  <out>_velocity.png  one row per trial: commanded (blue, steps) vs measured (orange; the filtered pose differenced) vs the sim's
                      true base velocity (grey) for vx, vy, wz, phase bands behind, the balance switch as a dashed line.
  <out>_errors.png    the robot's offset from the stop pose (fwd, left, heading) vs time aligned on the balance switch (t = 0),
                      all trials thin + the mean bold; the +-3 cm / +-2 deg tolerance bands.
    python3 homing_plot.py --jsonl logs/homing_trials.jsonl --n 10 --traj logs/homing_traj --out logs/homing_plots/homing_<stamp>
Sign convention in the plots: the ROBOT's offset (fwd + = past the stop towards the table, left +, heading + = turned left); the
jsonl / CSV carry the commander's to-go errors (opposite sign). Needs matplotlib (python3 or the isaacsim conda env)."""
import argparse
import csv
import json
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D

# the reference categorical palette (dataviz skill, light surface); hues by role in a fixed order
C = {"FAR": "#2a78d6", "ALIGN": "#eda100", "APPROACH": "#1baf7a", "PULSE": "#e87ba4", "HOLD": "#6250d6", "SWITCHING": "#6250d6",
     "SETTLE": "#9a9893", "cmd": "#2a78d6", "meas": "#eb6834", "truth": "#9a9893", "table": "#e5e3dd", "ink": "#0b0b0b", "muted": "#52514e",
     "grid": "#e8e6e1"}
PHASES = ["FAR", "ALIGN", "APPROACH", "PULSE", "HOLD", "SWITCHING", "SETTLE"]


def load(args):
    rows = [json.loads(l) for l in open(args.jsonl) if l.strip()][-args.n:]
    trajs = []
    for r in rows:
        d = None
        tf = r.get("traj_file")
        path = os.path.join(args.traj, os.path.basename(tf)) if tf else ""
        if tf and os.path.exists(path):
            with open(path) as f:
                rd = list(csv.DictReader(f))
            if rd:
                d = {k: np.array([row[k] for row in rd], dtype=(object if k == "phase" else float)) for k in rd[0]}
        trajs.append(d)
    return rows, trajs


def runs(ph):
    """contiguous phase runs of a trajectory: (phase, i0, i1 inclusive)"""
    out, i0 = [], 0
    for i in range(1, len(ph) + 1):
        if i == len(ph) or ph[i] != ph[i0]:
            out.append((ph[i0], i0, i - 1)); i0 = i
    return out


def style(ax):
    ax.grid(True, color=C["grid"], lw=0.6); ax.set_axisbelow(True)
    for s in ("top", "right"): ax.spines[s].set_visible(False)
    for s in ("left", "bottom"): ax.spines[s].set_color(C["muted"])
    ax.tick_params(colors=C["muted"], labelsize=8)


def offset(err):
    """to-go error dict -> the robot's offset (fwd m, left m, heading deg)"""
    return (-err["along_m"], -err["lateral_m"], -err["heading_deg"]) if err else (None, None, None)


def plot_map(rows, trajs, out, title):
    fig, (ax, az) = plt.subplots(1, 2, figsize=(15, 7.2), gridspec_kw={"width_ratios": [1.7, 1]})
    g = rows[0]["goal"]; psi = math.radians(g[2]); c, s = math.cos(psi), math.sin(psi)
    pre, stop = rows[0]["pre_pose"], rows[0]["stop_pose"]
    # the table (desk scene: top 0.80 m deep x 1.20 m wide behind the near edge centre = the goal)
    ax.add_patch(Rectangle((0, -0.6), 0.8, 1.2, transform=Affine2D().rotate(psi).translate(g[0], g[1]) + ax.transData, fc=C["table"], ec=C["muted"], lw=1))
    ax.text(g[0] + 0.4 * c, g[1] + 0.4 * s, "table", ha="center", va="center", color=C["muted"], fontsize=9)
    ax.plot([pre[0], stop[0]], [pre[1], stop[1]], ls="--", color=C["muted"], lw=0.8)
    ax.plot(pre[0], pre[1], "x", color=C["muted"], ms=8, mew=1.5)
    ax.plot(stop[0], stop[1], "+", color=C["ink"], ms=10, mew=1.5)
    ax.annotate("pre-pose", (pre[0], pre[1]), xytext=(0, -12), textcoords="offset points", ha="center", fontsize=8, color=C["muted"])
    ax.annotate("stop", (stop[0], stop[1]), xytext=(0, 8), textcoords="offset points", ha="center", fontsize=8, color=C["ink"])
    for r, d in zip(rows, trajs):
        if d is None:
            continue
        fx, fy, ph = d["fx"], d["fy"], d["phase"]
        for p, i0, i1 in runs(ph):
            ax.plot(fx[i0:i1 + 2], fy[i0:i1 + 2], color=C.get(p, C["muted"]), lw=1.4, alpha=0.9, solid_capstyle="round")
        st = r.get("start")
        if st:
            yaw = math.radians(st[2])
            ax.plot(st[0], st[1], "o", color=C["ink"], ms=5)
            ax.annotate("", xy=(st[0] + 0.35 * math.cos(yaw), st[1] + 0.35 * math.sin(yaw)), xytext=(st[0], st[1]), arrowprops=dict(arrowstyle="->", color=C["ink"], lw=1))
            ax.annotate(str(r["trial"]), (st[0], st[1]), xytext=(5, 5), textcoords="offset points", fontsize=9, color=C["ink"], fontweight="bold")
        for w in r.get("waypoints") or []:
            if w.get("pose") and w["phase"] in ("ALIGN", "APPROACH", "HOLD"):
                ax.plot(w["pose"][0], w["pose"][1], "o", mfc="white", mec=C[w["phase"]], ms=5, mew=1.2)
        fp = r.get("final_pose")
        if fp:
            yaw = math.radians(fp[2])
            ax.annotate("", xy=(fp[0] + 0.2 * math.cos(yaw), fp[1] + 0.2 * math.sin(yaw)), xytext=(fp[0], fp[1]), arrowprops=dict(arrowstyle="-|>", color=C["meas"], lw=1.2))
    xs = [g[0] + 0.8 * c - 0.6 * s, g[0] + 0.8 * c + 0.6 * s, g[0] - 0.6 * s, g[0] + 0.6 * s, pre[0]]
    ys = [g[1] + 0.8 * s + 0.6 * c, g[1] + 0.8 * s - 0.6 * c, g[1] + 0.6 * c, g[1] - 0.6 * c, pre[1]]
    for r, d in zip(rows, trajs):
        if r.get("start"): xs.append(r["start"][0]); ys.append(r["start"][1])
        if d is not None: xs += [float(d["fx"].min()), float(d["fx"].max())]; ys += [float(d["fy"].min()), float(d["fy"].max())]
    ax.set_xlim(min(xs) - 0.4, max(xs) + 0.4); ax.set_ylim(min(ys) - 0.4, max(ys) + 0.4)
    ax.set_aspect("equal"); style(ax)
    ax.set_xlabel("x [m]", color=C["muted"]); ax.set_ylabel("y [m]", color=C["muted"])
    ax.set_title("trajectories by phase (ring = phase change; orange arrow = settled pose)", fontsize=10, color=C["ink"], loc="left")
    handles = [Line2D([], [], color=C[p], lw=2, label=p) for p in ("FAR", "ALIGN", "APPROACH", "PULSE", "HOLD")]
    handles += [Line2D([], [], marker="o", color=C["ink"], ls="", ms=5, label="start (+ heading)"),
                Line2D([], [], marker="o", mfc="white", mec=C["muted"], ls="", ms=5, label="waypoint (phase began)"),
                Line2D([], [], marker=">", color=C["meas"], ls="", ms=6, label="settled pose")]
    ax.legend(handles=handles, fontsize=8, loc="lower left", frameon=False)
    # the stop zone: the robot's offset from the stop pose, at the switch (open) -> settled (filled)
    for box, ls in ((0.03, "-"), (0.05, "--")):
        az.add_patch(Rectangle((-box * 100, -box * 100), 2 * box * 100, 2 * box * 100, fill=False, ec=C["muted"], ls=ls, lw=0.8))
    az.plot(0, 0, "+", color=C["ink"], ms=12, mew=1.5)
    for r in rows:
        sa, sl, sh = offset(r.get("err_at_switch")); fa, fl, fh = offset(r.get("final_err"))
        if fa is None:
            continue
        if sa is not None:
            az.plot([sa * 100, fa * 100], [sl * 100, fl * 100], color=C["muted"], lw=0.7)
            az.plot(sa * 100, sl * 100, "o", mfc="white", mec=C["FAR"], ms=6, mew=1.2)
        az.plot(fa * 100, fl * 100, "o", color=C["meas"], ms=6)
        hr = math.radians(fh)
        az.plot([fa * 100, fa * 100 + 2.5 * math.cos(hr)], [fl * 100, fl * 100 + 2.5 * math.sin(hr)], color=C["meas"], lw=1.2)
        az.annotate(str(r["trial"]), (fa * 100, fl * 100), xytext=(4, 4), textcoords="offset points", fontsize=8, color=C["ink"])
    az.set_aspect("equal"); style(az)
    offs = [abs(v) * 100 for r in rows for v in offset(r.get("final_err"))[:2] + offset(r.get("err_at_switch"))[:2] if v is not None]
    lim = max(12.0, math.ceil(1.15 * max(offs) / 5) * 5) if offs else 20.0
    az.set_xlim(-lim, lim); az.set_ylim(-lim, lim)
    az.set_xlabel("fwd offset from the stop [cm]  (+ = towards the table)", color=C["muted"], fontsize=9)
    az.set_ylabel("left offset [cm]  (+ = left of the approach line)", color=C["muted"], fontsize=9)
    az.set_title("stop zone: switch (open) -> settled (filled; tick = heading)\nboxes = +-3 cm (solid) / +-5 cm (dashed)", fontsize=10, color=C["ink"], loc="left")
    fig.suptitle(title, fontsize=11, color=C["ink"], x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out + "_map.png", dpi=130); plt.close(fig)
    return out + "_map.png"


def plot_velocity(rows, trajs, out, title):
    sel = [(r, d) for r, d in zip(rows, trajs) if d is not None]
    if not sel:
        return None
    n = len(sel)
    fig, axs = plt.subplots(n, 3, figsize=(15, 1.55 * n + 1.2), squeeze=False)
    tmax = max(float(d["t"][-1]) for _, d in sel)
    cols = (("cmd_vx", "mvx", "ovx", "vx [m/s]"), ("cmd_vy", "mvy", "ovy", "vy [m/s]"), ("cmd_wz", "mwz", "owz", "wz [rad/s]"))
    for i, (r, d) in enumerate(sel):
        t = d["t"]
        for j, (kc, km, ko, lab) in enumerate(cols):
            ax = axs[i, j]
            named = set()
            for p, i0, i1 in runs(d["phase"]):
                t1 = t[i1 + 1] if i1 + 1 < len(t) else t[i1] + 0.1
                ax.axvspan(t[i0], t1, color=C.get(p, C["muted"]), alpha=0.08, lw=0)
                if i == 0 and j == 0 and p in ("FAR", "ALIGN", "APPROACH", "SETTLE") and p not in named and t1 - t[i0] > 1.5:
                    ax.text((t[i0] + t1) / 2, 1.02 + 0.16 * (len(named) % 2), p, ha="center", va="bottom", fontsize=7, color=C.get(p, C["muted"]), transform=ax.get_xaxis_transform())
                    named.add(p)
            if np.isfinite(d[ko]).any():
                ax.plot(t, d[ko], color=C["truth"], lw=0.8, label="sim truth")
            ax.plot(t, d[km], color=C["meas"], lw=1.1, label="measured (pose diff)")
            ax.plot(t, d[kc], color=C["cmd"], lw=1.5, drawstyle="steps-post", label="commanded")
            if r.get("t_switch") is not None:
                ax.axvline(r["t_switch"], color=C["muted"], ls="--", lw=0.8)
            ax.set_xlim(0, tmax); style(ax)
            if j == 0:
                ax.set_ylabel(f"trial {r['trial']}\n{lab}", fontsize=8, color=C["ink"])
            else:
                ax.set_ylabel(lab, fontsize=8, color=C["muted"])
            if i == n - 1:
                ax.set_xlabel("t since goto [s]", fontsize=8, color=C["muted"])
            if i == 0 and j == 2:
                ax.legend(fontsize=7, loc="upper right", frameon=False)
    fig.suptitle(title + "\nvelocities: commanded (blue) vs measured (orange) vs sim truth (grey); bands = phases; dashed = balance active", fontsize=10, color=C["ink"], x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out + "_velocity.png", dpi=120); plt.close(fig)
    return out + "_velocity.png"


def plot_errors(rows, trajs, out, title):
    sel = [(r, d) for r, d in zip(rows, trajs) if d is not None and r.get("t_switch") is not None]
    if not sel:
        return None
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    labs = ("fwd offset [cm]  (+ = towards the table)", "left offset [cm]", "heading [deg]  (+ = turned left)")
    keys = ("along", "lat", "head_deg"); scale = (-100, -100, -1)    # to-go error -> robot offset
    grid = np.arange(-15, max(float(d["t"][-1]) - r["t_switch"] for r, d in sel) + 0.1, 0.1)
    for k, (key, sc, lab) in enumerate(zip(keys, scale, labs)):
        ax = axs[k]; acc = []
        for r, d in sel:
            tt = d["t"] - r["t_switch"]
            ax.plot(tt, d[key] * sc, color=C["FAR"], lw=0.8, alpha=0.45)
            acc.append(np.interp(grid, tt, d[key] * sc, left=np.nan, right=np.nan))
        with np.errstate(all="ignore"):
            import warnings
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", RuntimeWarning)
                mean = np.nanmean(np.array(acc), axis=0)      # all-NaN columns before the earliest trial start are expected
        ax.plot(grid, mean, color=C["ink"], lw=1.8, label="mean over trials")
        band = 3 if k < 2 else 2
        ax.axhspan(-band, band, color=C["APPROACH"], alpha=0.12, lw=0)
        ax.axvline(0, color=C["muted"], ls="--", lw=0.9)
        ax.set_ylabel(lab, fontsize=9, color=C["muted"]); style(ax)
        ax.set_ylim(-20, 20) if k < 2 else ax.set_ylim(-15, 15)
    hold = [w["t"] - r["t_switch"] for r, _ in sel for w in (r.get("waypoints") or []) if w["phase"] == "HOLD"]
    if hold:
        axs[0].axvline(float(np.mean(hold)), color=C["HOLD"], ls=":", lw=1.0)
        axs[0].text(float(np.mean(hold)), 19, "arrival (mean)", fontsize=8, color=C["HOLD"], ha="right", va="top")
    axs[0].text(0.2, 19, "balance active", fontsize=8, color=C["muted"], va="top")
    axs[0].legend(fontsize=8, loc="lower left", frameon=False)
    axs[-1].set_xlabel("t relative to the balance switch [s]", fontsize=9, color=C["muted"])
    axs[-1].set_xlim(-15, grid[-1])
    fig.suptitle(title + "\noffset from the stop pose around the balance switch (green band = tolerance)", fontsize=10, color=C["ink"], x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out + "_errors.png", dpi=130); plt.close(fig)
    return out + "_errors.png"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jsonl", required=True); ap.add_argument("--n", type=int, default=10); ap.add_argument("--traj", required=True)
    ap.add_argument("--out", required=True); ap.add_argument("--title", default="homing batch")
    args = ap.parse_args()
    rows, trajs = load(args)
    if not rows:
        print("no records"); return
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    for fn in (plot_map, plot_velocity, plot_errors):
        p = fn(rows, trajs, args.out, args.title)
        print(f"plot -> {p}" if p else f"{fn.__name__}: nothing to plot (no trajectory files / no switch)")


if __name__ == "__main__":
    main()
