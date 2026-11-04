#!/usr/bin/env python3
"""homing_video_render.py -- N H1-2 robots walking home to ONE round table: a kinematic replay of real lock-step physics trials (2026-11-04).

Input: record dirs from tools/homing_video_record.sh (state_<seed>.bin/.json = rt/sim_base_pose at 50 Hz of sim time: base pose + all
27 joints; trials_<seed>.jsonl = the homing commander's records; teleports_<seed>.txt = its teleport lines).
Clips: every ARRIVED trial, from its teleport frame (the robot standing at its start under the walk policy; the goto comes 2 s later)
to its result + --tail-s (the balance policy standing; the next trial's walk request comes 2.0 s after the result).
Composition: the recording scene's desk edge centre (0.35, 0) with approach heading 0 deg is moved, by a rotation about the vertical
axis + a translation (a symmetry of flat-floor physics), onto slot i of a round table: edge point R (cos phi_i, sin phi_i), approach
heading phi_i + 180 deg (facing the centre), phi_i = i 360/N. Every robot is its own physics run; the robots do not interact
(kinematic replay, contacts off) -- what Isaac's per-env collision filtering does in training.
Selection: N of the clips and their slot order, maximising the minimum pelvis-to-pelvis distance over the synchronised timeline (all
robots start together); a clip whose transformed path comes closer than --table-clear to the table edge is discarded.
Render: MuJoCo offscreen (MUJOCO_GL=egl), orbit camera (--orbit deg, ~45 deg down, dolly-in that keeps every robot framed), coloured
floor trails, raw RGB piped to ffmpeg (H.264, yuv420p).

  MUJOCO_GL=egl ~/miniconda3/envs/tv/bin/python tools/homing_video_render.py --rec logs/homing_video/<stamp> --out video.mp4
  ... --dry                     selection report only
  ... --stills 0,0.3,0.6,1      PNG stills at these fractions of the video (next to --out), no video
"""
import argparse
import glob
import json
import math
import os
import re
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_XML = os.path.abspath(os.path.join(HERE, "..", "..", "unitree_robots", "h1_2", "h1_2_comx06_armature_hand790.xml"))
EDGE0 = np.array([0.35, 0.0])   # the recording scene's desk edge centre; approach heading 0 = +x
TELE_RE = re.compile(r"teleport to \(([-\d.]+), ([-\d.]+), ([-\d.]+)\) at t=([\d.]+), goto at t=([\d.]+)")
# trail / marker colours, adjacent slots alternate warm / cool so neighbours never look alike
COLORS = ["#ff8c1a", "#4cc9f0", "#ffd23f", "#9b7bff", "#a7e22e", "#ff5fa2", "#1ecbc1", "#ff4d4d", "#f2f2f2", "#7fb3ff"]


def hex_rgba(h, a=1.0):
    h = h.lstrip("#")
    return np.array([int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4)] + [a], dtype=np.float32)


# ───────────────────────────── clips ─────────────────────────────
def load_clips(rec_dirs, tail_s, max_switch_s):
    clips, dropped = [], []
    for d in rec_dirs:
        for jpath in sorted(glob.glob(os.path.join(d, "state_*.json"))):
            seed = os.path.basename(jpath)[len("state_"):-len(".json")]
            meta = json.load(open(jpath)); W = meta["width"]
            raw = np.fromfile(jpath[:-5] + ".bin", dtype=np.float64)
            raw = raw[: (len(raw) // W) * W].reshape(-1, W)
            raw = raw[np.argsort(raw[:, 0], kind="stable")]
            raw = raw[np.concatenate([[True], np.diff(raw[:, 0]) > 1e-6])]
            T = raw[:, 0]
            tp_path = os.path.join(d, f"teleports_{seed}.txt")
            tps = []
            if os.path.exists(tp_path):
                for line in open(tp_path, errors="replace"):
                    m = TELE_RE.search(line)
                    if m:
                        tps.append(tuple(float(v) for v in m.groups()))
            recs = [json.loads(l) for l in open(os.path.join(d, f"trials_{seed}.jsonl")) if l.strip()]
            for r in recs:
                tag = f"s{seed}/t{r.get('trial')}"
                if r.get("result") != "ARRIVED":
                    dropped.append((tag, f"result {r.get('result')}")); continue
                tel = r.get("teleport") or r.get("start")
                if r.get("t_switch") is not None and float(r["t_switch"]) > max_switch_s:
                    dropped.append((tag, f"slow arrival: balance switch {float(r['t_switch']):.0f} s after the goto (> --max-switch-s {max_switch_s:g})")); continue
                t_goto = float(r["t_start"])
                cand = [tp for tp in tps if abs(tp[4] - t_goto) < 0.05]
                t_tp = cand[0][3] if cand else t_goto - 2.0
                i0 = np.searchsorted(T, t_tp - 0.05)
                while i0 < len(T) and np.hypot(raw[i0, 2] - tel[0], raw[i0, 3] - tel[1]) > 0.3:
                    i0 += 1
                i0 += 1   # one frame after the teleport landed
                t_res = t_goto + float(r["t_total"])
                i1 = np.searchsorted(T, t_res + tail_s) - 1
                if i0 >= len(T) or i1 <= i0 + 50:
                    dropped.append((tag, "state recording does not cover the trial")); continue
                seg = raw[i0:i1 + 1]
                dt = np.diff(seg[:, 0]); dxy = np.hypot(np.diff(seg[:, 2]), np.diff(seg[:, 3]))
                if dt.max() > 0.25:
                    dropped.append((tag, f"state gap {dt.max():.2f} s")); continue
                if dxy.max() > 0.15:
                    dropped.append((tag, f"base jump {dxy.max():.2f} m inside the clip")); continue
                tau = seg[:, 0] - seg[0, 0]
                fe = r.get("final_err") or {}
                clips.append({
                    "tag": tag, "seed": seed, "trial": r.get("trial"), "tau": tau, "qpos": seg[:, 2:].copy(),
                    "t_goto": t_goto - seg[0, 0], "t_switch": t_goto + float(r.get("t_switch") or r["t_total"]) - seg[0, 0],
                    "t_result": t_res - seg[0, 0], "len": float(tau[-1]),
                    "start": tel, "final_planar": fe.get("planar_m"), "final_head": fe.get("heading_deg"),
                    "gaps": int(np.sum(dt > 0.03)),
                })
    return clips, dropped


def local_xy(clip, grid):
    """pelvis xy relative to the recording desk edge centre, on a time grid (held at the last sample after the clip)"""
    q = clip["qpos"]
    return np.stack([np.interp(grid, clip["tau"], q[:, 0]), np.interp(grid, clip["tau"], q[:, 1])], axis=-1) - EDGE0


def slot_rt(phi, R):
    a = phi + math.pi
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]]), R * np.array([math.cos(phi), math.sin(phi)]), a


def select(clips, n, R, table_clear, iters, seed=0, sep_cap=0.85, quality_w=0.01):
    grid = np.arange(0.0, max(c["len"] for c in clips) + 1e-9, 0.1)
    L = np.stack([local_xy(c, grid) for c in clips])                      # M x G x 2
    # table clearance is independent of the slot (rotation about the table centre)
    Rot0, E0, _ = slot_rt(0.0, R)
    W0 = E0 + L @ Rot0.T                                                  # every clip in slot 0
    clear = np.linalg.norm(W0, axis=-1).min(axis=1) - R                  # pelvis -> table edge, closest approach
    ok = np.where(clear >= table_clear)[0]
    table_drop = [(clips[i]["tag"], f"path {clear[i]:.2f} m from the table edge") for i in range(len(clips)) if clear[i] < table_clear]
    if len(ok) < n:
        raise SystemExit(f"only {len(ok)} usable clips for {n} robots")
    M = len(ok)
    Wk = []
    for k in range(n):                                                    # every usable clip in slot k
        Rk, Ek, _ = slot_rt(2 * math.pi * k / n, R)
        Wk.append(Ek + L[ok] @ Rk.T)
    Wk = np.stack(Wk)                                                     # n x M x G x 2
    # D[a, b, k] = closest pelvis-pelvis distance over the timeline, clip a in slot 0 vs clip b in slot k (rotational symmetry)
    D = np.full((M, M, n), np.inf)
    for k in range(1, n):
        diff = Wk[0][:, None, :, :] - Wk[k][None, :, :, :]                # M x M x G x 2
        D[:, :, k] = np.sqrt((diff ** 2).sum(-1)).min(-1)
    pairs = [(s, t) for s in range(n) for t in range(s + 1, n)]
    P_s = np.array([p[0] for p in pairs]); P_t = np.array([p[1] for p in pairs]); P_k = (P_t - P_s) % n

    # secondary: a clip that settles squarely on its spot (heading deg / 5 + planar m / 0.05) -- counts only once every pair keeps sep_cap
    pen = np.array([abs(clips[i]["final_head"] or 0.0) / 5.0 + (clips[i]["final_planar"] or 0.0) / 0.05 for i in ok])

    def score(a):
        dd = D[a[P_s], a[P_t], P_k]
        return min(dd.min(), sep_cap) + 0.002 * np.minimum(dd, 1.5).sum() - quality_w * pen[a].sum(), dd.min()

    rng = np.random.default_rng(seed)
    best, best_s = None, -1e9
    for _restart in range(24):
        a = rng.choice(M, n, replace=False)
        s_cur, _ = score(a)
        temp = 0.3
        for it in range(iters):
            b = a.copy()
            if rng.random() < 0.5:
                i, j = rng.choice(n, 2, replace=False); b[i], b[j] = b[j], b[i]
            else:
                unused = np.setdiff1d(np.arange(M), b)
                if len(unused) == 0:
                    continue
                b[rng.integers(n)] = rng.choice(unused)
            s_new, _ = score(b)
            if s_new >= s_cur or rng.random() < math.exp((s_new - s_cur) / max(temp, 1e-4)):
                a, s_cur = b, s_new
            temp *= 0.999
            if s_cur > best_s:
                best, best_s = a.copy(), s_cur
    _, dmin = score(best)
    chosen = [int(ok[i]) for i in best]
    per_pair = {(s, t): float(D[best[s], best[t], (t - s) % n]) for s, t in pairs}
    return chosen, dmin, per_pair, table_drop, clear


# ───────────────────────────── scene ─────────────────────────────
SCENE = """
<mujoco model="homing_video">
  <option timestep="0.002"><flag contact="disable" gravity="disable"/></option>
  <statistic center="0 0 0.8" extent="{extent}"/>
  <visual>
    <headlight diffuse="0.50 0.50 0.50" ambient="0.30 0.30 0.32" specular="0.15 0.15 0.15"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global offwidth="{w}" offheight="{h}" fovy="{fovy}"/>
    <quality shadowsize="8192" offsamples="8" numslices="128"/>
    <map znear="0.02" zfar="80" haze="0.4"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.25 0.38 0.52" rgb2="0.02 0.04 0.08" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="{texrep} {texrep}" reflectance="0.2"/>
    <material name="wood" rgba="0.55 0.39 0.25 1" specular="0.25" shininess="0.35"/>
    <material name="steel" rgba="0.22 0.22 0.24 1" specular="0.6" shininess="0.7"/>
  </asset>
  <worldbody>
    <light name="sun" pos="2 -3 9" dir="-0.18 0.28 -1" directional="true" castshadow="true" diffuse="0.62 0.62 0.60" specular="0.25 0.25 0.25"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
    <body name="table">
      <geom name="table_top" type="cylinder" size="{R} 0.02" pos="0 0 0.98" material="wood"/>
      <geom name="table_post" type="cylinder" size="0.07 0.47" pos="0 0 0.49" material="steel"/>
      <geom name="table_foot" type="cylinder" size="0.42 0.015" pos="0 0 0.015" material="steel"/>
    </body>
  </worldbody>
</mujoco>
"""


def build_model(n, R, w, h, fovy, extent, texrep):
    import mujoco
    spec = mujoco.MjSpec.from_string(SCENE.format(R=R, w=w, h=h, fovy=fovy, extent=extent, texrep=texrep))
    for i in range(n):
        child = mujoco.MjSpec.from_file(ROBOT_XML)
        frame = spec.worldbody.add_frame()
        spec.attach(child, frame=frame, prefix=f"r{i}_")
    m = spec.compile()
    adr = []
    for i in range(n):
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"r{i}_floating_base_joint")
        adr.append(int(m.jnt_qposadr[j]))
    return m, adr


# ───────────────────────────── timeline / state ─────────────────────────────
def quat_mul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def clip_qpos(clip, tau):
    """interpolated recorded qpos (34) at clip time tau (held at the ends): linear for positions / joints, normalised lerp for the quat"""
    t = clip["tau"]; q = clip["qpos"]
    tau = min(max(tau, 0.0), t[-1])
    i = int(np.searchsorted(t, tau))
    if i <= 0:
        return q[0].copy()
    if i >= len(t):
        return q[-1].copy()
    f = (tau - t[i - 1]) / max(t[i] - t[i - 1], 1e-9)
    a, b = q[i - 1], q[i]
    out = a + f * (b - a)
    qa, qb = a[3:7], b[3:7]
    if np.dot(qa, qb) < 0:
        qb = -qb
    qq = qa + f * (qb - qa)
    out[3:7] = qq / np.linalg.norm(qq)
    return out


def place(qpos_rec, Rm, E, alpha):
    """the recording-frame qpos -> the slot: rotate about the vertical axis by alpha around the desk edge, translate onto the slot edge"""
    out = qpos_rec.copy()
    out[0:2] = E + Rm @ (qpos_rec[0:2] - EDGE0)
    qz = np.array([math.cos(alpha / 2), 0.0, 0.0, math.sin(alpha / 2)])
    out[3:7] = quat_mul(qz, qpos_rec[3:7])
    return out


def smooth_track(xy, win):
    if win <= 1:
        return xy
    k = np.ones(win) / win
    pad = win // 2
    p = np.pad(xy, ((pad, win - 1 - pad), (0, 0)), mode="edge")
    return np.stack([np.convolve(p[:, 0], k, mode="valid"), np.convolve(p[:, 1], k, mode="valid")], axis=-1)


# ───────────────────────────── camera ─────────────────────────────
def cam_frame(lookat, dist, az, el):
    a, e = math.radians(az), math.radians(el)
    fwd = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
    up = np.array([-math.sin(e) * math.cos(a), -math.sin(e) * math.sin(a), math.cos(e)])
    right = np.cross(fwd, up)
    return lookat - dist * fwd, fwd, up, right


def needed_dist(points, lookat, az, el, fovy, aspect, margin):
    """the smallest camera distance that keeps every point inside the frame with an NDC margin"""
    ty = math.tan(math.radians(fovy) / 2); tx = ty * aspect
    lim = 1.0 - margin

    def fits(dist):
        pos, fwd, up, right = cam_frame(lookat, dist, az, el)
        v = points - pos
        z = v @ fwd
        if np.any(z <= 0.1):
            return False
        return np.all(np.abs(v @ right) / (z * tx) <= lim) and np.all(np.abs(v @ up) / (z * ty) <= lim)

    lo, hi = 1.0, 60.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if fits(mid):
            hi = mid
        else:
            lo = mid
    return hi


def ease(u):
    u = min(max(u, 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * u)


# ───────────────────────────── main ─────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rec", nargs="+", required=True, help="record dirs (tools/homing_video_record.sh)")
    ap.add_argument("--out", default=None, help="mp4 path")
    ap.add_argument("--n", type=int, default=10, help="robots")
    ap.add_argument("--table-r", type=float, default=1.1, help="round table radius = edge distance from the centre (m)")
    ap.add_argument("--table-clear", type=float, default=0.25, help="min pelvis -> table edge distance on the way (m)")
    ap.add_argument("--max-switch-s", type=float, default=45.0, help="clips whose balance switch comes later than this after the goto are not used (s)")
    ap.add_argument("--tail-s", type=float, default=1.9, help="clip end = result + this (s; the next walk request comes at +2.0)")
    ap.add_argument("--after-switch-s", type=float, default=5.0, help="the video ends this many sim s after the LAST robot's balance switch (+ the outro)")
    ap.add_argument("--speed", type=float, default=0.0, help="sim s per video s (0 = auto from --target-s, rounded to 0.25)")
    ap.add_argument("--target-s", type=float, default=30.0, help="video length the auto speed aims at (s)")
    ap.add_argument("--intro-s", type=float, default=1.5, help="video s on the first frame before the clock runs")
    ap.add_argument("--outro-s", type=float, default=3.0, help="video s after the end (the camera finishes its move)")
    ap.add_argument("--fps", type=int, default=60)
    ap.add_argument("--size", default="1920x1080")
    ap.add_argument("--fovy", type=float, default=40.0)
    ap.add_argument("--orbit", type=float, default=180.0, help="camera azimuth sweep (deg)")
    ap.add_argument("--az0", type=float, default=-120.0, help="start azimuth (deg)")
    ap.add_argument("--el", default="-47,-40", help="camera elevation start,end (deg, negative = looking down)")
    ap.add_argument("--margin", type=float, default=0.10, help="NDC margin kept around the robots")
    ap.add_argument("--min-dist", type=float, default=6.0, help="the closest the camera gets (m)")
    ap.add_argument("--trail-dt", type=float, default=0.12, help="trail segment spacing (sim s)")
    ap.add_argument("--no-trails", action="store_true")
    ap.add_argument("--texrepeat", type=float, default=2.5, help="floor checker repeats per metre (the desk scene: 5 = 0.1 m checks; 2.5 = 0.2 m, calmer in the wide shot)")
    ap.add_argument("--no-marks", action="store_true", help="no floor marks on the robots' table spots")
    ap.add_argument("--label", default="Simulation · {speed}× speed", help="corner label ('' = none)")
    ap.add_argument("--crf", type=int, default=17)
    ap.add_argument("--iters", type=int, default=4000, help="annealing iterations per restart (selection)")
    ap.add_argument("--select-seed", type=int, default=0)
    ap.add_argument("--sep-cap", type=float, default=0.85, help="selection: pelvis separation (m) above which the settle quality decides")
    ap.add_argument("--quality-w", type=float, default=0.01, help="selection: weight of the settled heading / position error")
    ap.add_argument("--dry", action="store_true", help="selection report only")
    ap.add_argument("--stills", default="", help="comma list of video fractions -> PNG stills, no video")
    a = ap.parse_args()

    clips, dropped = load_clips(a.rec, a.tail_s, a.max_switch_s)
    print(f"[clips] {len(clips)} ARRIVED clips usable, {len(dropped)} dropped")
    for tag, why in dropped:
        print(f"   dropped {tag}: {why}")
    chosen, dmin, per_pair, table_drop, clear = select(clips, a.n, a.table_r, a.table_clear, a.iters, a.select_seed, a.sep_cap, a.quality_w)
    for tag, why in table_drop:
        print(f"   table-clearance discard {tag}: {why}")
    sel = [clips[i] for i in chosen]
    print(f"[select] {a.n} of {len(clips)} clips; closest pelvis-pelvis approach over the timeline {dmin:.2f} m")
    for s, c in enumerate(sel):
        nb = min(per_pair.get((min(s, t), max(s, t)), 9) for t in ((s - 1) % a.n, (s + 1) % a.n))
        d0 = float(np.linalg.norm(c["qpos"][0, :2] - EDGE0))
        print(f"   slot {s} {COLORS[s % len(COLORS)]}: {c['tag']:9s} start {d0:.2f} m from the edge, goto {c['t_goto']:.1f} s, switch {c['t_switch']:.1f} s, "
              f"result {c['t_result']:.1f} s, settled err {c['final_planar']} m / {c['final_head']} deg, closest neighbour {nb:.2f} m, gaps {c['gaps']}")
    T_end = max(c["t_switch"] for c in sel) + a.after_switch_s
    speed = a.speed or max(1.0, round(T_end / (a.target_s - a.intro_s - a.outro_s) * 4) / 4)
    T_video = a.intro_s + T_end / speed + a.outro_s
    print(f"[timeline] sim {T_end:.1f} s at {speed}x -> video {T_video:.1f} s ({a.fps} fps)")
    sel_info = {"rec": a.rec, "n": a.n, "table_r": a.table_r, "speed": speed, "T_end_sim": T_end, "T_video": T_video, "closest_m": dmin,
                "slots": [{"slot": s, "clip": c["tag"], "color": COLORS[s % len(COLORS)], "t_switch": c["t_switch"], "final_planar": c["final_planar"]} for s, c in enumerate(sel)],
                "dropped": dropped, "table_discards": table_drop, "n_clips": len(clips)}
    if a.dry or not a.out:
        print(json.dumps(sel_info, indent=1, default=str)[:3000])
        return 0

    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    W, H = (int(v) for v in a.size.split("x"))
    m, adr = build_model(a.n, a.table_r, W, H, a.fovy, extent=8.0, texrep=a.texrepeat)
    d = mujoco.MjData(m)
    slots = [slot_rt(2 * math.pi * s / a.n, a.table_r) for s in range(a.n)]
    # trails: the smoothed pelvis track per robot in its slot (0.7 s window ~ one gait cycle -> no step wobble)
    trails = []
    for s, c in enumerate(sel):
        Rm, E, _ = slots[s]
        xy = E + (c["qpos"][:, :2] - EDGE0) @ Rm.T
        trails.append(smooth_track(xy, 35))
    marks = []
    for s in range(a.n):   # the nominal stop pose (pelvis) of each slot: torso front 0.25 + 0.15 m behind the edge
        Rm, E, _ = slots[s]
        marks.append(E + Rm @ np.array([-0.40, 0.0]))

    el0, el1 = (float(v) for v in a.el.split(","))
    lookat = np.array([0.0, 0.0, 0.75])
    nfr = int(round(T_video * a.fps))
    aspect = W / H

    def tau_of(k):
        return max(0.0, (k / a.fps - a.intro_s)) * speed

    def az_el(k):
        u = ease(k / max(nfr - 1, 1))
        return a.az0 + a.orbit * u, el0 + (el1 - el0) * u

    # camera distance: per frame the distance that frames every robot (feet + head) and the table ring, then the reverse running
    # max (never zoom in on a robot that will walk out of the frame later) and a smoothing pass
    ring = np.array([[ (a.table_r + 0.55) * math.cos(t), (a.table_r + 0.55) * math.sin(t), z] for t in np.linspace(0, 2 * math.pi, 16, endpoint=False) for z in (0.0, 1.75)])
    req = np.zeros(nfr)
    for k in range(nfr):
        tau = tau_of(k); pts = [ring]
        for s, c in enumerate(sel):
            Rm, E, _ = slots[s]
            p = E + Rm @ (clip_qpos(c, tau)[0:2] - EDGE0)
            pts.append(np.array([[p[0], p[1], 0.0], [p[0], p[1], 1.8]]))
        az, el = az_el(k)
        req[k] = max(a.min_dist, needed_dist(np.concatenate(pts), lookat, az, el, a.fovy, aspect, a.margin))
    dist = np.maximum.accumulate(req[::-1])[::-1]
    sig = int(0.6 * a.fps); kern = np.exp(-0.5 * (np.arange(-3 * sig, 3 * sig + 1) / sig) ** 2); kern /= kern.sum()
    dist = np.convolve(np.pad(dist, (3 * sig, 3 * sig), mode="edge"), kern, mode="valid") * 1.02
    print(f"[camera] distance {dist[0]:.1f} -> {dist[-1]:.1f} m, azimuth {a.az0:.0f} -> {a.az0 + a.orbit:.0f} deg, elevation {el0:.0f} -> {el1:.0f} deg")

    renderer = mujoco.Renderer(m, height=H, width=W, max_geom=20000)
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    opt = mujoco.MjvOption()
    rgba_trail = [hex_rgba(COLORS[s % len(COLORS)], 1.0) for s in range(a.n)]   # opaque: overlapping translucent capsules look beaded
    rgba_mark = [hex_rgba(COLORS[s % len(COLORS)], 0.55) for s in range(a.n)]
    eye3 = np.eye(3).flatten()

    label_patch = None
    if a.label:
        try:
            from PIL import Image, ImageDraw, ImageFont
            txt = a.label.format(speed=(f"{speed:g}"))
            font = ImageFont.truetype("/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf", int(H * 0.026))
            bbox = font.getbbox(txt); tw, th = bbox[2] - bbox[0] + 4, bbox[3] + 8
            im = Image.new("RGBA", (tw, th), (0, 0, 0, 0)); ImageDraw.Draw(im).text((2, 2), txt, font=font, fill=(255, 255, 255, 200))
            label_patch = np.asarray(im).astype(np.float32) / 255.0
        except Exception as e:  # noqa: BLE001
            print(f"[label] skipped ({e})")

    def add_geoms(scn, tau):
        if not a.no_marks:
            for s in range(a.n):
                if scn.ngeom >= scn.maxgeom:
                    break
                g = scn.geoms[scn.ngeom]
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CYLINDER, np.array([0.17, 0.0015, 0.0]),   # model-style size: radius, half-height
                                    np.array([marks[s][0], marks[s][1], 0.003]), eye3, rgba_mark[s])
                scn.ngeom += 1
        if a.no_trails:
            return
        for s, c in enumerate(sel):
            tr = trails[s]; tt = c["tau"]
            iend = int(np.searchsorted(tt, min(tau, tt[-1]), side="right"))
            if iend < 2:
                continue
            step = max(1, int(round(a.trail_dt / 0.02)))
            idx = list(range(0, iend, step))
            if idx[-1] != iend - 1:
                idx.append(iend - 1)
            for i0, i1 in zip(idx[:-1], idx[1:]):
                p0, p1 = tr[i0], tr[i1]
                if np.hypot(*(p1 - p0)) < 1e-3 or scn.ngeom >= scn.maxgeom:
                    continue
                g = scn.geoms[scn.ngeom]
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), eye3, rgba_trail[s])
                mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.016, np.array([p0[0], p0[1], 0.006]), np.array([p1[0], p1[1], 0.006]))
                scn.ngeom += 1

    def render(k):
        tau = tau_of(k)
        for s, c in enumerate(sel):
            Rm, E, al = slots[s]
            d.qpos[adr[s]:adr[s] + 34] = place(clip_qpos(c, tau), Rm, E, al)
        mujoco.mj_forward(m, d)
        az, el = az_el(k)
        cam.lookat[:] = lookat; cam.distance = float(dist[k]); cam.azimuth = az; cam.elevation = el
        renderer.update_scene(d, camera=cam, scene_option=opt)
        add_geoms(renderer.scene, tau)
        img = renderer.render()
        if label_patch is not None:
            ph, pw = label_patch.shape[:2]; y0 = H - ph - int(H * 0.03); x0 = W - pw - int(W * 0.02)
            al_ = label_patch[:, :, 3:4]
            reg = img[y0:y0 + ph, x0:x0 + pw].astype(np.float32) / 255.0
            img = img.copy(); img[y0:y0 + ph, x0:x0 + pw] = ((reg * (1 - al_) + label_patch[:, :, :3] * al_) * 255).astype(np.uint8)
        return img

    if a.stills:
        from PIL import Image
        base = os.path.splitext(a.out)[0]
        os.makedirs(os.path.dirname(os.path.abspath(base)), exist_ok=True)
        for fr in a.stills.split(","):
            k = min(nfr - 1, int(float(fr) * (nfr - 1)))
            t0 = time.time(); img = render(k)
            p = f"{base}_still_{float(fr):.2f}.png"; Image.fromarray(img).save(p)
            print(f"[still] frame {k} (video {k / a.fps:.1f} s, sim {tau_of(k):.1f} s) -> {p} ({time.time() - t0:.2f} s)")
        renderer.close()
        return 0

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    tmp = a.out + ".part.mp4"
    ff = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(a.fps), "-i", "-",
                           "-c:v", "libx264", "-preset", "slow", "-crf", str(a.crf), "-profile:v", "high", "-pix_fmt", "yuv420p",
                           "-movflags", "+faststart", tmp], stdin=subprocess.PIPE)
    t0 = time.time()
    for k in range(nfr):
        ff.stdin.write(render(k).tobytes())
        if k % (a.fps * 5) == 0:
            print(f"[render] {k}/{nfr} frames ({time.time() - t0:.0f} s)", flush=True)
    ff.stdin.close(); rc = ff.wait()
    renderer.close()
    if rc != 0:
        print(f"ffmpeg failed rc={rc}"); return 1
    os.replace(tmp, a.out)
    with open(os.path.join(a.rec[0], "selection_" + os.path.splitext(os.path.basename(a.out))[0] + ".json"), "w") as f:
        json.dump(sel_info, f, indent=1, default=str)
    print(f"[done] {a.out}: {nfr} frames, {T_video:.1f} s, rendered in {time.time() - t0:.0f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
