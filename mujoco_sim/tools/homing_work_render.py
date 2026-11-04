#!/usr/bin/env python3
"""homing_work_render.py -- part 2 of the homing + work video (2026-11-04): the ten robots at the round table, each on its balance
policy, reaching coloured balls with its arms; optionally glued after part 1 (tools/homing_video_render.py --hands --fill
--cut-after-switch-s) into one video.

Input: --work-rec = a tools/homing_video_record.sh --items ... --work PLAN record dir: per robot (key <seed>_t<T>) state_<key>.bin/.json
(rt/sim_base_pose, 50 Hz of sim time), trials_<key>.jsonl and work_<key>.jsonl (tools/homing_work_driver.py: the sequence start t0,
per ball its torso-frame centre and its window t_move0 / t_arrive / t_hold_end, the palm point per hand). --selection = part 1's
selection json (slot -> clip "s<seed>/t<T>"): slot s gets the work recording of the SAME trial, placed with the same rotation onto
slot s of the round table (hv.place). --rec = part 1's record dirs (the trails and the spot discs stay on the floor).
Every robot is its own physics run, replayed kinematically (contacts off), the robots never touch.
Timeline (real time): at the cut robot s stands at sim t0_s - --lead - s x --ripple, so its arm sequence starts --lead + s x --ripple
video s after the cut -- a ripple around the table.
Balls (the robot's own colour, render-only props): a ball's centre is fixed in the world where the robot's torso frame put it when that
reach began (the IK target was solved in the torso frame); it appears --appear-s before the reach, and POPS when the palm point
(FK of the recorded joints, the plan's hand point in the wrist_yaw_link frame) comes within --pop-dist of its centre -- or, if it never
does, at the end of its hold (listed as a MISS in the report). The pair (P_L + P_R) appears together, each ball pops on its own hand.
Camera: from above the table centre (--cam-h) looking outward and down, panning so the robots face it in turn; then it rises to a
top-down view of all ten (--rise-at .. --rise-end) and turns slowly.
Report (stdout + <out>.report.json): per robot the stand steps (a foot lifted > --step-lift), the pelvis drift over the arm sequence and
over the 8 s before it (the stand's own creep), each ball's minimum palm distance and how it popped.

  MUJOCO_GL=egl ~/miniconda3/envs/tv/bin/python tools/homing_work_render.py --work-rec logs/homing_video/work_<stamp> \\
      --rec logs/homing_video/rec_A logs/homing_video/rec_B --selection logs/homing_video/rec_A/selection_homing_10_robots.json \\
      --out part2.mp4 [--prepend part1.mp4 --full full.mp4] [--stills 0.1,0.5,0.9] [--report-only]
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import homing_video_render as hv  # noqa: E402


def load_work(d, key):
    meta = json.load(open(os.path.join(d, f"state_{key}.json"))); W = meta["width"]
    raw = np.fromfile(os.path.join(d, f"state_{key}.bin"), dtype=np.float64)
    raw = raw[: (len(raw) // W) * W].reshape(-1, W)
    raw = raw[np.argsort(raw[:, 0], kind="stable")]
    raw = raw[np.concatenate([[True], np.diff(raw[:, 0]) > 1e-6])]
    lines = [json.loads(l) for l in open(os.path.join(d, f"work_{key}.jsonl")) if l.strip()]
    plan = next((l for l in lines if l.get("kind") == "plan"), None)
    if plan is None:
        raise SystemExit(f"{key}: no arm sequence in work_{key}.jsonl ({[l.get('kind') for l in lines]}) -- re-record this trial")
    T = raw[:, 0]
    return {"key": key, "T": T, "qpos": raw[:, 2:].copy(), "plan": plan, "tau": T - T[0]}


def qpos_at(w, t):
    return hv.clip_qpos(w, t - w["T"][0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--work-rec", required=True)
    ap.add_argument("--rec", nargs="+", required=True, help="part 1's record dirs (trails, spot discs)")
    ap.add_argument("--selection", required=True, help="part 1's selection json")
    ap.add_argument("--out", required=True)
    ap.add_argument("--prepend", default="", help="part 1 mp4: --full = it + this part (stream copy, same encoder settings)")
    ap.add_argument("--full", default="")
    ap.add_argument("--lead", type=float, default=1.0, help="video s from the cut to robot 0's first arm move")
    ap.add_argument("--ripple", type=float, default=0.2, help="video s between neighbouring robots' sequences")
    ap.add_argument("--end", default="ready", help="the video ends after every robot's arms are back at: ready | default")
    ap.add_argument("--end-pad", type=float, default=1.5, help="video s after that")
    ap.add_argument("--appear-s", type=float, default=0.6, help="a ball appears this long before its reach starts (s)")
    ap.add_argument("--pop-dist", type=float, default=0.05, help="palm point -> ball centre distance that pops the ball (m)")
    ap.add_argument("--step-lift", type=float, default=0.015, help="a foot higher than this above its stand height = a step (m)")
    ap.add_argument("--fps", type=int, default=60)
    ap.add_argument("--size", default="1920x1080")
    ap.add_argument("--fovy", type=float, default=45.0)
    ap.add_argument("--cam-h", type=float, default=1.65, help="camera height over the table centre, pan phase (m above the floor)")
    ap.add_argument("--cam-el", type=float, default=-14.0, help="pan elevation (deg)")
    ap.add_argument("--cam-look", type=float, default=1.5, help="look distance of the pan (m)")
    ap.add_argument("--az0", type=float, default=-25.0, help="pan start azimuth relative to slot 0 (deg)")
    ap.add_argument("--pan-rate", type=float, default=13.0, help="deg per video s")
    ap.add_argument("--rise-at", type=float, default=10.5, help="video s: the camera starts rising")
    ap.add_argument("--rise-end", type=float, default=14.0, help="video s: top-down reached")
    ap.add_argument("--top-h", type=float, default=5.4, help="top-down camera height (m)")
    ap.add_argument("--top-el", type=float, default=-80.0, help="top-down elevation (deg)")
    ap.add_argument("--top-rate", type=float, default=4.0, help="top-down turn (deg per video s)")
    ap.add_argument("--table-r", type=float, default=1.1)
    ap.add_argument("--texrepeat", type=float, default=2.5)
    ap.add_argument("--label", default="Simulation · real time")
    ap.add_argument("--crf", type=int, default=17)
    ap.add_argument("--stills", default="", help="comma list of video fractions -> PNG stills, no video")
    ap.add_argument("--report-only", action="store_true")
    a = ap.parse_args()

    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    selj = json.load(open(a.selection))
    n = len(selj["slots"])
    clips, _ = hv.load_clips(a.rec, 1.9, 1e9)
    by_tag = {c["tag"]: c for c in clips}
    sel = [by_tag[sl["clip"]] for sl in selj["slots"]]
    works = []
    for sl in selj["slots"]:
        seed, t = sl["clip"][1:].split("/t")
        works.append(load_work(a.work_rec, f"{seed}_t{t}"))

    # ── FK model (one robot, no hands) for the torso, the palm and the feet ──
    fm = mujoco.MjModel.from_xml_path(hv.ROBOT_XML); fd = mujoco.MjData(fm)
    b_torso = fm.body("torso_link").id
    b_wrist = {"left": fm.body("left_wrist_yaw_link").id, "right": fm.body("right_wrist_yaw_link").id}
    b_foot = [fm.body("left_ankle_roll_link").id, fm.body("right_ankle_roll_link").id]

    def fk(q):
        fd.qpos[:34] = q; mujoco.mj_kinematics(fm, fd)

    report = []
    for s, w in enumerate(works):
        pl = w["plan"]; t0 = pl["t0"]; hp = {k: np.array(v) for k, v in pl["hand_point"].items()}
        T, Q = w["T"], w["qpos"]
        balls = []
        for b in pl["balls"]:
            fk(qpos_at(w, b["t_move0"]))
            c = fd.xpos[b_torso] + fd.xmat[b_torso].reshape(3, 3) @ np.array(b["c_torso"])
            i0, i1 = np.searchsorted(T, b["t_move0"]), np.searchsorted(T, b["t_hold_end"])
            dist = []
            for i in range(i0, min(i1 + 1, len(T))):
                fk(Q[i]); wid = b_wrist[b["side"]]
                dist.append(np.linalg.norm(fd.xpos[wid] + fd.xmat[wid].reshape(3, 3) @ hp[b["side"]] - c))
            dist = np.array(dist)
            hit = np.where(dist <= a.pop_dist)[0]
            t_pop = float(T[i0 + hit[0]]) if len(hit) else float(b["t_hold_end"])
            balls.append(dict(b, c_world=c.tolist(), min_dist=float(dist.min()), t_min=float(T[i0 + int(dist.argmin())]),
                              popped="touch" if len(hit) else "MISS (hold end)", t_pop=t_pop, t_appear=b["t_move0"] - a.appear_s))
        # steps / drift
        def stand_stats(ta, tb):
            ia, ib = np.searchsorted(T, ta), np.searchsorted(T, tb)
            F, P = [], []
            for i in range(ia, min(ib, len(T))):
                fk(Q[i]); F.append([fd.xpos[f].copy() for f in b_foot]); P.append(Q[i, :2].copy())
            F, P = np.array(F), np.array(P)
            lift = (F[:, :, 2] - np.median(F[:, :, 2], axis=0)).max(axis=0)
            slide = np.hypot(F[:, :, 0] - F[0, :, 0], F[:, :, 1] - F[0, :, 1]).max(axis=0)
            return {"drift_end_cm": float(np.hypot(*(P[-1] - P[0])) * 100), "drift_max_cm": float(np.hypot(P[:, 0] - P[0, 0], P[:, 1] - P[0, 1]).max() * 100),
                    "foot_lift_cm": (lift * 100).round(1).tolist(), "foot_slide_cm": (slide * 100).round(1).tolist(),
                    "steps": int((lift > a.step_lift).sum())}
        seq = stand_stats(t0, pl["t_end"])
        pre = stand_stats(t0 - 8.0, t0)
        report.append({"slot": s, "color": hv.COLORS[s % len(hv.COLORS)], "clip": selj["slots"][s]["clip"], "work_key": w["key"],
                       "trial_result": pl["trial"].get("result"), "t_switch_after_goto": pl["trial"].get("t_switch"),
                       "settle_err": pl["trial"].get("final_err"), "t0": t0, "t_end": pl["t_end"], "seq": seq, "pre": pre, "balls": balls})
        w["balls"] = balls

    print(f"[work] {n} robots, arm sequence {min(r['t_end'] - r['t0'] for r in report):.1f}-{max(r['t_end'] - r['t0'] for r in report):.1f} s")
    for r in report:
        sq, pr = r["seq"], r["pre"]
        bl = "  ".join(f"{b['name']} {b['min_dist'] * 100:.1f} cm{'' if b['popped'] == 'touch' else ' MISS'}" for b in r["balls"])
        print(f"   slot {r['slot']} {r['color']} {r['clip']:9s} steps {sq['steps']} (lift L/R {sq['foot_lift_cm']} cm, slide {sq['foot_slide_cm']} cm) "
              f"drift {sq['drift_end_cm']:.1f} cm (max {sq['drift_max_cm']:.1f}; 8 s before: {pr['drift_end_cm']:.1f}) | palm->ball min: {bl}")
    rep_path = os.path.splitext(a.out)[0] + ".report.json"
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    json.dump(report, open(rep_path, "w"), indent=1)
    print(f"[report] -> {rep_path}")
    if a.report_only:
        return 0

    # ── timeline ──
    def t_of(s, v):
        return works[s]["plan"]["t0"] - a.lead - a.ripple * s + v

    def v_of(s, t):
        return t - works[s]["plan"]["t0"] + a.lead + a.ripple * s

    def seg_end(s, key):   # the end of the robot's LAST move to `key`
        return max(g["t_b"] for g in works[s]["plan"]["segments"] if g["kind"] == "move" and g["key"] == key)

    v_end = max(v_of(s, seg_end(s, a.end)) for s in range(n)) + a.end_pad
    nfr = int(round(v_end * a.fps))
    print(f"[timeline] part 2 = {v_end:.1f} s real time ({nfr} frames); robot 0 starts its arms at {a.lead:.1f} s, robot {n - 1} at {a.lead + a.ripple * (n - 1):.1f} s")

    W, H = (int(x) for x in a.size.split("x"))
    m, adr = hv.build_model(n, a.table_r, W, H, a.fovy, extent=8.0, texrep=a.texrepeat, hands=True)
    d = mujoco.MjData(m)
    slots = [hv.slot_rt(2 * math.pi * s / n, a.table_r) for s in range(n)]
    trails = []
    for s, c in enumerate(sel):
        Rm, E, _ = slots[s]
        trails.append(hv.smooth_track(E + (c["qpos"][:, :2] - hv.EDGE0) @ Rm.T, 35))
    marks = [slots[s][1] + slots[s][0] @ np.array([-0.40, 0.0]) for s in range(n)]
    ball_world = []   # per slot: the balls in the slot frame
    for s in range(n):
        Rm, E, _ = slots[s]
        ball_world.append([np.concatenate([E + Rm @ (np.array(b["c_world"][:2]) - hv.EDGE0), [b["c_world"][2]]]) for b in works[s]["balls"]])
    rgba = [hv.hex_rgba(hv.COLORS[s % len(hv.COLORS)], 1.0) for s in range(n)]
    eye3 = np.eye(3).flatten()
    r_ball = works[0]["plan"].get("ball_r", 0.035)

    def add_geoms(scn, v):
        for s in range(n):
            if scn.ngeom >= scn.maxgeom - 4:
                break
            for rad, z, col in ((0.17, 0.0045, rgba[s]),):
                g = scn.geoms[scn.ngeom]
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CYLINDER, np.array([rad, 0.0016, 0.0]), np.array([marks[s][0], marks[s][1], z]), eye3, col)
                scn.ngeom += 1
            t = t_of(s, v)
            for b, cw in zip(works[s]["balls"], ball_world[s]):
                if t < b["t_appear"] or t > b["t_pop"] + 0.25:
                    continue
                if t < b["t_pop"]:
                    sc, al = hv.ease((t - b["t_appear"]) / 0.25), 1.0
                else:
                    u = (t - b["t_pop"]) / 0.25; sc, al = 1.0 + 0.9 * u, 1.0 - u
                col = rgba[s].copy(); col[3] = al
                g = scn.geoms[scn.ngeom]
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([r_ball * max(sc, 1e-3), 0, 0]), cw, eye3, col)
                g.emission = 0.35; g.specular = 0.6; g.shininess = 0.8
                scn.ngeom += 1
        for s, c in enumerate(sel):   # part 1's trails, complete
            tr = trails[s]
            idx = list(range(0, len(tr), 6)) + [len(tr) - 1]
            for i0, i1 in zip(idx[:-1], idx[1:]):
                p0, p1 = tr[i0], tr[i1]
                if np.hypot(*(p1 - p0)) < 1e-3 or scn.ngeom >= scn.maxgeom:
                    continue
                g = scn.geoms[scn.ngeom]
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), eye3, rgba[s])
                mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.016, np.array([p0[0], p0[1], 0.006]), np.array([p1[0], p1[1], 0.006]))
                scn.ngeom += 1

    # ── camera: (position, target) per frame ──
    def dirv(az, el):
        az, el = math.radians(az), math.radians(el)
        return np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])

    def cam_at(v):
        az = a.az0 + a.pan_rate * v
        C_pan = np.array([0.0, 0.0, a.cam_h]); T_pan = C_pan + a.cam_look * dirv(az, a.cam_el)
        az_top = a.az0 + a.pan_rate * a.rise_end + a.top_rate * max(0.0, v - a.rise_end)
        tgt_top = np.array([0.0, 0.0, 1.0])
        C_top = tgt_top - (a.top_h - 1.0) / math.sin(math.radians(-a.top_el)) * dirv(az_top, a.top_el)
        u = hv.ease((v - a.rise_at) / max(a.rise_end - a.rise_at, 1e-6))
        if u <= 0:
            return C_pan, T_pan
        if u >= 1:
            return C_top, tgt_top
        return C_pan + u * (C_top - C_pan), T_pan + u * (tgt_top - T_pan)

    renderer = mujoco.Renderer(m, height=H, width=W, max_geom=20000)
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    opt = mujoco.MjvOption()
    label_patch = None
    if a.label:
        from PIL import Image, ImageDraw, ImageFont
        font = ImageFont.truetype("/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf", int(H * 0.026))
        bbox = font.getbbox(a.label); tw, th = bbox[2] - bbox[0] + 4, bbox[3] + 8
        im = Image.new("RGBA", (tw, th), (0, 0, 0, 0)); ImageDraw.Draw(im).text((2, 2), a.label, font=font, fill=(255, 255, 255, 200))
        label_patch = np.asarray(im).astype(np.float32) / 255.0

    def render(k):
        v = k / a.fps
        for s in range(n):
            Rm, E, al = slots[s]
            d.qpos[adr[s]:adr[s] + 34] = hv.place(qpos_at(works[s], t_of(s, v)), Rm, E, al)
        mujoco.mj_forward(m, d)
        C, Tg = cam_at(v)
        f = Tg - C; dist = float(np.linalg.norm(f)); f /= dist
        cam.lookat[:] = Tg; cam.distance = dist
        cam.azimuth = math.degrees(math.atan2(f[1], f[0])); cam.elevation = math.degrees(math.asin(max(-1.0, min(1.0, f[2]))))
        renderer.update_scene(d, camera=cam, scene_option=opt)
        add_geoms(renderer.scene, v)
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
        for fr in a.stills.split(","):
            k = min(nfr - 1, int(float(fr) * (nfr - 1)))
            Image.fromarray(render(k)).save(f"{base}_still_{float(fr):.2f}.png")
            print(f"[still] frame {k} (video {k / a.fps:.1f} s) -> {base}_still_{float(fr):.2f}.png")
        renderer.close()
        return 0

    tmp = a.out + ".part.mp4"
    ff = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(a.fps), "-i", "-",
                           "-c:v", "libx264", "-preset", "slow", "-crf", str(a.crf), "-profile:v", "high", "-pix_fmt", "yuv420p",
                           "-movflags", "+faststart", tmp], stdin=subprocess.PIPE)
    t0w = time.time()
    for k in range(nfr):
        ff.stdin.write(render(k).tobytes())
        if k % (a.fps * 5) == 0:
            print(f"[render] {k}/{nfr} frames ({time.time() - t0w:.0f} s)", flush=True)
    ff.stdin.close(); rc = ff.wait()
    renderer.close()
    if rc != 0:
        print(f"ffmpeg failed rc={rc}"); return 1
    os.replace(tmp, a.out)
    print(f"[done] {a.out}: {nfr} frames, {nfr / a.fps:.1f} s, rendered in {time.time() - t0w:.0f} s")
    if a.prepend and a.full:
        lst = a.full + ".concat.txt"
        with open(lst, "w") as f:
            f.write(f"file '{os.path.abspath(a.prepend)}'\nfile '{os.path.abspath(a.out)}'\n")
        rc = subprocess.call(["ffmpeg", "-v", "error", "-y", "-f", "concat", "-safe", "0", "-i", lst, "-c", "copy", "-movflags", "+faststart", a.full])
        os.remove(lst)
        print(f"[full] {a.full} (rc {rc})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
