#!/usr/bin/env python3
"""stumble_field.py -- the rough floor of Exam #2 Test 5 "stumble" (2026-11-02, results/EXAM2_SPEC.md section 3, Test 5).

Why: on the robot (2026-11-02) lm7i_combo_soft scuffed its left toe at 0.15 m/s and recovered with a huge right step; the exam never
flagged it because the sim floor is perfectly flat and its swing height (1.6 cm L / 2.3 cm R at 0.15) was not a headline column.

The field lives FAR from every other test (x 50..64 m, y -6..6 m: the ladders walk to -x from x = -3, the pushes stay near x = -3, the
homing starts within 5 m of the desk at x = 0.35, the handover stands at x = -3). Two kinds (STUMBLE_KIND):
  bumps  (default) -- partially buried SPHERES on the plain plane: radius R, cap heights uniform in [H_MIN, H_MAX] (0.4..2 cm), one per
         20 x 20 cm cell with a +-8 cm jitter (~28 % of the area covered, 0.4-1 bump under a 26 x 9 cm sole); smooth convex bumps, no
         seams, no boundary walls, no surface coincident with the plane. The first 10 cm of the field hold no bump rim (the robot steps
         INTO the roughness). A bump of cap height h on a sphere of radius R has the rim radius sqrt(2Rh - h^2) (h 1 cm: 5.4 cm; 2 cm:
         7.5 cm) and a rim slope of acos(1 - h/R) (21 / 30 deg): a pebble, not a wall.
  hfield -- a MuJoCo height field, 10 cm tiles with heights uniform in [0, AMP], sampled every 5 cm, the first 10 cm strip at 0, flush
         with the plane. REJECTED 2026-11-02 for the exam: lm7i_combo_soft stalled at the field's near edge after the same 9 lifts at
         AMP 2 cm, 1 cm AND 0.4 cm (bit-identical up to the stall: not the bumps but the field boundary -- the prisms' vertical side faces
         meet a foot that slides at z ~ 0 with a few mm of contact penetration and act as a wall; it walked the same spot on the plain
         plane at 0.14 m/s). Kept for reference / A-B.
DETERMINISTIC per seed: every random number is a SplitMix64 hash of (seed, cell row, cell col, salt) -> the same seed gives the same field
on every host and python (no numpy RNG); `checksum(seed)` = sha1 of the field's geometry text, printed at launch and stored in the exam
record (stumble.field.checksum).

The start pose: base 45 cm before the near edge, facing +x (the toes ~30 cm from the edge), on the plain plane -- the 3 s settle is on
the same floor as every other test, the first step lands among the bumps.

Use:
  python3 stumble_field.py --base <scene.xml> --out <scene_stumbleN.xml> --seed N     # writes the sibling scene (atomic), prints the checksum
  python3 stumble_field.py --seed N --checksum                                         # the checksum only
  from stumble_field import Field; f = Field(seed); f.height(x, y)                       # ground height (m) above the plane under (x, y); 0 off the field
"""
import argparse
import hashlib
import math
import os

KIND = os.environ.get("STUMBLE_KIND", "bumps")
X0, X1 = 50.0, 64.0          # the field along x (m): 14 m -- 0.15 x 15 s + 0.30 x 15 s = 6.75 m of walking + heading drift room
Y0, Y1 = -6.0, 6.0           # across (m): room for a policy that drifts sideways / turns
EDGE_FLAT = 0.10             # the first strip (along x, at the near edge) is flat
START = (X0 - 0.45, 0.0, 0.0)   # base x y yaw_deg before the near edge, facing +x
GEOM = "stumble_field"
# bumps
SP = 0.20                    # one sphere per SP x SP cell
JIT = 0.08                   # +- jitter of the centre inside the cell (m)
R = 0.15                     # sphere radius (m)
H_MIN, H_MAX = 0.004, float(os.environ.get("STUMBLE_AMP", "0.02"))   # cap heights (m); STUMBLE_AMP = the top (a dev / calibration knob, both ends read it)
# hfield
CELL = 0.05                  # sample spacing (m)
TILE = 0.10                  # tile = 2 x 2 samples share one height
AMP = float(os.environ.get("STUMBLE_AMP", "0.02"))   # heights in [0, AMP] m (+-1 cm about the mean)
_M = (1 << 64) - 1


def _mix(z):
    """SplitMix64 finaliser: 64-bit int -> 64-bit int"""
    z = (z + 0x9E3779B97F4A7C15) & _M
    z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & _M
    z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & _M
    return z ^ (z >> 31)


def tile_u(seed, ti, tj, salt=0):
    """uniform [0, 1) of cell (ti along y, tj along x) for a seed and a salt"""
    return _mix(((int(seed) & 0xFFFFFFFF) << 32) ^ ((salt & 0xFF) << 40) ^ ((ti & 0xFFF) << 12) ^ (tj & 0xFFF)) / float(1 << 64)


class Field:
    def __init__(self, seed, kind=KIND):
        self.seed = int(seed); self.kind = kind
        if kind == "bumps":
            self._build_bumps()
        elif kind == "hfield":
            self._build_hfield()
        else:
            raise ValueError(f"STUMBLE_KIND {kind!r}: bumps | hfield")

    # ── bumps ──
    def _build_bumps(self):
        self.ncx, self.ncy = int(round((X1 - X0) / SP)), int(round((Y1 - Y0) / SP))
        self.bumps = []            # (cx, cy, h)
        self.cells = {}            # (ti, tj) -> [bump index]
        for ti in range(self.ncy):
            for tj in range(self.ncx):
                h = H_MIN + (H_MAX - H_MIN) * tile_u(self.seed, ti, tj, 0)
                cx = X0 + (tj + 0.5) * SP + (2 * tile_u(self.seed, ti, tj, 1) - 1) * JIT
                cy = Y0 + (ti + 0.5) * SP + (2 * tile_u(self.seed, ti, tj, 2) - 1) * JIT
                rim = math.sqrt(max(2 * R * h - h * h, 0.0))
                if cx - rim < X0 + EDGE_FLAT or cx + rim > X1 or cy - rim < Y0 or cy + rim > Y1:
                    continue
                self.cells.setdefault((ti, tj), []).append(len(self.bumps))
                self.bumps.append((round(cx, 4), round(cy, 4), round(h, 5)))

    def _height_bumps(self, x, y):
        tj, ti = int((x - X0) / SP), int((y - Y0) / SP)
        z = 0.0
        for i in (ti - 1, ti, ti + 1):
            for j in (tj - 1, tj, tj + 1):
                for b in self.cells.get((i, j), ()):
                    cx, cy, h = self.bumps[b]
                    r2 = (x - cx) ** 2 + (y - cy) ** 2
                    if r2 < R * R:
                        z = max(z, h - R + math.sqrt(R * R - r2))
        return z

    def _text_bumps(self):
        return "\n".join(f"{cx:.4f} {cy:.4f} {h:.5f}" for cx, cy, h in self.bumps)

    # ── hfield ──
    def _build_hfield(self):
        self.ncol = int(round((X1 - X0) / CELL)) + 1     # along x
        self.nrow = int(round((Y1 - Y0) / CELL)) + 1     # along y
        per = int(round(TILE / CELL))
        nt_i, nt_j = (self.nrow + per - 1) // per, (self.ncol + per - 1) // per
        edge = int(round(EDGE_FLAT / CELL))
        rows = []
        for i in range(self.nrow):
            ti = i // per
            row = []
            for j in range(self.ncol):
                tj = j // per
                if j <= edge:
                    u = 0.0
                elif ti == 0 and tj == nt_j - 1:
                    u = 0.0          # pins MuJoCo's [0, 1] normalisation: min exactly 0 ...
                elif ti == nt_i - 1 and tj == nt_j - 1:
                    u = 1.0          # ... and max exactly 1 (the two far corners)
                else:
                    u = tile_u(self.seed, ti, tj)
                row.append(u)
            rows.append(row)
        self.rows = rows            # rows[i][j], i along y, j along x -- in [0, 1]

    def _text_hfield(self):
        # MuJoCo stores the hfield row-major with ROW 0 at +y (the top of an image) and COLUMN 0 at -x (mj_ray check 2026-11-02)
        return "\n".join(" ".join(f"{u:.5f}" for u in self.rows[i]) for i in range(self.nrow - 1, -1, -1))

    def _height_hfield(self, x, y):
        fx, fy = (x - X0) / CELL, (y - Y0) / CELL
        j0, i0 = min(int(fx), self.ncol - 2), min(int(fy), self.nrow - 2)
        a, b = fx - j0, fy - i0
        r = self.rows
        u = (r[i0][j0] * (1 - a) + r[i0][j0 + 1] * a) * (1 - b) + (r[i0 + 1][j0] * (1 - a) + r[i0 + 1][j0 + 1] * a) * b
        return AMP * u

    # ── common ──
    @staticmethod
    def inside(x, y):
        return X0 <= x <= X1 and Y0 <= y <= Y1

    def height(self, x, y):
        """the ground height (m) above the plane under world (x, y); 0 off the field. bumps: the highest sphere cap under the point
        (exact); hfield: bilinear in the 5 cm grid (MuJoCo's triangles differ by < 3 mm)"""
        if not self.inside(x, y):
            return 0.0
        return self._height_bumps(x, y) if self.kind == "bumps" else self._height_hfield(x, y)

    def text(self):
        head = (f"kind {self.kind} sp {SP} jit {JIT} R {R} h {H_MIN} {H_MAX}" if self.kind == "bumps" else f"kind hfield amp {AMP:.4f} cell {CELL} tile {TILE}")
        return head + "\n" + (self._text_bumps() if self.kind == "bumps" else self._text_hfield())

    def checksum(self):
        return hashlib.sha1(self.text().encode()).hexdigest()[:12]

    def meta(self):
        m = {"kind": self.kind, "seed": self.seed, "x": [X0, X1], "y": [Y0, Y1], "edge_flat": EDGE_FLAT, "start": list(START), "checksum": self.checksum()}
        if self.kind == "bumps":
            m.update({"spacing": SP, "jitter": JIT, "radius": R, "h": [H_MIN, H_MAX], "n": len(self.bumps)})
        else:
            m.update({"cell": CELL, "tile": TILE, "amp": AMP, "nrow": self.nrow, "ncol": self.ncol})
        return m

    def scene_xml(self, base_scene):
        if self.kind == "bumps":
            body = "\n".join(f'    <geom name="{GEOM}_{i}" type="sphere" size="{R}" pos="{cx} {cy} {h - R:.5f}" solref="0.02 1" rgba="0.55 0.47 0.33 1" />'
                             for i, (cx, cy, h) in enumerate(self.bumps))
            what = f"{len(self.bumps)} sphere bumps (R {R} m, caps {H_MIN * 100:.1f}..{H_MAX * 100:.1f} cm, one per {SP * 100:.0f} cm cell)"
            asset = ""
        else:
            cx, cy = 0.5 * (X0 + X1), 0.5 * (Y0 + Y1); rx, ry = 0.5 * (X1 - X0), 0.5 * (Y1 - Y0)
            asset = (f'  <asset>\n    <hfield name="{GEOM}" nrow="{self.nrow}" ncol="{self.ncol}" size="{rx} {ry} {AMP} 0.1" elevation="\n{self._text_hfield()}\n" />\n  </asset>\n')
            body = f'    <geom name="{GEOM}" type="hfield" hfield="{GEOM}" pos="{cx} {cy} 0" solref="0.02 1" rgba="0.45 0.4 0.3 1" />'
            what = f"a height field, {TILE * 100:.0f} cm tiles, heights 0..{AMP * 100:.0f} cm"
        return (f'<mujoco model="stumble field seed {self.seed} over {base_scene}">\n'
                f'  <!-- GENERATED by mujoco_sim/tools/stumble_field.py (Exam #2 Test 5): {base_scene} + a {X1 - X0:.0f} x {Y1 - Y0:.0f} m rough floor\n'
                f'       at x {X0}..{X1}, y {Y0}..{Y1}: {what}, seed {self.seed}, checksum {self.checksum()}. World-body geoms (the sim\'s\n'
                f'       exam counters treat every world-body geom as floor). Do not edit, do not commit (gitignored): the launcher rewrites it per seed. -->\n'
                f'  <include file="{base_scene}" />\n'
                f'{asset}'
                f'  <worldbody>\n{body}\n  </worldbody>\n'
                f'</mujoco>\n')

    def write_scene(self, base_path, out_path):
        """the sibling scene next to the base scene (same dir: the include and the mesh paths resolve); atomic (parallel rigs write the same file)"""
        base = os.path.basename(base_path)
        assert os.path.dirname(os.path.abspath(out_path)) == os.path.dirname(os.path.abspath(base_path)), "the stumble scene must sit next to its base scene"
        tmp = f"{out_path}.{os.getpid()}.tmp"
        with open(tmp, "w") as f:
            f.write(self.scene_xml(base))
        os.replace(tmp, out_path)
        return out_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True); ap.add_argument("--base", default=""); ap.add_argument("--out", default="")
    ap.add_argument("--checksum", action="store_true", help="print the field checksum only")
    ap.add_argument("--start", action="store_true", help="print the start pose 'x y yaw_deg'")
    a = ap.parse_args()
    f = Field(a.seed)
    if a.start:
        print(f"{START[0]:.3f} {START[1]:.3f} {START[2]:.1f}"); return
    if a.checksum or not a.base:
        print(f.checksum()); return
    out = a.out or f"{a.base[:-4]}_stumble{a.seed}.xml"
    f.write_scene(a.base, out)
    m = f.meta()
    print(f"{out} kind {f.kind} seed {a.seed} checksum {f.checksum()} " + (f"{m['n']} bumps" if f.kind == "bumps" else f"{m['nrow']}x{m['ncol']} samples"))


if __name__ == "__main__":
    main()
