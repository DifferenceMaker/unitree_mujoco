#!/usr/bin/env python3
"""grasp_reward_ledger — LIVE per-term reward recomputation for the GRASP rig.

The balance line's reward_ledger pattern (gauges v2: fixed-order 'name:value:frac'
rows, frac = value/|weight| clamped [-1,1]) applied to the grasp economy. The sim
already renders rt/balance_metrics as the bottom-left ledger panel
(policy_hud::set_metrics_from_json) — same envelope, zero sim changes.

Inputs (host-side, sim DDS domain):
    rt/sim_hand/state   obj/torso/palm world poses, 17 pad forces, closure(6),
                        obj_lift (grasp_sim.h)
    rt/lowstate         arm joint q/dq (quiet_hold / park_keep)
Weights/params from the STAGED env.yaml (the policy's own economy).

Terms (~ = approximated vs Isaac):
    hold_cube        exp(-(|palm-obj|/sigma)^2) * touching
    cube_hold_above  income * lift ramp (obj_lift vs ramp_lo/hi) * touching
    approach_cube    exp(-(d/sigma)^2) * (1-touching)
    cube_orientation_hold~  exp(-(ang/sigma)^2) vs FIRST-SEEN quat * touching * ramp
    quiet_hold~      exp(-sum(qd^2)/sigma^2) * touching      (arm qd from lowstate)
    park_keep~       exp(-|q-park|^2/sigma^2)                (park from deploy.yaml offset)
    pad_arrangement~ loaded-pad count / 6 * weight-sign      (crude count proxy)
Contact penalties (table_hit/torso_hit/crush) need contact-pair data the state
stream doesn't carry — omitted, the HUD shows the income economy.
"""
import json
import math
import os
import re
import sys
import time

import numpy as np

sys.path.insert(0, os.environ.get("SDK_PATH", os.path.expanduser(
    "~/Projects/robot_projects/repos/unitree_sdk2_python")))
from unitree_sdk2py.core.channel import (ChannelFactoryInitialize,   # noqa: E402
                                         ChannelPublisher, ChannelSubscriber)
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_             # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_         # noqa: E402


def _num(text, pat, default):
    m = re.search(pat, text)
    return float(m.group(1)) if m else default


class GraspLedger:
    def __init__(self, env_yaml, deploy_yaml):
        s = open(env_yaml).read()

        def wt(term):
            m = re.search(rf"  {re.escape(term)}:\n(?:.*\n){{0,14}}?    weight: (-?[\d.eE+]+)", s)
            return float(m.group(1)) if m else None

        def par(term, key, default):
            i = s.find(f"  {term}:")
            return _num(s[i:i + 900], rf"{key}: (-?[\d.eE+]+)", default) if i >= 0 else default

        self.obj = "cube"
        for cand in ("tube", "ring"):
            if f"{cand}_d180" in s:
                self.obj = cand
        pre = {"cube": "cube", "tube": "tube", "ring": "ring"}[self.obj]
        # term names are per-object in gr8b+ (hold_cube stays cube-named for cube only)
        self.terms = {}
        for t, generic in ((f"hold_cube", "hold"), (f"{pre}_hold_above", "hold_above"),
                           (f"approach_{pre}", "approach"),
                           (f"{pre}_orientation_hold", "orient"),
                           ("cube_orientation_hold", "orient"),
                           ("cube_hold_above", "hold_above"), ("approach_cube", "approach"),
                           ("quiet_hold", "quiet"), ("park_keep", "park"),
                           ("pad_arrangement", "pads")):
            w = wt(t)
            if w is not None and generic not in {v[1] for v in self.terms.values()}:
                self.terms[t] = (w, generic)
        self.p = {
            "hold_sigma": par("hold_cube", "sigma", 0.06),
            "above_lo": par(f"{pre}_hold_above" if wt(f"{pre}_hold_above") else "cube_hold_above", "ramp_lo", 0.025),
            "above_hi": par(f"{pre}_hold_above" if wt(f"{pre}_hold_above") else "cube_hold_above", "ramp_hi", 0.05),
            "appr_sigma": par(f"approach_{pre}" if wt(f"approach_{pre}") else "approach_cube", "sigma", 0.3),
            "orient_sigma": par(f"{pre}_orientation_hold" if wt(f"{pre}_orientation_hold") else "cube_orientation_hold", "sigma", 0.7),
            "quiet_sigma": par("quiet_hold", "sigma", 3.0),
            "park_sigma": par("park_keep", "sigma", 1.7),
        }
        d = open(deploy_yaml).read()
        m = re.search(r"offset:\n((?:\s+- .+\n){7})", d)
        self.park = [float(x) for x in re.findall(r"- (-?[\d.eE+]+)", m.group(1))] if m else None
        self._order = sorted(self.terms, key=lambda t: -abs(self.terms[t][0]))
        self._ref_q = None
        print(f"[grasp_ledger] object={self.obj}; terms: " +
              ", ".join(f"{t}({self.terms[t][0]:+g})" for t in self._order), flush=True)

    def tick(self, st, arm_q, arm_dq):
        obj = np.asarray(st["obj"]["p"]); palm = np.asarray(st["palm"]["p"])
        pads = np.asarray(list(st["pads"].values()), dtype=float)
        touching = 1.0 if pads.sum() > 1.0 else 0.0
        lift = float(st.get("obj_lift", 0.0))
        lo, hi = self.p["above_lo"], self.p["above_hi"]
        ramp = min(1.0, max(0.0, (lift - lo) / max(1e-6, hi - lo)))
        d = float(np.linalg.norm(obj - palm))
        vals = {}
        for t, (w, g) in self.terms.items():
            if g == "hold":
                v = math.exp(-(d / self.p["hold_sigma"]) ** 2) * touching
            elif g == "hold_above":
                v = touching * ramp
            elif g == "approach":
                v = math.exp(-(d / self.p["appr_sigma"]) ** 2) * (1.0 - touching)
            elif g == "orient":
                q = np.asarray(st["obj"]["q"], dtype=float)
                if self._ref_q is None:
                    self._ref_q = q.copy()
                dot = min(1.0, abs(float(np.dot(self._ref_q, q))))
                ang = 2.0 * math.acos(dot)
                v = math.exp(-(ang / self.p["orient_sigma"]) ** 2) * touching * ramp
            elif g == "quiet":
                v = (math.exp(-float(np.sum(np.square(arm_dq))) / self.p["quiet_sigma"] ** 2)
                     * touching) if arm_dq is not None else 0.0
            elif g == "park":
                if arm_q is None or self.park is None:
                    v = 0.0
                else:
                    dev = float(np.sum(np.square(np.asarray(arm_q) - np.asarray(self.park))))
                    v = math.exp(-dev / self.p["park_sigma"] ** 2)
            elif g == "pads":
                v = float((pads > 0.5).sum()) / 6.0
            else:
                v = 0.0
            vals[t] = v * w
        return vals

    def hud_field(self, vals, top=10):
        total = sum(vals.values())
        rows = [f"TOTAL:{total:+.2f}:{max(-1.0, min(1.0, total / 60.0)):+.3f}"]
        for t in self._order[:top]:
            if t not in vals:
                continue
            w = abs(self.terms[t][0]) or 1.0
            disp = (t + "~") if t in ("quiet_hold", "park_keep", "pad_arrangement",
                                      "cube_orientation_hold") or "orientation" in t else t
            frac = max(-1.0, min(1.0, vals[t] / w))
            rows.append(f"{disp[:18]}:{vals[t]:+.2f}:{frac:+.3f}")
        return "|".join(rows)


def main():
    dom = int(os.environ.get("SIM_DDS_DOMAIN", "1"))
    nic = os.environ.get("SIM_DDS_NIC", "lo")
    env_yaml = sys.argv[1]
    deploy_yaml = sys.argv[2]
    ChannelFactoryInitialize(dom, nic)
    led = GraspLedger(env_yaml, deploy_yaml)
    state = {"st": None, "arm_q": None, "arm_dq": None}

    def on_state(msg):
        try:
            state["st"] = json.loads(msg.data)
        except Exception:
            pass

    def on_low(m):
        state["arm_q"] = [m.motor_state[i].q for i in range(20, 27)]
        state["arm_dq"] = [m.motor_state[i].dq for i in range(20, 27)]

    ChannelSubscriber("rt/sim_hand/state", String_).Init(on_state, 10)
    ChannelSubscriber("rt/lowstate", LowState_).Init(on_low, 10)
    pub = ChannelPublisher("rt/balance_metrics", String_)
    pub.Init()
    print(f"[grasp_ledger] publishing the grasp reward ledger on rt/balance_metrics "
          f"(domain {dom}/{nic}) — sim panel: same key as the balance ledger", flush=True)
    while True:
        time.sleep(1.0 / 20.0)
        if state["st"] is None:
            continue
        try:
            vals = led.tick(state["st"], state["arm_q"], state["arm_dq"])
            pub.Write(String_(data='{"ledger":"%s"}' % led.hud_field(vals)))
        except Exception as e:
            print(f"[grasp_ledger] tick error: {e}", flush=True)
            time.sleep(1.0)


if __name__ == "__main__":
    main()
