#!/usr/bin/env python
"""reward_oracle — the ISAAC reward functions, evaluated on MUJOCO state (2026-09-10).

Operator design: "the terms are calculated via the same principle that the Isaac
rewards are, but just run in the sidecar of MuJoCo, and the data provided is the
MuJoCo data... in case something can't be calculated or isn't permitted in MuJoCo
it gets displayed as n/a."

Replaces reward_ledger.py's hand-written twin of 16 terms. Nothing is
re-implemented here: every reward row of the active policy's params/env.yaml is
evaluated by calling THE function that trained it (the `func:` string resolves to
the exact module:function — unitree_rl_lab.tasks.locomotion.mdp.rewards,
isaaclab.envs.mdp, isaaclab_tasks ...), with the run's own weight and params.
The only thing swapped is the data source: `MjEnv` presents MuJoCo state under
the names the functions read (env.scene["robot"].data.root_pos_w, joint_pos,
body_pos_w, env.scene.sensors["contact_forces"].data.net_forces_w_history,
env.command_manager.get_command(...), env.spawn_root_xy ...). A term whose
input the rig does not export, or whose Isaac object cannot be reproduced,
renders as `n/a` with the reason (printed once to the log).

PROCESS MODEL. This runs in the ISAACSIM conda env (torch + isaaclab source; no
SimulationApp — the kit-only modules pxr/omni/carb are stubbed by a meta-path
finder; `import isaaclab.envs` FIRST breaks the managers<->envs import cycle).
That env has no DDS, and the tv env that owns DDS cannot import isaaclab, so
the sidecar (balance_metrics.py --oracle PORT, via oracle_bridge.py) forwards
one JSON state datagram per tick over localhost UDP and gets the HUD row string
back. Protocol:
    bridge -> oracle : {"type":"meta", "joint_names_sdk":[27], "jnt_range":[[lo,hi]*27],
                        "body_names":[nbody]}               (once, and every ~5 s)
    bridge -> oracle : {"type":"state", "t":..., "q":[27], "dq":[27], "tau":[27],
                        "quat":[w,x,y,z], "gyro":[3], "pose":{sim_base_pose json}|null,
                        "anchor":[x,y,z]|null, "qcmd":[27]|null, "lean":float|null}
    oracle -> bridge : "TOTAL:v:f|name:v:f|..."   (policy_hud.h set_ledger format)

sim_base_pose v3 (2026-08-21 build): p,q,v,w,ucnt,umax,th,fc,lw,rw,fl,fr,tl,tr.
sim_base_pose v4 (2026-09-10 build): + "xb","vb","wb" (nbody x 3, world) and
"cf" (nbody x 3 net contact force, world) — the inputs the contact and body
terms need. Without v4 those terms are n/a (never silently zero).

Values are WEIGHTED per second (weight * value) like the old ledger; Isaac's
extra * step_dt is left out so the gauges keep the scale the operator knows.
"""
from __future__ import annotations

import argparse
import importlib
import importlib.abc
import importlib.machinery
import json
import math
import os
import re
import signal
import socket
import sys
import time
import traceback
import types

# ─────────────────────────── kit-module stubs + Isaac import ───────────────────────────
_STUB_ROOTS = ("pxr", "omni", "carb", "isaacsim", "warp", "usdrt", "Semantics")


class _StubModule(types.ModuleType):
    __path__: list = []  # package-like: `import omni.kit.app` resolves

    def __getattr__(self, k):
        if k.startswith("__") and k.endswith("__"):
            raise AttributeError(k)
        v = _StubModule(f"{self.__name__}.{k}")
        setattr(self, k, v)
        return v

    def __call__(self, *a, **kw):
        return _StubModule(self.__name__ + "()")

    def __mro_entries__(self, bases):
        return (object,)

    def __iter__(self):
        return iter(())

    def __bool__(self):
        return False


class _StubFinder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    def find_spec(self, name, path=None, target=None):
        if name.split(".")[0] in _STUB_ROOTS:
            return importlib.machinery.ModuleSpec(name, self, is_package=True)
        return None

    def create_module(self, spec):
        return _StubModule(spec.name)

    def exec_module(self, module):
        pass


def import_isaac(rl_lab_repo: str, isaaclab_root: str):
    sys.meta_path.insert(0, _StubFinder())
    for p in (os.path.join(isaaclab_root, "isaaclab"), os.path.join(isaaclab_root, "isaaclab_tasks"),
              os.path.join(isaaclab_root, "isaaclab_rl"), os.path.join(isaaclab_root, "isaaclab_assets"),
              os.path.join(rl_lab_repo, "source", "unitree_rl_lab")):
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)
    import isaaclab.envs  # noqa: F401  (FIRST — breaks the managers<->envs cycle)
    from unitree_rl_lab.tasks.locomotion import mdp  # noqa: F401  (registers every reward module)
    return mdp


# ─────────────────────────── env.yaml / deploy.yaml ───────────────────────────
def _load_yaml_tolerant(path: str):
    import yaml

    class L(yaml.SafeLoader):
        pass

    def _any(loader, suffix, node):
        if isinstance(node, yaml.MappingNode):
            return loader.construct_mapping(node)
        if isinstance(node, yaml.SequenceNode):
            return loader.construct_sequence(node)
        return loader.construct_scalar(node)

    L.add_multi_constructor("tag:yaml.org,2002:python/", _any)
    with open(path) as f:
        return yaml.load(f, Loader=L)


def _is_slice_repr(v) -> bool:
    return isinstance(v, (list, tuple)) and len(v) == 3 and all(x is None for x in v)


def _is_entity_cfg(v) -> bool:
    return isinstance(v, dict) and "name" in v and ("joint_ids" in v or "body_ids" in v)


# ─────────────────────────── MuJoCo-backed Isaac data surface ───────────────────────────
class _Bag:
    """Attribute bag (ArticulationData / ContactSensorData stand-in)."""


class MjArticulation:
    def __init__(self, joint_names: list[str], body_names: list[str], torch):
        from isaaclab.utils.string import resolve_matching_names
        self._rmn = resolve_matching_names
        self.joint_names = list(joint_names)
        self.body_names = list(body_names)
        self.num_joints = len(joint_names)
        self.num_bodies = len(body_names)
        self.fixed_tendon_names: list[str] = []
        self.data = _Bag()
        self._torch = torch

    def find_joints(self, name_keys, joint_subset=None, preserve_order=False):
        return self._rmn(name_keys, self.joint_names if joint_subset is None else joint_subset, preserve_order)

    def find_bodies(self, name_keys, preserve_order=False):
        return self._rmn(name_keys, self.body_names, preserve_order)


class MjContactSensor:
    """Isaac ContactSensor semantics on per-body net forces: history roll,
    contact/air timers (contact_sensor.py _update_buffers_impl), force_threshold."""

    def __init__(self, body_names: list[str], history_length: int, force_threshold: float,
                 track_air_time: bool, torch):
        from isaaclab.utils.string import resolve_matching_names
        self._rmn = resolve_matching_names
        self.body_names = list(body_names)
        self.num_bodies = len(body_names)
        self.cfg = types.SimpleNamespace(history_length=max(1, int(history_length)),
                                         force_threshold=float(force_threshold),
                                         track_air_time=bool(track_air_time))
        B, H = self.num_bodies, self.cfg.history_length
        d = self.data = _Bag()
        d.net_forces_w = torch.zeros(1, B, 3)
        d.net_forces_w_history = torch.zeros(1, H, B, 3)
        d.current_contact_time = torch.zeros(1, B)
        d.current_air_time = torch.zeros(1, B)
        d.last_contact_time = torch.zeros(1, B)
        d.last_air_time = torch.zeros(1, B)
        # bodies whose force is REALLY measured (v3 export: feet only; v4: all)
        self.known = torch.zeros(B, dtype=torch.bool)
        self._torch = torch

    def find_bodies(self, name_keys, preserve_order=False):
        return self._rmn(name_keys, self.body_names, preserve_order)

    def update(self, forces_w, known_mask, dt: float):
        t, d = self._torch, self.data
        d.net_forces_w = forces_w.reshape(1, -1, 3)
        d.net_forces_w_history = d.net_forces_w_history.roll(1, dims=1)
        d.net_forces_w_history[:, 0] = d.net_forces_w
        self.known = known_mask
        if self.cfg.track_air_time:
            is_contact = t.norm(d.net_forces_w, dim=-1) > self.cfg.force_threshold
            first_contact = (d.current_air_time > 0) & is_contact
            first_detached = (d.current_contact_time > 0) & ~is_contact
            d.current_contact_time = t.where(is_contact, d.current_contact_time + dt, t.zeros_like(d.current_contact_time))
            d.last_air_time = t.where(first_contact, d.current_air_time + dt, d.last_air_time)
            d.current_air_time = t.where(~is_contact, d.current_air_time + dt, t.zeros_like(d.current_air_time))
            d.last_contact_time = t.where(first_detached, d.current_contact_time + dt, d.last_contact_time)


class MjScene:
    def __init__(self, robot: MjArticulation, sensors: dict):
        self._ent = {"robot": robot}
        self.sensors = sensors

    def keys(self):
        return list(self._ent) + list(self.sensors)

    def __getitem__(self, name):
        if name in self._ent:
            return self._ent[name]
        if name in self.sensors:
            return self.sensors[name]
        raise KeyError(f"scene entity '{name}' is not reproduced in MuJoCo")


class MjArmPoseCommand:
    """The slice of IKArmPoseCommand that desk_reach_bonus reads."""

    def __init__(self, robot: MjArticulation, torch):
        self._torch = torch
        self.ee_body_idx = {}
        for side in ("left", "right"):
            ids, _ = robot.find_bodies(f"{side}_wrist_yaw_link")
            self.ee_body_idx[side] = ids[0] if ids else 0
        self.wish_w = {s: torch.zeros(1, 3) for s in ("left", "right")}
        self.desk_wish_mask = {s: torch.zeros(1, dtype=torch.bool) for s in ("left", "right")}
        self.default_mode = torch.zeros(1, dtype=torch.bool)

    def set_targets(self, tl, tr):
        for side, tgt in (("left", tl), ("right", tr)):
            vis = bool(tgt is not None and len(tgt) == 3 and tgt[2] > 0.0)
            self.desk_wish_mask[side][0] = vis
            if vis:
                self.wish_w[side][0] = self._torch.tensor(tgt, dtype=self._torch.float32)


class MjCommandManager:
    def __init__(self, robot: MjArticulation, torch):
        self._torch = torch
        self.base_velocity = torch.zeros(1, 3)
        self.lean = torch.zeros(1, 1)
        self.lean_known = False
        self.arm = MjArmPoseCommand(robot, torch)
        self.arm_cmd14 = torch.zeros(1, 14)

    def get_command(self, name: str):
        if name == "base_velocity":
            return self.base_velocity
        if name == "lean_command":
            return self.lean
        if name == "arm_pose_command":
            return self.arm_cmd14
        raise KeyError(f"command '{name}' is not reproduced in MuJoCo")

    def get_term(self, name: str):
        if name == "arm_pose_command":
            return self.arm
        raise KeyError(f"command term '{name}' is not reproduced in MuJoCo")


class MjActionManager:
    def __init__(self, n_act: int, torch):
        self.action = torch.zeros(1, n_act)
        self.prev_action = torch.zeros(1, n_act)


class MjEnv:
    """Attribute-open env stand-in (reward functions set env._fivp_* etc. lazily)."""

    def __init__(self, scene, cmd, act, step_dt: float, torch):
        self.num_envs = 1
        self.device = torch.device("cpu")
        self.scene = scene
        self.command_manager = cmd
        self.action_manager = act
        self.step_dt = float(step_dt)
        self.episode_length_buf = torch.zeros(1, dtype=torch.long)
        self.common_step_counter = 0
        self.reset_buf = torch.zeros(1, dtype=torch.bool)
        self.spawn_root_xy = torch.zeros(1, 2)
        self.spawn_yaw = torch.zeros(1)
        self.spawn_foot_pos = torch.zeros(1, 2, 2)
        # is_alive reads env.termination_manager.terminated; the rig never terminates
        # an episode (the robot is either standing or the operator resets), so alive
        # is the constant income it is in a surviving Isaac step.
        self.termination_manager = types.SimpleNamespace(terminated=torch.zeros(1, dtype=torch.bool),
                                                         time_outs=torch.zeros(1, dtype=torch.bool))


# ─────────────────────────── the oracle ───────────────────────────
def _quat_to_R(q):
    w, x, y, z = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def _yaw(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class RewardOracle:
    def __init__(self, env_yaml: str, deploy_yaml: str | None, mdp, torch, verbose=True):
        self.torch = torch
        self.mdp = mdp
        self.verbose = verbose
        self.env_yaml = env_yaml
        deploy_yaml = deploy_yaml or os.path.join(os.path.dirname(env_yaml), "deploy.yaml")
        self.E = _load_yaml_tolerant(env_yaml)
        self.D = _load_yaml_tolerant(deploy_yaml) if os.path.isfile(deploy_yaml) else {}
        self.step_dt = float(self.D.get("step_dt", 0.02))
        self.jmap = [int(i) for i in self.D["joint_ids_map"]]           # isaac idx -> sdk idx
        self.q_default_isaac = [float(x) for x in self.D["default_joint_pos"]]
        act = (self.D.get("actions") or {}).get("JointPositionAction") or {}
        self.act_ids = [int(i) for i in act.get("joint_ids", [])]
        self.act_scale = [float(x) for x in act.get("scale", [])]
        self.act_offset = [float(x) for x in act.get("offset", [])]
        sc = self.E.get("scene") or {}
        cs = sc.get("contact_forces") or {}
        self.hist_len = int(cs.get("history_length", 3) or 3)
        self.force_thr = float(cs.get("force_threshold", 1.0) or 1.0)
        self.track_air = bool(cs.get("track_air_time", True))
        rb = sc.get("robot") or {}
        self.soft_factor = float(rb.get("soft_joint_pos_limit_factor", 0.9) or 0.9)
        # anchor fwd offset: from the anchor obs / anchor_hold params (0.5 default)
        self.anchor_fwd = 0.5
        for grp in ("policy", "critic"):
            t = ((self.E.get("observations") or {}).get(grp) or {}).get("anchor_point")
            if isinstance(t, dict) and (t.get("params") or {}).get("fwd_offset") is not None:
                self.anchor_fwd = float(t["params"]["fwd_offset"])
        self.terms = []  # [(name, weight, func_str, params_raw)]
        for name, spec in (self.E.get("rewards") or {}).items():
            if spec is None:
                continue
            w = spec.get("weight")
            if w is None or float(w) == 0.0:
                continue
            self.terms.append((name, float(w), str(spec.get("func")), spec.get("params") or {}))
        self.terms.sort(key=lambda t: -abs(t[1]))
        self.order = [t[0] for t in self.terms]
        self.meta = None
        self.env = None
        self.robot = None
        self.sensor = None
        self._fns = {}
        self._params = {}
        self._na_reason = {}
        self._printed = set()
        self._t_prev = None
        self._dq_prev = None
        self._xb_prev = None
        self._spawned = False
        self._v4 = False
        self.tape = []
        self.n_ticks = 0

    # ---- meta: names/limits from the bridge (the sidecar owns the MJCF) ----
    def set_meta(self, meta: dict, scene_bodies: list | None = None):
        """Build the adapter. Body list = the SIM's scene bodies when the v4 message
        carries "bn" (desk scene = robot + desk + balls, 33), else the sidecar's
        robot-MJCF list from meta (29). Names, not counts, are the contract."""
        t = self.torch
        sdk_names = list(meta["joint_names_sdk"])
        if len(sdk_names) != 27 or max(self.jmap) >= 27:
            raise ValueError(f"expected 27 SDK joints, got {len(sdk_names)}")
        isaac_names = [sdk_names[i] for i in self.jmap]
        body_names = list(scene_bodies) if scene_bodies else list(meta["body_names"])
        self._scene_bodies = list(scene_bodies) if scene_bodies else None
        self.body_names = body_names
        self.robot = MjArticulation(isaac_names, body_names, t)
        self.sensor = MjContactSensor(body_names, self.hist_len, self.force_thr, self.track_air, t)
        scene = MjScene(self.robot, {"contact_forces": self.sensor})
        cmd = MjCommandManager(self.robot, t)
        act = MjActionManager(len(self.act_ids), t)
        self.env = MjEnv(scene, cmd, act, self.step_dt, t)
        d = self.robot.data
        J, B = 27, len(body_names)
        rng = meta.get("jnt_range")
        lo = t.tensor([[rng[s][0] for s in self.jmap]], dtype=t.float32) if rng else t.full((1, J), -math.inf)
        hi = t.tensor([[rng[s][1] for s in self.jmap]], dtype=t.float32) if rng else t.full((1, J), math.inf)
        mid, half = 0.5 * (lo + hi), 0.5 * (hi - lo)
        d.joint_pos_limits = t.stack([lo, hi], dim=-1)
        d.soft_joint_pos_limits = t.stack([mid - half * self.soft_factor, mid + half * self.soft_factor], dim=-1)
        d.default_joint_pos = t.tensor([self.q_default_isaac], dtype=t.float32)
        d.joint_pos = d.default_joint_pos.clone()
        d.joint_vel = t.zeros(1, J)
        d.joint_acc = t.zeros(1, J)
        d.applied_torque = t.zeros(1, J)
        d.root_pos_w = t.tensor([[0.0, 0.0, math.nan]])
        d.root_quat_w = t.tensor([[1.0, 0.0, 0.0, 0.0]])
        d.root_lin_vel_w = t.zeros(1, 3)
        d.root_ang_vel_w = t.zeros(1, 3)
        d.root_lin_vel_b = t.zeros(1, 3)
        d.root_ang_vel_b = t.zeros(1, 3)
        d.projected_gravity_b = t.tensor([[0.0, 0.0, -1.0]])
        d.body_pos_w = t.full((1, B, 3), math.nan)
        d.body_quat_w = t.zeros(1, B, 4)
        d.body_quat_w[..., 0] = 1.0
        d.body_lin_vel_w = t.full((1, B, 3), math.nan)
        d.body_ang_vel_w = t.full((1, B, 3), math.nan)
        self.b_idx = {n: i for i, n in enumerate(body_names)}
        # resolve every term's callable + SceneEntityCfg params now (once)
        from isaaclab.managers import SceneEntityCfg
        self._fns.clear()
        self._params.clear()
        self._na_reason.clear()
        for name, w, func, praw in self.terms:
            try:
                mod, fn = func.split(":")
                self._fns[name] = getattr(importlib.import_module(mod), fn)
            except Exception as e:  # noqa: BLE001
                self._na_reason[name] = f"func {func}: {type(e).__name__}: {e}"
                continue
            params = {}
            try:
                for k, v in praw.items():
                    if _is_entity_cfg(v):
                        kw = {kk: vv for kk, vv in v.items() if kk in ("name", "joint_names", "body_names",
                                                                        "fixed_tendon_names", "object_collection_names",
                                                                        "preserve_order")}
                        cfg = SceneEntityCfg(**kw)
                        for fld in ("joint_ids", "body_ids", "fixed_tendon_ids", "object_collection_ids"):
                            val = v.get(fld)
                            if val is not None and not _is_slice_repr(val):
                                setattr(cfg, fld, val)
                        cfg.resolve(scene)
                        params[k] = cfg
                    else:
                        params[k] = v
                self._params[name] = params
            except Exception as e:  # noqa: BLE001
                self._na_reason[name] = f"params: {type(e).__name__}: {e}"
        self.meta = meta
        self._v4 = False
        if self.verbose:
            print(f"[ORACLE] {len(self.terms)} weighted terms from {os.path.basename(os.path.dirname(os.path.dirname(self.env_yaml)))}"
                  f"/params/env.yaml; {len(self._fns)} callables resolved; contact sensor H={self.hist_len} thr={self.force_thr}", flush=True)
            for n, r in self._na_reason.items():
                print(f"[ORACLE] n/a {n}: {r}", flush=True)

    # ---- per-tick state ----
    def update(self, s: dict):
        t, d, env = self.torch, self.robot.data, self.env
        now = float(s.get("t", time.monotonic()))
        dt = self.step_dt if self._t_prev is None else max(1e-3, min(0.1, now - self._t_prev))
        self._t_prev = now
        J = 27
        q_sdk, dq_sdk, tau_sdk = s["q"], s["dq"], s.get("tau") or [0.0] * J
        q_i = [q_sdk[i] for i in self.jmap]
        dq_i = [dq_sdk[i] for i in self.jmap]
        tau_i = [tau_sdk[i] for i in self.jmap]
        d.joint_pos = t.tensor([q_i], dtype=t.float32)
        dq_new = t.tensor([dq_i], dtype=t.float32)
        d.joint_acc = (dq_new - self._dq_prev) / dt if self._dq_prev is not None else t.zeros(1, J)
        self._dq_prev = dq_new
        d.joint_vel = dq_new
        d.applied_torque = t.tensor([tau_i], dtype=t.float32)
        pose = s.get("pose") or None
        self.have_pose = pose is not None and "p" in pose and "q" in pose
        if self.have_pose:
            p, q, v, w = pose["p"], pose["q"], pose.get("v", [0, 0, 0]), pose.get("w", [0, 0, 0])
            R = _quat_to_R(q)
            Rt = t.tensor(R, dtype=t.float32)
            d.root_pos_w = t.tensor([p], dtype=t.float32)
            d.root_quat_w = t.tensor([q], dtype=t.float32)
            d.root_lin_vel_w = t.tensor([v], dtype=t.float32)
            wb = t.tensor([w], dtype=t.float32)
            d.root_ang_vel_b = wb
            d.root_ang_vel_w = (Rt @ wb.T).T
            d.root_lin_vel_b = (Rt.T @ d.root_lin_vel_w.T).T
            d.projected_gravity_b = (Rt.T @ t.tensor([[0.0], [0.0], [-1.0]])).T
            bn = pose.get("bn")
            if bn and bn != getattr(self, "_scene_bodies", None) and len(bn) == len(pose.get("xb", [])):
                print(f"[ORACLE] v4 export detected: {len(bn)} scene bodies "
                      f"({', '.join(x for x in bn if x not in self.body_names) or 'same as meta'} beyond the robot); "
                      f"rebuilding the adapter on the scene's body list", flush=True)
                self.set_meta(self.meta, scene_bodies=bn)
                d, env = self.robot.data, self.env
                d.joint_pos = t.tensor([q_i], dtype=t.float32)
                d.joint_vel = dq_new
                d.applied_torque = t.tensor([tau_i], dtype=t.float32)
                self._spawned = False
                self._xb_prev = None
            B = len(self.body_names)
            if "xb" in pose and len(pose["xb"]) != B and not bn:
                if "xb-mismatch" not in self._printed:
                    self._printed.add("xb-mismatch")
                    print(f"[ORACLE] v4 arrays have {len(pose['xb'])} bodies but the body list has {B} and the "
                          f"message carries no \"bn\" names — rebuild simulate (2026-09-10 build adds bn); using v3 path", flush=True)
            if "xb" in pose and len(pose["xb"]) == B:          # v4 export
                self._v4 = True
                d.body_pos_w = t.tensor([pose["xb"]], dtype=t.float32)
                d.body_lin_vel_w = t.tensor([pose.get("vb", [[math.nan] * 3] * B)], dtype=t.float32)
                d.body_ang_vel_w = t.tensor([pose.get("wb", [[math.nan] * 3] * B)], dtype=t.float32)
                forces = t.tensor(pose.get("cf", [[0.0] * 3] * B), dtype=t.float32)
                known = t.ones(B, dtype=t.bool)
            else:                                                 # v3: 5 bodies + feet forces
                xb = t.full((B, 3), math.nan)
                for key, bn in (("p", "pelvis"), ("lw", "left_wrist_yaw_link"), ("rw", "right_wrist_yaw_link"),
                                ("fl", "left_ankle_roll_link"), ("fr", "right_ankle_roll_link")):
                    if key in pose and bn in self.b_idx:
                        xb[self.b_idx[bn]] = t.tensor(pose[key], dtype=t.float32)
                vb = t.full((B, 3), math.nan)
                if self._xb_prev is not None:
                    vb = (xb - self._xb_prev) / dt
                self._xb_prev = xb.clone()
                d.body_pos_w = xb.unsqueeze(0)
                d.body_lin_vel_w = vb.unsqueeze(0)
                d.body_ang_vel_w = t.full((1, B, 3), math.nan)
                forces = t.zeros(B, 3)
                known = t.zeros(B, dtype=t.bool)
                fc = pose.get("fc")
                for k, bn in enumerate(("left_ankle_roll_link", "right_ankle_roll_link")):
                    if fc and bn in self.b_idx:
                        forces[self.b_idx[bn], 2] = float(fc[k])
                        known[self.b_idx[bn]] = True
            self.sensor.update(forces, known, dt)
            # arm targets (click balls) for desk_reach
            env.command_manager.arm.set_targets(pose.get("tl"), pose.get("tr"))
            # spawn frame: yaw captured at the first pose; home = anchor - fwd_offset
            yaw = _yaw(q)
            if not self._spawned:
                env.spawn_yaw = t.tensor([yaw], dtype=t.float32)
                env.spawn_root_xy = t.tensor([p[:2]], dtype=t.float32)
                fl, fr = pose.get("fl"), pose.get("fr")
                if fl and fr:
                    env.spawn_foot_pos = t.tensor([[fl[:2], fr[:2]]], dtype=t.float32)
                self._spawned = True
            a = s.get("anchor")
            if a is not None:
                aw = [p[i] + sum(R[i][j] * a[j] for j in range(3)) for i in range(3)]
                sy = float(env.spawn_yaw[0])
                env.spawn_root_xy = t.tensor([[aw[0] - self.anchor_fwd * math.cos(sy),
                                                aw[1] - self.anchor_fwd * math.sin(sy)]], dtype=t.float32)
        # lean command (rt/lean_cmd via the bridge); None -> 0 and flagged '~'
        cm = env.command_manager
        if s.get("lean") is not None:
            cm.lean[0, 0] = float(s["lean"])
            cm.lean_known = True
        # action from the controller's joint targets: a = (q_target - offset) / scale
        qc = s.get("qcmd")
        if qc and self.act_ids:
            qi = [qc[i] for i in self.jmap]
            a = [(qi[j] - self.act_offset[k]) / (self.act_scale[k] or 1.0) for k, j in enumerate(self.act_ids)]
            am = env.action_manager
            am.prev_action = am.action
            am.action = t.tensor([a], dtype=t.float32)
        env.episode_length_buf += 1
        env.common_step_counter += 1
        self.n_ticks += 1

    # ---- evaluate every term with ITS function ----
    def evaluate(self) -> list[tuple[str, float | None, str]]:
        """[(name, weighted_value | None, note)] in fixed |weight| order."""
        out = []
        t = self.torch
        for name, w, func, _ in self.terms:
            if name in self._na_reason:
                out.append((name, None, self._na_reason[name]))
                continue
            fn, params = self._fns[name], self._params[name]
            # inputs the rig does not export -> n/a, never silently zero
            reason = None
            for v in params.values():
                if hasattr(v, "name") and getattr(v, "name", None) == "contact_forces" and self.sensor is not None:
                    ids = v.body_ids
                    idx = list(range(self.sensor.num_bodies)) if isinstance(ids, slice) else list(ids)
                    if not bool(self.sensor.known[idx].all()):
                        reason = "needs sim_base_pose v4 (per-body contact forces)"
            if not self.have_pose:
                reason = reason or "no rt/sim_base_pose"
            if reason:
                out.append((name, None, reason))
                continue
            try:
                with t.no_grad():
                    val = fn(self.env, **params)
                val = float(val.reshape(-1)[0])
            except Exception as e:  # noqa: BLE001
                msg = f"{type(e).__name__}: {str(e)[:60]}"
                if name not in self._printed:
                    self._printed.add(name)
                    print(f"[ORACLE] n/a {name}: {msg}", flush=True)
                    if self.verbose:
                        traceback.print_exc(limit=2)
                out.append((name, None, msg))
                continue
            if not math.isfinite(val):
                out.append((name, None, "non-finite (input not exported: body pos/vel)"))
                continue
            note = ""
            if any(isinstance(v, str) and v == "lean_command" for v in params.values()) and not self.env.command_manager.lean_known:
                note = "~lean=0 assumed"
            out.append((name, w * val, note))
        return out

    def hud_field(self, items, top: int | None = None) -> str:
        rows = []
        total = sum(v for _, v, _ in items if v is not None)
        rows.append(f"TOTAL:{total:+.2f}:{max(-1.0, min(1.0, total / 50.0)):+.3f}")
        wmap = {n: w for n, w, _, _ in self.terms}
        for name, v, note in items[: top or len(items)]:
            if v is None:
                rows.append(f"{name[:15]} n/a:+0.00:+0.000")
                continue
            disp = name[:18] + ("~" if note.startswith("~") else "")
            frac = max(-1.0, min(1.0, v / (abs(wmap[name]) or 1.0)))
            rows.append(f"{disp}:{v:+.2f}:{frac:+.3f}")
        return "|".join(rows)

    def record(self, items):
        self.tape.append([time.monotonic()] + [(v if v is not None else None) for _, v, _ in items])

    def save_tape(self, path: str):
        if not self.tape:
            return None
        out = {"names": self.order, "weights": {n: w for n, w, _, _ in self.terms}, "rows": self.tape,
               "n_a": self._na_reason, "note": "cols: [t] + names; weighted /s; null = n/a; "
               "evaluated by the Isaac reward functions on MuJoCo state (reward_oracle.py)"}
        with open(path, "w") as f:
            json.dump(out, f)
        return path


# ─────────────────────────── UDP server ───────────────────────────
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--env-yaml", required=True, help="active policy's params/env.yaml")
    ap.add_argument("--deploy-yaml", default=None, help="default: sibling deploy.yaml")
    ap.add_argument("--port", type=int, default=47311)
    ap.add_argument("--rl-lab", default=os.environ.get("UNITREE_RL_LAB_DIR",
                    os.path.expanduser("~/Projects/robot_projects/repos/unitree_rl_lab-ik")))
    ap.add_argument("--isaaclab", default=os.environ.get("ISAACLAB_SRC",
                    os.path.expanduser("~/Projects/robot_projects/repos/IsaacLab/source")))
    ap.add_argument("--tape-dir", default=os.environ.get("LEDGER_TAPE_DIR", "."))
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    t0 = time.time()
    mdp = import_isaac(args.rl_lab, args.isaaclab)
    import torch
    torch.set_num_threads(1)
    print(f"[ORACLE] Isaac reward functions imported in {time.time() - t0:.1f}s (no SimulationApp; kit modules stubbed)", flush=True)
    oracle = RewardOracle(args.env_yaml, args.deploy_yaml, mdp, torch, verbose=not args.quiet)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.port))
    sock.settimeout(1.0)
    print(f"[ORACLE] listening on udp://127.0.0.1:{args.port}; waiting for the bridge's meta message", flush=True)
    stop = {"flag": False}

    def _sig(*_):
        stop["flag"] = True

    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)
    last_stat, n_state, t_eval = time.monotonic(), 0, 0.0
    while not stop["flag"]:
        try:
            data, addr = sock.recvfrom(65535)
        except socket.timeout:
            continue
        try:
            msg = json.loads(data.decode())
        except Exception:
            continue
        if msg.get("type") == "meta":
            if oracle.meta is None or msg.get("body_names") != oracle.meta.get("body_names"):
                try:
                    oracle.set_meta(msg, scene_bodies=getattr(oracle, "_scene_bodies", None))
                except Exception as e:  # noqa: BLE001
                    print(f"[ORACLE] meta rejected: {e}", flush=True)
            continue
        if oracle.meta is None:
            continue
        te = time.monotonic()
        try:
            oracle.update(msg)
            items = oracle.evaluate()
        except Exception as e:  # noqa: BLE001
            print(f"[ORACLE] tick failed: {type(e).__name__}: {e}", flush=True)
            traceback.print_exc(limit=3)
            continue
        t_eval += time.monotonic() - te
        n_state += 1
        oracle.record(items)
        try:
            sock.sendto(oracle.hud_field(items).encode(), addr)
        except OSError:
            pass
        if time.monotonic() - last_stat > 10.0:
            na = [n for n, v, _ in items if v is None]
            live = sum(1 for _, v, _ in items if v is not None)
            print(f"[ORACLE] {n_state} ticks, {1e3 * t_eval / max(1, n_state):.2f} ms/tick, "
                  f"{live} live / {len(na)} n/a{' (' + ', '.join(na[:6]) + (', …' if len(na) > 6 else '') + ')' if na else ''}"
                  f"{'' if oracle._v4 else ' — sim_base_pose v3 (rebuild simulate for per-body contacts/velocities)'}", flush=True)
            last_stat, n_state, t_eval = time.monotonic(), 0, 0.0
    path = oracle.save_tape(os.path.join(args.tape_dir, f"reward_tape_oracle_{time.strftime('%Y%m%d_%H%M%S')}.json"))
    if path:
        print(f"[ORACLE] tape saved: {path}", flush=True)


if __name__ == "__main__":
    main()
