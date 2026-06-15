#!/usr/bin/env python3
"""
sim_action_consumer.py — sim-only action consumer for the Architecture B loop.

Stands in for BOTH (a) the transform that ActionModule/JointCommander is
supposed to apply but does not, and (b) BridgeModule/set_joints' rt/lowcmd
writer (which also can't run in sim — MotionSwitcher). See JOINTCOMMANDER_SPEC.md
for the production version the teammate owns.

Per control step (50 Hz, = balance_contract.STEP_DT):
  1. read /MovementModule/policy_action       (13 raw policy outputs)
  2. read /BridgeModule/joint_set arm slots    (IK-resolved arm pose, SDK 13..26)
     (or a static default if --static-arms / no joint_set yet)
  3. transform legs+torso  (imported from balance_contract — single source):
        a = clip(action, ACTION_CLIP_LOW, ACTION_CLIP_HIGH)         # knee clip
        target_sdk[ACTION_SDK_IDS[k]] = a[k]*ACTION_SCALE + ACTION_OFFSET[k]
  4. arms:  target_sdk[ARM_SDK_ORDER] = arm_cmd_14   (commanded, not measured)
  5. write rt/lowcmd (unitree_hg LowCmd_) with per-motor q + kp/kd:
        legs+torso kp/kd = deploy.yaml stiffness/damping (motor order)
        arms       kp/kd = 50 / 1.0      (harness/sim2real default; --gains)

Cross-checked: joint_ids_map[LEG_TORSO_URDF_IDS] in the C++ reference controller
equals ACTION_SDK_IDS here, so the action→motor routing matches the known-good
deploy path. We import the constants rather than re-hardcoding them.

Run (inside the ros2-humble-dev container, ROS2 sourced):
    python3 sim_action_consumer.py --iface lo
Requires PYTHONPATH:  <Aspired>/.global, <Aspired>/MovementModule/main
Requires installed:   unitree_sdk2py, numpy, rclpy, pyyaml
"""

import argparse
import os
import sys
import threading
import time

import numpy as np

ASPIRED_ROOT = os.environ.get("ASPIRED_ROOT", "/workspace")
for _p in (os.path.join(ASPIRED_ROOT, ".global"),
           os.path.join(ASPIRED_ROOT, "MovementModule", "main")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from Ros2_connector import Ros2  # noqa: E402
from balance_contract import (  # noqa: E402
    N_ACTION, N_BODY,
    ACTION_SCALE, ACTION_OFFSET, ACTION_CLIP_LOW, ACTION_CLIP_HIGH,
    ACTION_SDK_IDS, ARM_SDK_ORDER, OBS_SDK_ORDER, DEFAULT_JOINT_POS,
    STEP_DT, policy_dir,
)

NUM_MOTOR = 27
ARM_KP_DEFAULT = 50.0   # team real-robot arm gains (BridgeModule H1_2_KP/KD)
ARM_KD_DEFAULT = 1.0


def sdk_default_pose() -> np.ndarray:
    """DEFAULT_JOINT_POS (policy order) → SDK/motor order, for the pre-command
    hold. Inverse of the OBS_SDK_ORDER reindex (mirrors offline_check)."""
    q = np.zeros(NUM_MOTOR, dtype=np.float32)
    for k, sdk_idx in enumerate(OBS_SDK_ORDER):
        q[int(sdk_idx)] = DEFAULT_JOINT_POS[k]
    return q


def load_gains(gains_mode: str):
    """Return (kp[27], kd[27]) in SDK/motor order.

    deploy.yaml stiffness/damping are authored in motor order (first 27 entries;
    trailing zeros are padding). harness: legs+torso from deploy.yaml, arms 50/1.
    """
    import yaml
    # deploy.yaml carries Isaac python-tagged slices (asset_cfg.*_ids); ignore
    # them so SafeLoader doesn't choke (same approach as offline_check.py).
    class _Loader(yaml.SafeLoader):
        pass
    _Loader.add_multi_constructor(
        "tag:yaml.org,2002:python/object/apply:", lambda ldr, suf, node: None)
    dy = policy_dir() / "deploy.yaml"
    with open(dy, "r") as f:
        d = yaml.load(f, Loader=_Loader)
    kp = np.asarray(d["stiffness"][:NUM_MOTOR], dtype=np.float32)
    kd = np.asarray(d["damping"][:NUM_MOTOR], dtype=np.float32)

    if gains_mode == "deploy":
        pass  # all-motor deploy.yaml gains (stock C++ controller regime)
    elif gains_mode == "harness":
        kp[13:27] = ARM_KP_DEFAULT   # arm motors → real-robot arm gains
        kd[13:27] = ARM_KD_DEFAULT
    elif gains_mode == "flat50":
        kp[:] = ARM_KP_DEFAULT       # BridgeModule-equivalent (legs WRONG; for A/B only)
        kd[:] = ARM_KD_DEFAULT
    else:
        raise SystemExit(f"unknown --gains {gains_mode!r}")
    return kp, kd


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default="lo")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--gains", default="harness", choices=["harness", "deploy", "flat50"],
                    help="kp/kd regime (default harness: legs deploy.yaml, arms 50/1)")
    ap.add_argument("--static-arms", action="store_true",
                    help="ignore /BridgeModule/joint_set; hold default arm pose "
                         "(Mode A bring-up, no ActionModule)")
    args = ap.parse_args()

    kp, kd = load_gains(args.gains)
    default_sdk = sdk_default_pose()
    arm_default = default_sdk[ARM_SDK_ORDER].copy()
    print(f"[sim_action_consumer] gains={args.gains}  kp(motor0..12)="
          f"{np.round(kp[:13],0)}  arm_kp={kp[13]}  static_arms={args.static_arms}",
          flush=True)

    # ── Unitree DDS: rt/lowstate (for mode_machine + current-q hold), rt/lowcmd
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber,
    )
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC

    ChannelFactoryInitialize(args.domain, args.iface)
    crc = CRC()
    low_cmd = unitree_hg_msg_dds__LowCmd_()

    st = {"msg": None, "mode_machine": None}
    lock = threading.Lock()

    def on_lowstate(msg: "LowState_"):
        with lock:
            st["msg"] = msg
            if st["mode_machine"] is None:
                st["mode_machine"] = msg.mode_machine

    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(on_lowstate, 10)

    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    # ── ROS2: policy_action + joint_set ──────────────────────────────────────
    import rclpy
    rclpy.init()
    ros = Ros2("ArchBSimConsumer")
    ACTION_TOPIC = "/MovementModule/policy_action"
    JOINT_SET_TOPIC = "/BridgeModule/joint_set"

    print("[sim_action_consumer] waiting for mode_machine from rt/lowstate...",
          flush=True)
    while rclpy.ok() and st["mode_machine"] is None:
        time.sleep(0.01)
    print(f"[sim_action_consumer] mode_machine={st['mode_machine']} — control ready",
          flush=True)

    last_action = None
    next_t = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(ros, timeout_sec=0.0)

            action = ros.get_data(ACTION_TOPIC, "f32arr")
            with lock:
                msg = st["msg"]

            if msg is None:
                time.sleep(STEP_DT)
                continue

            # current measured q (SDK order) — the safe pre-command hold
            cur_q = np.array([msg.motor_state[i].q for i in range(NUM_MOTOR)],
                             dtype=np.float32)

            target = cur_q.copy()  # default: hold current pose

            # arms: commanded IK pose from joint_set (SDK 13..26 via ARM_SDK_ORDER),
            # else static default
            if args.static_arms:
                arm_cmd = arm_default
            else:
                jset = ros.get_data(JOINT_SET_TOPIC, "f32arr")
                if jset is not None and len(jset) >= N_BODY:
                    arm_cmd = np.asarray(jset, dtype=np.float32)[ARM_SDK_ORDER]
                else:
                    arm_cmd = cur_q[ARM_SDK_ORDER]  # hold measured arms until joint_set flows
            target[ARM_SDK_ORDER] = arm_cmd

            # legs+torso: transform the 13 raw actions (only once we have them)
            if action is not None and len(action) >= N_ACTION:
                a = np.asarray(action[:N_ACTION], dtype=np.float32)
                a = np.clip(a, ACTION_CLIP_LOW, ACTION_CLIP_HIGH)  # knee clip etc.
                legs_torso = a * ACTION_SCALE + ACTION_OFFSET
                target[ACTION_SDK_IDS] = legs_torso
                last_action = a
            # else: legs/torso stay at current measured pose (hold)

            # ── write rt/lowcmd ──────────────────────────────────────────────
            low_cmd.mode_pr = 0  # PR series control
            low_cmd.mode_machine = st["mode_machine"]
            for i in range(NUM_MOTOR):
                m = low_cmd.motor_cmd[i]
                m.mode = 1
                m.q = float(target[i])
                m.dq = 0.0
                m.tau = 0.0
                m.kp = float(kp[i])
                m.kd = float(kd[i])
            low_cmd.crc = crc.Crc(low_cmd)
            pub.Write(low_cmd)

            next_t += STEP_DT
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        ros.destroy_node()
        rclpy.shutdown()
        print("[sim_action_consumer] stopped", flush=True)


if __name__ == "__main__":
    main()
