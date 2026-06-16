#!/usr/bin/env python3
"""
selftest_transform.py — offline self-test for sim_action_consumer's action
transform + remap. No DDS, no ROS2, no MuJoCo. Asserts the consumer's math
against the SOURCE OF TRUTH (deploy.yaml + the C++ reference controller's map),
rather than trusting the vendored constants.

Run in the container with ROS sourced (for numpy + pyyaml):
    source /opt/ros/humble/setup.bash
    PYTHONPATH=/workspace/MovementModule/main python3 selftest_transform.py
"""
import os
import sys

import numpy as np

ASPIRED_ROOT = os.environ.get("ASPIRED_ROOT", "/workspace")
sys.path.insert(0, os.path.join(ASPIRED_ROOT, "MovementModule", "main"))

from balance_contract import (  # noqa: E402
    ACTION_SCALE, ACTION_OFFSET, ACTION_CLIP_LOW, ACTION_CLIP_HIGH,
    ACTION_SDK_IDS, ARM_SDK_ORDER, OBS_SDK_ORDER, DEFAULT_JOINT_POS, policy_dir,
)

# The C++ reference controller (unitree_rl_lab .../State_RLBase.cpp:18-35)
# documents these URDF/articulation indices for the 13 leg+torso actions.
LEG_TORSO_URDF_IDS = [0, 1, 2, 3, 4, 7, 8, 11, 12, 15, 16, 19, 20]

ok = True
def check(name, cond):
    global ok
    ok &= bool(cond)
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}")


def main():
    print("=" * 60)
    print("ACTION TRANSFORM / REMAP SELF-TEST")
    print("=" * 60)

    # 1) ACTION_SDK_IDS must equal joint_ids_map[LEG_TORSO_URDF_IDS] from the
    #    ACTIVE policy's deploy.yaml — the exact route the C++ controller uses.
    import yaml
    # deploy.yaml carries Isaac python-tagged slices (asset_cfg.*_ids); ignore
    # them the same way offline_check does, so SafeLoader doesn't choke.
    class _Loader(yaml.SafeLoader):
        pass
    _Loader.add_multi_constructor(
        "tag:yaml.org,2002:python/object/apply:", lambda ldr, suf, node: None)
    with open(policy_dir() / "deploy.yaml") as f:
        d = yaml.load(f, Loader=_Loader)
    jmap = d["joint_ids_map"]
    cpp_map = [jmap[u] for u in LEG_TORSO_URDF_IDS]
    print(f"\njoint_ids_map[LEG_TORSO_URDF_IDS] = {cpp_map}")
    print(f"balance_contract.ACTION_SDK_IDS  = {list(ACTION_SDK_IDS)}")
    check("ACTION_SDK_IDS == C++ reference map", list(ACTION_SDK_IDS) == cpp_map)

    # 2) The 13 leg/torso motors and 14 arm motors together cover 0..26, disjoint.
    legs = set(int(x) for x in ACTION_SDK_IDS)
    arms = set(int(x) for x in ARM_SDK_ORDER)
    check("13 leg/torso slots unique", len(legs) == 13)
    check("14 arm slots unique", len(arms) == 14)
    check("legs ∩ arms == ∅", legs.isdisjoint(arms))
    check("legs ∪ arms == {0..26}", legs | arms == set(range(27)))

    # 3) Transform equals deploy.yaml scale/offset/clip and produces finite,
    #    in-limit knee targets even for extreme actions.
    sa = d["actions"]["JointPositionAction"]
    check("scale == deploy.yaml", np.allclose([ACTION_SCALE] * 13, sa["scale"]))
    check("offset == deploy.yaml", np.allclose(ACTION_OFFSET, sa["offset"]))

    def transform(action):
        a = np.clip(np.asarray(action, np.float32), ACTION_CLIP_LOW, ACTION_CLIP_HIGH)
        return a * ACTION_SCALE + ACTION_OFFSET

    # extreme knee push: action slots 7,8 are the knees, clip [0.45, 2.5]
    big = np.full(13, 100.0, np.float32)
    t = transform(big)
    knee_hi = 2.5 * ACTION_SCALE + ACTION_OFFSET[7]   # = 1.225
    check("knee clips high (action+100 → 2.5)", np.isclose(t[7], knee_hi) and np.isclose(t[8], 2.5 * ACTION_SCALE + ACTION_OFFSET[8]))
    small = np.full(13, -100.0, np.float32)
    t2 = transform(small)
    knee_lo = 0.45 * ACTION_SCALE + ACTION_OFFSET[7]  # = 0.7125
    check("knee clips low (action-100 → 0.45)", np.isclose(t2[7], knee_lo))
    check("all transformed targets finite", np.all(np.isfinite(t)) and np.all(np.isfinite(t2)))

    # 4) At zero action, non-knee slots land on ACTION_OFFSET; the two knee
    #    slots (7,8) clip UP to the 0.45 floor first (crouch floor), so they
    #    equal 0.45*scale+offset, NOT offset. Confirms clip-before-scale/offset.
    zero_t = transform(np.zeros(13, np.float32))
    nonknee = [i for i in range(13) if i not in (7, 8)]
    check("zero-action non-knee == ACTION_OFFSET",
          np.allclose(zero_t[nonknee], ACTION_OFFSET[nonknee]))
    check("zero-action knees clip to 0.45 floor",
          np.isclose(zero_t[7], 0.45 * ACTION_SCALE + ACTION_OFFSET[7]) and
          np.isclose(zero_t[8], 0.45 * ACTION_SCALE + ACTION_OFFSET[8]))

    print("\n" + "=" * 60)
    print(f"RESULT: {'ALL PASSED' if ok else 'FAILURES'}")
    print("=" * 60)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
