#pragma once
// anchor_pub.h — the training anchor, live in sim2sim.
//
// On policy engage (the ARCHB_BAND_RELEASE_FILE flag the band thread already
// watches), capture the base pose and plant a world-fixed anchor FWD_OFFSET m
// ahead at HEIGHT_W m — the same contract as training's anchor_point_b obs
// (spawn + 0.5 m forward, 1.0 m world height). Every physics iteration:
//   * drive the YELLOW mocap ball (scene body "anchor_ball") to the anchor,
//   * hand back the anchor in the BASE frame (full-quat inverse rotate) for
//     the DDS publisher (rt/anchor_point -> BridgeModule getter tail ->
//     MovementModule 90-obs policies).
// Inert until engage; ball-less scenes just skip the marker. Env overrides:
// ANCHOR_FWD_OFFSET (m, default 0.5), ANCHOR_HEIGHT_W (m, default 1.0).
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <mujoco/mujoco.h>

namespace anchor_pub {

inline std::atomic<bool>& engage_request() {
  static std::atomic<bool> f{false};
  return f;
}
inline bool& active() {
  static bool a = false;
  return a;
}
inline mjtNum* anchor_w() {
  static mjtNum p[3] = {0, 0, 0};
  return p;
}
inline double fwd_offset() {
  static double v = [] {
    const char* e = std::getenv("ANCHOR_FWD_OFFSET");
    return e && e[0] ? std::atof(e) : 0.5;
  }();
  return v;
}
inline double height_w() {
  static double v = [] {
    const char* e = std::getenv("ANCHOR_HEIGHT_W");
    return e && e[0] ? std::atof(e) : 1.0;
  }();
  return v;
}

// Call once per physics iteration under the sim mutex. Returns true and fills
// rel_b (anchor in the BASE frame) while the anchor is active.
inline bool step(const mjModel* m, mjData* d, double rel_b[3]) {
  static int mocap_id = -2;
  if (mocap_id == -2) {
    int b = mj_name2id(m, mjOBJ_BODY, "anchor_ball");
    mocap_id = (b >= 0) ? m->body_mocapid[b] : -1;
  }
  const mjtNum* q = d->qpos;  // freejoint base: pos q[0..2], quat q[3..6] (w,x,y,z)
  if (engage_request().exchange(false)) {
    double w = q[3], x = q[4], y = q[5], z = q[6];
    double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    anchor_w()[0] = q[0] + fwd_offset() * std::cos(yaw);
    anchor_w()[1] = q[1] + fwd_offset() * std::sin(yaw);
    anchor_w()[2] = height_w();
    active() = true;
    std::printf("[anchor] planted at (%.2f, %.2f, %.2f) — %.2fm ahead of the engage pose\n",
                anchor_w()[0], anchor_w()[1], anchor_w()[2], fwd_offset());
    std::fflush(stdout);
  }
  if (!active()) return false;
  if (mocap_id >= 0) {
    d->mocap_pos[3 * mocap_id + 0] = anchor_w()[0];
    d->mocap_pos[3 * mocap_id + 1] = anchor_w()[1];
    d->mocap_pos[3 * mocap_id + 2] = anchor_w()[2];
  }
  mjtNum v[3] = {anchor_w()[0] - q[0], anchor_w()[1] - q[1], anchor_w()[2] - q[2]};
  mjtNum qinv[4], out[3];
  mju_negQuat(qinv, q + 3);
  mju_rotVecQuat(out, v, qinv);
  rel_b[0] = out[0]; rel_b[1] = out[1]; rel_b[2] = out[2];
  return true;
}

}  // namespace anchor_pub
