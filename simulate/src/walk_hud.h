#pragma once
// walk_hud.h — WALK teleop HUD (2026-08-12): bottom-right panel with three
// center-zero bar gauges (vx / vy / wz). FILL = commanded velocity from
// rt/wirelesscontroller (the walk teleop tab, or a real remote); WHITE TICK =
// the ACTUAL base velocity measured in the sim (yaw-frame). Commanded-vs-actual
// on one bar answers "do the controls even work" at a glance — and shows when
// yawing is policy bias (wz cmd at 0, tick off-center).
// Auto-hides when no wirelesscontroller traffic for >2 s, so balance workflows
// never see it. DEBUG/sim2sim overlay only — no effect on physics.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mujoco/mujoco.h>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>

namespace walk_hud {

// lm2 walk deploy contract (display clamps; the controller clamps identically)
constexpr float VX_LO = -0.3f, VX_HI = 1.0f;
constexpr float VY_LO = -0.3f, VY_HI = 0.3f;
constexpr float WZ_LO = -0.5f, WZ_HI = 0.5f;

inline std::atomic<float>& cmd_vx() { static std::atomic<float> v{0}; return v; }
inline std::atomic<float>& cmd_vy() { static std::atomic<float> v{0}; return v; }
inline std::atomic<float>& cmd_wz() { static std::atomic<float> v{0}; return v; }
inline std::atomic<float>& act_vx() { static std::atomic<float> v{0}; return v; }
inline std::atomic<float>& act_vy() { static std::atomic<float> v{0}; return v; }
inline std::atomic<float>& act_wz() { static std::atomic<float> v{0}; return v; }
inline std::atomic<long>& last_ms() { static std::atomic<long> v{0}; return v; }

inline long now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Subscribe rt/wirelesscontroller (velocity_commands mapping: vx=ly, vy=-lx, wz=-rx).
inline void ensure_sub() {
  // InitChannel() is REQUIRED or the subscriber never receives (the
  // FsmCmdSubscriber pattern). Guarded + retried: render() can run before the
  // bridge has initialized the DDS factory.
  static std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>> sub;
  static long next_try = 0;
  if (sub || now_ms() < next_try) return;
  try {
    auto s = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>>(
        "rt/wirelesscontroller",
        [](const void* msg) {
          const auto& m = *reinterpret_cast<const unitree_go::msg::dds_::WirelessController_*>(msg);
          cmd_vx() = std::fmax(VX_LO, std::fmin(VX_HI, m.ly()));
          cmd_vy() = std::fmax(VY_LO, std::fmin(VY_HI, -m.lx()));
          cmd_wz() = std::fmax(WZ_LO, std::fmin(WZ_HI, -m.rx()));
          last_ms() = now_ms();
        });
    s->InitChannel();
    sub = s;
  } catch (...) {
    next_try = now_ms() + 1000;   // factory not up yet — retry in 1 s
  }
}

// Called from the bridge/physics side: actual base velocity in the YAW frame.
// Free-joint layout assumed at qpos 0 (h1_2 scenes): qpos[3..6] wxyz quat,
// qvel[0..2] world linear, qvel[3..5] angular (local z ~ yaw rate upright).
// Display smoothing for the ACTUAL markers: raw qvel at physics rate is spiky
// (every foot impact). ~100 ms EMA (alpha 0.02 at ~500 Hz) — display only.
constexpr float ACT_EMA = 0.02f;

inline void update_actual(const mjModel* m, const mjData* d) {
  // subscriber is created HERE (physics/bridge side, DDS factory guaranteed
  // live) — creating it on the render thread crashed the sim at startup
  // (SDK aborts pre-factory-init; not catchable as an exception).
  ensure_sub();
  if (m->nq < 7 || m->nv < 6) return;
  const mjtNum* q = d->qpos + 3;
  const double yaw = std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                                1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
  const double c = std::cos(yaw), s = std::sin(yaw);
  const float nvx = static_cast<float>( c * d->qvel[0] + s * d->qvel[1]);
  const float nvy = static_cast<float>(-s * d->qvel[0] + c * d->qvel[1]);
  const float nwz = static_cast<float>(d->qvel[5]);
  act_vx() = act_vx() * (1.0f - ACT_EMA) + nvx * ACT_EMA;
  act_vy() = act_vy() * (1.0f - ACT_EMA) + nvy * ACT_EMA;
  act_wz() = act_wz() * (1.0f - ACT_EMA) + nwz * ACT_EMA;
}

inline void draw_bar(const mjrContext* con, int x, int y, int w, int h,
                     float cmd, float act, float lo, float hi,
                     float r, float g, float b) {
  auto frac = [&](float v) { return (std::fmax(lo, std::fmin(hi, v)) - lo) / (hi - lo); };
  mjrRect track{x, y, w, h};
  mjr_rectangle(track, 0.15f, 0.15f, 0.15f, 0.75f);                 // track
  const int zero_px = x + static_cast<int>(frac(0.0f) * w);
  const int cmd_px = x + static_cast<int>(frac(cmd) * w);
  mjrRect fill{std::min(zero_px, cmd_px), y,
               std::max(2, std::abs(cmd_px - zero_px)), h};
  mjr_rectangle(fill, r, g, b, 0.9f);                               // cmd fill
  mjrRect zero{zero_px - 1, y - 2, 2, h + 4};
  mjr_rectangle(zero, 0.55f, 0.55f, 0.55f, 0.9f);                   // center tick
  const int act_px = x + static_cast<int>(frac(act) * w);
  mjrRect tick{act_px - 2, y - 2, 4, h + 4};
  mjr_rectangle(tick, 1.0f, 1.0f, 1.0f, 1.0f);                      // ACTUAL marker
}

// Render bottom-right; call from simulate.cc Render() (viewport = rect).
inline void render(const mjrRect& rect, const mjrContext* con) {
  if (now_ms() - last_ms() > 2000) return;   // no teleop -> hidden

  const int w = 240, h = 12, gap = 26;
  const int x = rect.left + rect.width - w - 20;
  int y = rect.bottom + 104;

  // DISPLAY convention is SCREEN-intuitive, not robot-frame: bar fill moving
  // RIGHT = strafe right / clockwise yaw (robot-frame +y is LEFT and +wz is
  // CCW, so vy/wz are sign-flipped FOR THE BARS ONLY; the numeric text below
  // keeps the true signed values). vx: right = forward.
  draw_bar(con, x, y, w, h, -cmd_wz(), -act_wz(), WZ_LO, WZ_HI, 1.0f, 0.6f, 0.1f); y += gap;
  draw_bar(con, x, y, w, h, -cmd_vy(), -act_vy(), VY_LO, VY_HI, 0.2f, 0.8f, 0.9f); y += gap;
  draw_bar(con, x, y, w, h,  cmd_vx(),  act_vx(), VX_LO, VX_HI, 0.3f, 0.9f, 0.3f);

  char txt[256];
  std::snprintf(txt, sizeof(txt),
                "WALK CMD | actual\nvx %+6.2f | %+6.2f m/s\nvy %+6.2f | %+6.2f m/s\nwz %+6.2f | %+6.2f rad/s",
                cmd_vx().load(), act_vx().load(), cmd_vy().load(), act_vy().load(),
                cmd_wz().load(), act_wz().load());
  mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMRIGHT, rect, txt, nullptr,
              const_cast<mjrContext*>(con));
}

}  // namespace walk_hud
