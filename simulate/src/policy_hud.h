#pragma once

#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

// Sim2sim on-screen HUD shared state. Three independently-written slots, each
// fed by a different source and drawn by the MuJoCo render loop (simulate.cc):
//   status  - active control policy + arm gains, from the controller (rt/policy_status)
//   payload - hand payload mass, set locally by the sim (keyboard)
//   metrics - balance metrics (lean, touchdowns, ...) from the sidecar (rt/balance_metrics)
// Thread-safe. DEBUG-ONLY — no effect on physics or control.
namespace policy_hud {

inline std::mutex& mutex() {
  static std::mutex m;
  return m;
}
inline std::string& status_slot() {
  static std::string s;
  return s;
}
inline std::string& payload_slot() {
  static std::string s;
  return s;
}
inline std::string& metrics_slot() {
  static std::string s;
  return s;
}
inline std::string& fsm_slot() {
  static std::string s;
  return s;
}

inline void set_status(const std::string& v) {
  std::lock_guard<std::mutex> lk(mutex());
  status_slot() = v;
}
inline void set_payload(const std::string& v) {
  std::lock_guard<std::mutex> lk(mutex());
  payload_slot() = v;
}
inline void set_metrics(const std::string& v) {
  std::lock_guard<std::mutex> lk(mutex());
  metrics_slot() = v;
}
inline void set_fsm(const std::string& v) {
  std::lock_guard<std::mutex> lk(mutex());
  fsm_slot() = v;
}

// Top-left block: controller policy/gains.
inline std::string get_status() {
  std::lock_guard<std::mutex> lk(mutex());
  return status_slot();
}
// Bottom-left block: sim payload + sidecar metrics, stacked.
inline std::string get_aux() {
  std::lock_guard<std::mutex> lk(mutex());
  std::string s = payload_slot();
  if (!metrics_slot().empty()) {
    if (!s.empty()) s += "\n";
    s += metrics_slot();
  }
  return s;
}

// Top-right block: vertical FSM key list ("press digit -> state").
inline std::string get_fsm_list() {
  std::string raw;
  {
    std::lock_guard<std::mutex> lk(mutex());
    raw = fsm_slot();
  }
  if (raw.empty()) return "";
  std::string out = "FSM KEYS";
  size_t i = 0;
  while (i < raw.size()) {
    while (i < raw.size() && raw[i] == ' ') ++i;
    size_t j = raw.find(' ', i);
    if (j == std::string::npos) j = raw.size();
    std::string entry = raw.substr(i, j - i);          // "2=Balance_x"
    size_t eq = entry.find('=');
    if (eq != std::string::npos) {
      out += "\n" + entry.substr(0, eq) + "  " + entry.substr(eq + 1);
    }
    i = j;
  }
  return out;
}

// Minimal flat-JSON field extractor (no JSON dependency). Handles "key":"str",
// "key":[array], and "key":scalar. Returns "" if the key is absent.
inline std::string json_field(const std::string& js, const std::string& key) {
  const std::string k = "\"" + key + "\"";
  auto p = js.find(k);
  if (p == std::string::npos) return "";
  p = js.find(':', p + k.size());
  if (p == std::string::npos) return "";
  ++p;
  while (p < js.size() && js[p] == ' ') ++p;
  if (p >= js.size()) return "";
  if (js[p] == '"') {
    auto e = js.find('"', p + 1);
    if (e == std::string::npos) return "";
    return js.substr(p + 1, e - (p + 1));
  }
  if (js[p] == '[') {
    auto e = js.find(']', p);
    if (e == std::string::npos) return "";
    return js.substr(p, e - p + 1);
  }
  auto e = js.find_first_of(",}", p);
  return js.substr(p, (e == std::string::npos ? js.size() : e) - p);
}

inline void set_ledger(const std::string& field);  // defined below (ledger gauges)

// Controller rt/policy_status JSON -> status slot.
inline void set_from_json(const std::string& js) {
  const std::string policy = json_field(js, "policy");
  const std::string trans = json_field(js, "arm_transition_s");
  const std::string ovr = json_field(js, "arm_gain_override");
  const std::string kp = json_field(js, "arm_kp");
  const std::string kd = json_field(js, "arm_kd");

  const std::string fsm_keys = json_field(js, "fsm_keys");
  if (!fsm_keys.empty()) set_fsm(fsm_keys);

  const std::string oa = json_field(js, "obs_age_ms");
  const std::string aa = json_field(js, "act_age_ms");

  std::string out = "POLICY: " + (policy.empty() ? std::string("?") : policy);
  if (!oa.empty() || !aa.empty())
    out += "\nobs_age " + (oa.empty() ? "?" : oa) + " ms   act_age " +
           (aa.empty() ? "?" : aa) + " ms";
  if (!trans.empty()) out += "\narm_transition: " + trans + "s";
  if (!ovr.empty()) out += "   gain_override: " + ovr;
  if (!kp.empty()) out += "\narm_kp " + kp;
  if (!kd.empty()) out += "\narm_kd " + kd;
  set_status(out);
}

// Sidecar rt/balance_metrics JSON -> metrics slot. Fields are best-effort; any
// the sidecar omits are simply skipped.
inline void set_metrics_from_json(const std::string& js) {
  const std::string lean_fwd = json_field(js, "lean_fwd");
  const std::string lean_lat = json_field(js, "lean_lat");
  const std::string steps_l = json_field(js, "steps_l");
  const std::string steps_r = json_field(js, "steps_r");
  const std::string td_rate = json_field(js, "touchdown_rate");
  const std::string ang_rms = json_field(js, "torso_ang_vel_rms");

  std::string out = "METRICS";
  if (!lean_fwd.empty() || !lean_lat.empty())
    out += "\nlean fwd " + lean_fwd + "  lat " + lean_lat;
  if (!steps_l.empty() || !steps_r.empty())
    out += "\nsteps L " + steps_l + "  R " + steps_r;
  if (!td_rate.empty()) out += "  (" + td_rate + "/s)";
  if (!ang_rms.empty()) out += "\ntorso_ang_vel rms " + ang_rms;
  // Reward LEDGER (2026-08-21, gauges v2): structured '|'-separated rows
  // "name:value:frac" from the sidecar's --ledger mode. Parsed into
  // ledger_rows() and drawn as walk_hud-style bars by ledger_render();
  // NOT appended to this text block.
  const std::string ledger = json_field(js, "ledger");
  if (!ledger.empty()) set_ledger(ledger);
  set_metrics(out);
}

// Keyboard FSM control (digits pressed in the sim window -> rt/fsm_cmd).
// Publisher is injected by main.cc (same pattern as arm_gui::publish_fn).
inline void (*&fsm_publish_fn())(char) {
  static void (*fn)(char) = nullptr;
  return fn;
}
inline void request_fsm(char key) {
  if (fsm_publish_fn()) fsm_publish_fn()(key);
}

// ── Reward LEDGER gauges (2026-08-21) ───────────────────────────────────────
// Sidecar --ledger publishes '|'-separated "name:value:frac" rows in a FIXED
// order (sorted by |weight| at sidecar init — rows never switch places).
// frac in [-1,1] normalizes value to the term's own scale (|weight|). Drawn
// as center-zero bars + name/value text, top-left column, walk_hud style.
struct LedgerRow {
  std::string name;
  float val = 0.f;
  float frac = 0.f;
};
inline std::vector<LedgerRow>& ledger_rows() {
  static std::vector<LedgerRow> v;
  return v;
}
inline double& ledger_ms() {
  static double t = 0.0;
  return t;
}
inline double ledger_now_ms() {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

inline void set_ledger(const std::string& field) {
  std::vector<LedgerRow> rows;
  size_t p = 0;
  while (p < field.size()) {
    size_t e = field.find('|', p);
    if (e == std::string::npos) e = field.size();
    const std::string item = field.substr(p, e - p);
    p = e + 1;
    const size_t c1 = item.find(':');
    const size_t c2 = (c1 == std::string::npos) ? std::string::npos
                                                : item.find(':', c1 + 1);
    if (c1 == std::string::npos || c2 == std::string::npos) continue;
    LedgerRow r;
    r.name = item.substr(0, c1);
    try {
      r.val = std::stof(item.substr(c1 + 1, c2 - c1 - 1));
      r.frac = std::stof(item.substr(c2 + 1));
    } catch (...) {
      continue;
    }
    rows.push_back(std::move(r));
  }
  std::lock_guard<std::mutex> lk(mutex());
  ledger_rows().swap(rows);
  ledger_ms() = ledger_now_ms();
}

// Call from simulate.cc Render() (viewport = rect). Top-left column.
inline void ledger_render(const mjrRect& rect, const mjrContext* con) {
  std::vector<LedgerRow> rows;
  {
    std::lock_guard<std::mutex> lk(mutex());
    if (ledger_now_ms() - ledger_ms() > 2000.0) return;  // sidecar gone -> hide
    rows = ledger_rows();
  }
  if (rows.empty()) return;

  const int name_w = 150, bar_w = 130, bar_h = 9, gap = 17, val_w = 62;
  const int x0 = rect.left + 12;
  int y = rect.bottom + rect.height - 40;  // top-left, below the top edge

  for (const auto& r : rows) {
    // name (left) + value (right of bar) as text; bar in the middle
    const float ty = static_cast<float>(y) / rect.height;
    mjr_text(mjFONT_SHADOW, r.name.c_str(), con,
             static_cast<float>(x0) / rect.width, ty, 0.9f, 0.9f, 0.9f);
    const int bx = x0 + name_w;
    mjrRect track{bx, y - 1, bar_w, bar_h};
    mjr_rectangle(track, 0.15f, 0.15f, 0.15f, 0.75f);
    const int zero_px = bx + bar_w / 2;
    float frac = std::fmax(-1.f, std::fmin(1.f, r.frac));
    const int fill_px = static_cast<int>(std::fabs(frac) * (bar_w / 2));
    if (fill_px > 0) {
      mjrRect fill{frac >= 0 ? zero_px : zero_px - fill_px, y - 1,
                   std::max(2, fill_px), bar_h};
      if (frac >= 0)
        mjr_rectangle(fill, 0.35f, 0.78f, 0.39f, 0.9f);   // income: green
      else
        mjr_rectangle(fill, 0.88f, 0.35f, 0.31f, 0.9f);   // penalty: red
    }
    mjrRect zero{zero_px - 1, y - 3, 2, bar_h + 4};
    mjr_rectangle(zero, 0.55f, 0.55f, 0.55f, 0.9f);
    char vtxt[32];
    std::snprintf(vtxt, sizeof vtxt, "%+7.2f", r.val);
    mjr_text(mjFONT_SHADOW, vtxt, con,
             static_cast<float>(bx + bar_w + 8) / rect.width, ty,
             0.85f, 0.85f, 0.85f);
    (void)val_w;
    y -= gap;
    if (y < rect.bottom + 200) break;  // don't collide with the METRICS block
  }
}

}  // namespace policy_hud
