#pragma once

#include <mutex>
#include <string>

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

}  // namespace policy_hud
