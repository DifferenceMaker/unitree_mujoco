#pragma once
// grasp_sim.h — GRASP RIG (2026-08-28): Inspire RIGHT-hand emulation + object/torso
// pose feed, for testing the gr-line grasp policies THROUGH the arch-B stack.
//
// The real hand is a separate Modbus-TCP device (BridgeModule/finger_manager ->
// inspire_sdkpy), not a lowstate/lowcmd motor. So in the sim the 6 finger DRIVER
// actuators (rh:drv_<finger>, position servos built by
// aspired-isaac-lab/scripts/tools/build_grasp_scene.py, followers coupled by
// <equality>) are driven HERE, from closure commands that arrive on
// rt/sim_hand/cmd (JSON {"closure":[6], "speed":s}) — published by the Inspire
// Modbus emulator (Aspired BridgeModule/Utils/inspire_sim_emu.py) that the REAL
// finger_manager talks to over 127.0.0.1:6000. Closure = 0 open .. 1 closed,
// FINGER_ORDER little/ring/middle/index/thumb_bend/thumb_rot (= Inspire angle_set
// slot order). The hand state goes back the same way: rt/sim_hand/state JSON
// with measured closure(6), the 17 tactile pad contact forces BY PAD LINK NAME
// (the emulator maps names -> Modbus taxel registers; never by index — the
// right-hand pad order trap, finger_mapping.POLICY_PAD_LINK_ORDER), plus the
// world poses of the object, torso_link and the palm pad (ground truth for the
// vision stand-in object_pose_relay.py, which adds the 1-2 s hold + noise).
// The same JSON is mirrored to ARCHB_GRASP_FILE (atomic rename) because the
// ActionModule venv has no unitree_sdk2py (same mechanism as .lean_cmd).
//
// Enabled by ARCHB_GRASP=1 (main.cc). Silent no-op when the scene has no
// rh:drv_* actuators.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <sys/stat.h>

#include <mujoco/mujoco.h>

namespace grasp_sim {

inline bool& enabled() { static bool e = false; return e; }
inline std::function<void(const std::string&)>& publish_fn() {
  static std::function<void(const std::string&)> f; return f;
}

struct State {
  bool init = false, ok = false;
  int drv_act[6] = {-1, -1, -1, -1, -1, -1};
  int drv_jnt[6] = {-1, -1, -1, -1, -1, -1};
  double lo[6] = {0}, hi[6] = {0};
  // The vendor URDF's 17 force_sensor pad LINKS hang on fixed joints, which the
  // MuJoCo import fuses into the phalanx bodies — so a pad is a mesh GEOM (mesh
  // name rh:right_<x>_force_sensor[_n]), not a body. Contacts are matched per geom.
  std::vector<int> pad_geom;          // 17 pad geoms
  std::vector<std::string> pad_name;  // namespace stripped: right_palm_force_sensor, ...
  std::vector<int> geom2pad;          // ngeom -> pad idx or -1
  std::vector<char> geom_is_hand;     // geom belongs to an rh:* body (self-touch mask)
  int obj_body = -1, obj_jnt = -1, torso_body = -1, palm_geom = -1, table_body = -1;
  int belief_mocap = -1, belief_geom = -1;   // vision-belief ghost (mocap marker)
  double belief_mtime = 0.0, belief_flash_t = -1.0;
  double retract_mtime = 0.0;
  bool retracted = false;
  double place_mtime = 0.0;
  double obj_z0 = 0.0;
  std::mutex mtx;
  double target[6] = {0, 0, 0, 0, 0, 0};    // commanded closure
  double applied[6] = {0, 0, 0, 0, 0, 0};   // slewed closure actually sent to the servo
  double slew = 2.5;                        // closure/s: Inspire full travel ~0.4 s at speed 1000
  int pub_cnt = 0, print_cnt = 0;
  int cmd_count = 0;
};
inline State& st() { static State s; return s; }

static const char* kDrivers[6] = {"little", "ring", "middle", "index", "thumb_bend", "thumb_rot"};

inline void init(const mjModel* m) {
  State& s = st();
  s.init = true;
  for (int i = 0; i < 6; ++i) {
    const std::string an = std::string("rh:drv_") + kDrivers[i];
    s.drv_act[i] = mj_name2id(m, mjOBJ_ACTUATOR, an.c_str());
    if (s.drv_act[i] < 0) { std::printf("[GRASP] no actuator %s — hand emulation OFF\n", an.c_str()); return; }
    s.drv_jnt[i] = m->actuator_trnid[2 * s.drv_act[i]];
    s.lo[i] = m->jnt_range[2 * s.drv_jnt[i]];
    s.hi[i] = m->jnt_range[2 * s.drv_jnt[i] + 1];
  }
  s.geom2pad.assign(m->ngeom, -1);
  s.geom_is_hand.assign(m->ngeom, 0);
  for (int g = 0; g < m->ngeom; ++g) {
    const char* bn = mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[g]);
    if (bn && std::strncmp(bn, "rh:", 3) == 0) s.geom_is_hand[g] = 1;
  }
  for (int g = 0; g < m->ngeom; ++g) {
    if (m->geom_type[g] != mjGEOM_MESH) continue;
    const char* mn = mj_id2name(m, mjOBJ_MESH, m->geom_dataid[g]);
    if (!mn) continue;
    std::string n(mn);
    if (n.find("force_sensor") == std::string::npos) continue;
    const size_t c = n.find(':');
    if (c != std::string::npos) n = n.substr(c + 1);
    if (n.find("palm_force_sensor") != std::string::npos) s.palm_geom = g;
    s.geom2pad[g] = (int)s.pad_geom.size();
    s.pad_geom.push_back(g);
    s.pad_name.push_back(n);
  }
  s.obj_body = mj_name2id(m, mjOBJ_BODY, "obj:object");   // attached with the "obj:" prefix by build_grasp_scene.py
  if (s.obj_body < 0) s.obj_body = mj_name2id(m, mjOBJ_BODY, "object");
  s.obj_jnt = mj_name2id(m, mjOBJ_JOINT, "obj:object_free");
  if (s.obj_jnt < 0) s.obj_jnt = mj_name2id(m, mjOBJ_JOINT, "object_free");
  s.torso_body = mj_name2id(m, mjOBJ_BODY, "torso_link");
  {
    int bb = mj_name2id(m, mjOBJ_BODY, "belief");
    if (bb >= 0) s.belief_mocap = m->body_mocapid[bb];
    s.belief_geom = mj_name2id(m, mjOBJ_GEOM, "belief_marker");
  }
  s.table_body = mj_name2id(m, mjOBJ_BODY, "tbl:table");
  if (s.table_body < 0) s.table_body = mj_name2id(m, mjOBJ_BODY, "table");
  s.ok = s.obj_body >= 0 && s.torso_body >= 0 && s.palm_geom >= 0 && s.pad_geom.size() == 17;
  std::printf("[GRASP] hand emulation %s: 6 drivers, %zu pad geoms, object body %d, torso body %d, palm geom %d\n",
              s.ok ? "ON" : "INCOMPLETE (check scene)", s.pad_geom.size(), s.obj_body, s.torso_body, s.palm_geom);
  std::fflush(stdout);
}

// {"closure":[c0..c5]} (+ optional "speed": 0..1 fraction of the max slew)
inline void set_cmd_json(const std::string& js) {
  State& s = st();
  const size_t k = js.find("\"closure\"");
  if (k == std::string::npos) return;
  const size_t a = js.find('[', k);
  if (a == std::string::npos) return;
  double v[6];
  const char* p = js.c_str() + a + 1;
  for (int i = 0; i < 6; ++i) {
    char* end = nullptr;
    v[i] = std::strtod(p, &end);
    if (end == p) return;
    p = end;
    while (*p == ',' || *p == ' ') ++p;
  }
  double speed = -1.0;
  const size_t sk = js.find("\"speed\"");
  if (sk != std::string::npos) {
    const size_t colon = js.find(':', sk);
    if (colon != std::string::npos) speed = std::strtod(js.c_str() + colon + 1, nullptr);
  }
  std::lock_guard<std::mutex> lk(s.mtx);
  for (int i = 0; i < 6; ++i) s.target[i] = std::min(1.0, std::max(0.0, v[i]));
  if (speed > 0.0) s.slew = 2.5 * std::min(1.0, speed);
  ++s.cmd_count;
}

inline void write_file_atomic(const char* path, const std::string& body) {
  const std::string tmp = std::string(path) + ".tmp";
  { std::ofstream f(tmp); f << body << "\n"; }
  std::rename(tmp.c_str(), path);
}

// SPAWN HOLD: the welded-pelvis scene has no balance policy and no elastic band; until
// the Bridge's first lowcmd arrives (~10 s: emulator + Bridge start) the 27 body joints
// would hang unactuated (arm swings into the legs, torso yaw drifts) and the Bridge
// then latches THAT pose as its startup hold. So hold every body joint at its spawn
// value with a PD through qfrc_applied (no race with the bridge's ctrl writes) while
// all 27 body ctrls are exactly zero — i.e. no lowcmd has been applied yet.
inline void spawn_hold(const mjModel* m, mjData* d) {
  static bool released = false;
  static std::vector<double> q0;
  static const char* hs = std::getenv("ARCHB_SPAWN_HOLD");
  if (released || (hs && hs[0] == '0')) return;
  const int nbody_act = 27 <= m->nu ? 27 : m->nu;
  bool any_ctrl = false;
  for (int a = 0; a < nbody_act; ++a) if (d->ctrl[a] != 0.0) { any_ctrl = true; break; }
  if (any_ctrl) {
    released = true;
    for (int a = 0; a < nbody_act; ++a) { const int dof = m->jnt_dofadr[m->actuator_trnid[2 * a]]; d->qfrc_applied[dof] = 0.0; }
    std::printf("[GRASP] lowcmd flowing at t=%.2f s — spawn hold released to the Bridge\n", d->time);
    std::fflush(stdout);
    return;
  }
  if (q0.empty()) {
    q0.resize(nbody_act);
    for (int a = 0; a < nbody_act; ++a) q0[a] = d->qpos[m->jnt_qposadr[m->actuator_trnid[2 * a]]];
  }
  for (int a = 0; a < nbody_act; ++a) {
    const int j = m->actuator_trnid[2 * a];
    const int qa = m->jnt_qposadr[j], va = m->jnt_dofadr[j];
    const double lim = m->actuator_ctrlrange[2 * a + 1] > 0 ? m->actuator_ctrlrange[2 * a + 1] : 100.0;
    double f = 300.0 * (q0[a] - d->qpos[qa]) - 5.0 * d->qvel[va];
    d->qfrc_applied[va] = std::max(-lim, std::min(lim, f));
  }
}

// Per physics step (call BEFORE mj_step, under the sim mutex).
inline void step(const mjModel* m, mjData* d) {
  if (!enabled()) return;
  State& s = st();
  if (!s.init) init(m);
  spawn_hold(m, d);
  if (s.drv_act[0] < 0) return;
  const double dt = m->opt.timestep;
  double meas[6];
  {
    std::lock_guard<std::mutex> lk(s.mtx);
    for (int i = 0; i < 6; ++i) {
      const double dmax = s.slew * dt;
      const double e = s.target[i] - s.applied[i];
      s.applied[i] += std::max(-dmax, std::min(dmax, e));
      d->ctrl[s.drv_act[i]] = s.lo[i] + s.applied[i] * (s.hi[i] - s.lo[i]);
      const double q = d->qpos[m->jnt_qposadr[s.drv_jnt[i]]];
      meas[i] = std::min(1.0, std::max(0.0, (q - s.lo[i]) / std::max(1e-6, s.hi[i] - s.lo[i])));
    }
  }
  // BELIEF GHOST (2026-09-03): the relay writes each vision SAMPLE's believed
  // WORLD position to GRASP_BELIEF_FILE; render it as a small marker that
  // FLASHES bright on update and dims while the sample is held — the visible
  // gap between what the policy believes and where the object is.
  if (s.belief_mocap >= 0) {
    static const char* bf = std::getenv("ARCHB_GRASP_BELIEF_FILE");
    const char* bpath = bf && bf[0] ? bf : "logs/.grasp_belief";
    struct stat bst {};
    if (stat(bpath, &bst) == 0) {
      const double bmt = (double)bst.st_mtime + (double)bst.st_mtim.tv_nsec * 1e-9;
      if (bmt > s.belief_mtime) {
        s.belief_mtime = bmt;
        std::FILE* fp = std::fopen(bpath, "r");
        if (fp) {
          double bx, by, bz;
          if (std::fscanf(fp, "%lf %lf %lf", &bx, &by, &bz) == 3) {
            d->mocap_pos[3 * s.belief_mocap + 0] = bx;
            d->mocap_pos[3 * s.belief_mocap + 1] = by;
            d->mocap_pos[3 * s.belief_mocap + 2] = bz;
            s.belief_flash_t = d->time;
          }
          std::fclose(fp);
        }
      }
    }
    if (s.belief_geom >= 0 && s.belief_flash_t >= 0.0) {
      const double age = d->time - s.belief_flash_t;
      const float a = age < 0.35 ? 0.95f : (age < 1.5 ? 0.45f : 0.22f);
      m->geom_rgba[4 * s.belief_geom + 3] = a;   // alpha pulse: bright on update, dim while held
    }
  }

  // SUPPORT RETRACT (2026-09-03, diagnostic): the TRAINING world's table VANISHES
  // 0.3-1.0 s after the approach (grasp_mdp.retract_support teleports the platform
  // far below — the policy learned to CATCH a released cube, not to pick one off a
  // solid table; the rig's permanent table made its close-fast reflex punt the
  // cube). When rl_grasp writes ARCHB_GRASP_RETRACT_FILE (AM_GRASP_RETRACT=1),
  // kill the table's collisions and ghost it visually — trained physics on demand.
  if (!s.retracted) {
    static const char* rf = std::getenv("ARCHB_GRASP_RETRACT_FILE");
    if (rf && rf[0]) {
      struct stat rst {};
      if (stat(rf, &rst) == 0) {
        const double rmt = (double)rst.st_mtime + (double)rst.st_mtim.tv_nsec * 1e-9;
        if (s.retract_mtime == 0.0) s.retract_mtime = rmt;   // stale file at boot: arm only
        else if (rmt > s.retract_mtime) {
          for (int g = 0; g < m->ngeom; ++g) {
            const char* bn = mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[g]);
            if (bn && std::strncmp(bn, "tbl:", 4) == 0) {
              m->geom_contype[g] = 0;
              m->geom_conaffinity[g] = 0;
              m->geom_rgba[4 * g + 3] = 0.15f;
            }
          }
          s.retracted = true;
          std::printf("[GRASP] SUPPORT RETRACTED — table collisions OFF (training reset semantics)\n");
        }
      }
    }
  }

  // PLACE-OBJECT command (ARCHB_GRASP_PLACE_FILE, written by the rl_grasp sequence,
  // AM_GRASP_HOVER=place): teleport the object under the LIVE palm pad — Isaac's
  // palm_track reset semantics reproduced at HANDOVER time. This puts the object
  // exactly where the (sagged) palm actually is, i.e. the state the policy trained
  // from, instead of asking an absolute-target policy to cross the sag gap; it also
  // undoes any pre-handover nudge (2026-08-31 run: the IK approach swept the cube
  // 11 cm sideways). xy = palm pad, z kept (the object stays on the table).
  {
    static const char* pf = std::getenv("ARCHB_GRASP_PLACE_FILE");
    if (pf && *pf && s.obj_jnt >= 0 && s.palm_geom >= 0) {
      struct stat sb;
      if (stat(pf, &sb) == 0) {
        const double mt = (double)sb.st_mtime;
        if (s.place_mtime != 0.0 && mt > s.place_mtime) {
          const int qa = m->jnt_qposadr[s.obj_jnt], va = m->jnt_dofadr[s.obj_jnt];
          d->qpos[qa] = d->geom_xpos[3 * s.palm_geom];
          d->qpos[qa + 1] = d->geom_xpos[3 * s.palm_geom + 1];
          for (int k = 0; k < 6; ++k) d->qvel[va + k] = 0.0;
          std::printf("[GRASP] PLACE: object teleported under the live palm (%.3f, %.3f), z kept %.3f\n",
                      d->qpos[qa], d->qpos[qa + 1], d->qpos[qa + 2]);
          std::fflush(stdout);
        }
        s.place_mtime = mt;
      } else {
        s.place_mtime = 1.0;   // file absent yet: arm on first appearance too
      }
    }
  }
  // 100 Hz state publish (timestep 0.002 -> every 5 steps)
  const int every = std::max(1, (int)std::lround(0.01 / dt));
  if (++s.pub_cnt < every) return;
  s.pub_cnt = 0;
  if (!s.ok) return;
  // tactile: contact normal-force magnitude summed per pad body
  // SELF-TOUCH EXCLUDED (2026-09-01): with real inter-finger collisions a closed
  // fist reads ~17 N on the pads and the policy (trained with self_collision=False
  // — pads only ever felt object/world contact) slams the fingers shut on phantom
  // "contact". Count a contact only when at least one geom is NOT part of the hand.
  std::vector<double> pad_f(s.pad_geom.size(), 0.0);
  double hit_table = 0.0, hit_robot = 0.0;   // hand-vs-table / hand-vs-robot-body force sums (penalty HUD)
  for (int c = 0; c < d->ncon; ++c) {
    const int g1 = d->contact[c].geom1, g2 = d->contact[c].geom2;
    if (s.geom_is_hand[g1] && s.geom_is_hand[g2]) continue;   // finger-on-finger: invisible to the pads
    // penalty aggregates: any HAND geom against the table / another robot body
    if (s.geom_is_hand[g1] != s.geom_is_hand[g2]) {
      const int go = s.geom_is_hand[g1] ? g2 : g1;            // the non-hand geom
      const char* bn = mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[go]);
      if (bn) {
        mjtNum ff6[6]; mj_contactForce(m, d, c, ff6);
        const double fnc = std::abs(ff6[0]);
        if (std::strncmp(bn, "tbl:", 4) == 0) hit_table += fnc;
        else if (std::strncmp(bn, "obj:", 4) != 0 && m->geom_bodyid[go] != 0) hit_robot += fnc;
      }
    }
    const int p1 = s.geom2pad[g1], p2 = s.geom2pad[g2];
    if (p1 < 0 && p2 < 0) continue;
    mjtNum f6[6];
    mj_contactForce(m, d, c, f6);
    const double fn = std::sqrt(f6[0] * f6[0] + f6[1] * f6[1] + f6[2] * f6[2]);
    if (p1 >= 0) pad_f[p1] += fn;
    if (p2 >= 0) pad_f[p2] += fn;
  }
  if (s.obj_z0 == 0.0) s.obj_z0 = d->xpos[3 * s.obj_body + 2];
  char js[4096];
  int n = std::snprintf(js, sizeof js, "{\"t\":%.3f,\"closure\":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f],"
                        "\"cmd\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],\"pads\":{",
                        d->time, meas[0], meas[1], meas[2], meas[3], meas[4], meas[5],
                        s.applied[0], s.applied[1], s.applied[2], s.applied[3], s.applied[4], s.applied[5]);
  for (size_t i = 0; i < s.pad_geom.size() && n > 0 && n < (int)sizeof js - 64; ++i)
    n += std::snprintf(js + n, sizeof js - n, "%s\"%s\":%.3f", i ? "," : "", s.pad_name[i].c_str(), pad_f[i]);
  auto add_pose = [&](const char* key, int bid) {
    if (n > 0 && n < (int)sizeof js - 160)
      n += std::snprintf(js + n, sizeof js - n,
                         ",\"%s\":{\"p\":[%.4f,%.4f,%.4f],\"q\":[%.5f,%.5f,%.5f,%.5f]}", key,
                         d->xpos[3 * bid], d->xpos[3 * bid + 1], d->xpos[3 * bid + 2],
                         d->xquat[4 * bid], d->xquat[4 * bid + 1], d->xquat[4 * bid + 2], d->xquat[4 * bid + 3]);
  };
  if (n > 0 && n < (int)sizeof js - 4) n += std::snprintf(js + n, sizeof js - n, "}");
  add_pose("obj", s.obj_body); add_pose("torso", s.torso_body);
  if (s.table_body >= 0) add_pose("table", s.table_body);   // the MoveIt planning scene needs it (relay -> /collision_object)
  {  // palm pad GEOM pose (the Isaac palm frame = the fused pad link frame; the mesh origin is identity)
    mjtNum q[4];
    mju_mat2Quat(q, d->geom_xmat + 9 * s.palm_geom);
    if (n > 0 && n < (int)sizeof js - 160)
      n += std::snprintf(js + n, sizeof js - n,
                         ",\"palm\":{\"p\":[%.4f,%.4f,%.4f],\"q\":[%.5f,%.5f,%.5f,%.5f]}",
                         d->geom_xpos[3 * s.palm_geom], d->geom_xpos[3 * s.palm_geom + 1], d->geom_xpos[3 * s.palm_geom + 2],
                         q[0], q[1], q[2], q[3]);
  }
  if (n > 0 && n < (int)sizeof js - 48)
    n += std::snprintf(js + n, sizeof js - n, ",\"obj_lift\":%.4f,\"hit_table\":%.2f,\"hit_robot\":%.2f,\"ncmd\":%d}", d->xpos[3 * s.obj_body + 2] - s.obj_z0, hit_table, hit_robot, s.cmd_count);
  const std::string out(js);
  if (publish_fn()) publish_fn()(out);
  static const char* gf = std::getenv("ARCHB_GRASP_FILE");
  if (gf && *gf && (s.print_cnt % 2 == 0)) write_file_atomic(gf, out);   // 50 Hz file mirror
  if (++s.print_cnt >= 100) {   // 1 Hz console line
    s.print_cnt = 0;
    double pmax = 0.0; for (double f : pad_f) pmax = std::max(pmax, f);
    std::printf("[GRASP] closure cmd [%.2f %.2f %.2f %.2f %.2f %.2f] meas [%.2f %.2f %.2f %.2f %.2f %.2f] "
                "pad max %.1f N  palm (%.3f,%.3f,%.3f) obj (%.3f,%.3f,%.3f) lift %+.3f m  (cmds %d)\n",
                s.applied[0], s.applied[1], s.applied[2], s.applied[3], s.applied[4], s.applied[5],
                meas[0], meas[1], meas[2], meas[3], meas[4], meas[5], pmax,
                d->geom_xpos[3 * s.palm_geom], d->geom_xpos[3 * s.palm_geom + 1], d->geom_xpos[3 * s.palm_geom + 2],
                d->xpos[3 * s.obj_body], d->xpos[3 * s.obj_body + 1], d->xpos[3 * s.obj_body + 2],
                d->xpos[3 * s.obj_body + 2] - s.obj_z0, s.cmd_count);
    std::fflush(stdout);
  }
}

}  // namespace grasp_sim
