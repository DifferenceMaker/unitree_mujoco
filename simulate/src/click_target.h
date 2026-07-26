#pragma once
// click_target.h — ALT+click desk reach targets (the in-GUI replacement for
// `echo "l x y z" >> .arm_targets`).
//
// Flow: UiEvent (simulate.cc hunk) captures ALT+press in the 3D view ->
// set_pending(). Simulate::Sync (second hunk) resolves it with mjv_select
// (same rect math as the stock double-click select) -> resolve() converts the
// world point to the TORSO frame and APPENDS a command line to the
// arm-targets file that arm_ik_commander (Aspired_Robot_Project) already
// watches — IK stays entirely in their resolver, exactly as tested in the
// arch-B `arms` profile.
//
//   physical ALT+LEFT   -> "l x y z"  (left hand,  red ball)
//   physical ALT+RIGHT  -> "r x y z"  (right hand, blue ball)
//   physical ALT+MIDDLE -> "default"  (both arms to the held pose, balls hide)
//
// The red/blue mocap balls (scene bodies target_ball_left/right, see
// scene_sym_soft07_desk.xml) render the LIVE commander targets: update_balls()
// (called from main.cc's PhysicsLoop) re-parses the arm-targets file on mtime
// change — so hand-typed echo lines move the balls too — and places each ball
// at torso_pose * target every step (targets are torso-frame, the balls float
// with the robot like training's debug_vis dots). Ball geoms are click-inert
// (resolve() ignores hits on them; they are contype/conaffinity 0 anyway).
//
// Feature is inert when the scene has no target_ball_* bodies (ids cached as
// -1) and when the file path is unwritable (append fails silently to stderr).
#include <sys/stat.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>

#include <mujoco/mujoco.h>

namespace click_target {

inline std::mutex& mu() { static std::mutex m; return m; }

struct Click {
  bool active = false;
  int button = 0;  // physical: 0=left, 1=right, 2=middle
  double x = 0, y = 0;
  mjrRect r{0, 0, 0, 0};
};

inline Click& pending() { static Click c; return c; }

// last commanded targets (torso frame) for ball rendering
struct Targets {
  bool has_l = false, has_r = false;
  mjtNum l[3] = {0, 0, 0}, r[3] = {0, 0, 0};
};
inline Targets& targets() { static Targets t; return t; }

inline const char* targets_file() {
  static std::string path = [] {
    const char* env = std::getenv("ARM_TARGETS_FILE");
    // launcher runs the sim with cwd = <repo>/simulate/
    return std::string(env ? env : "../mujoco_sim/logs/.arm_targets");
  }();
  return path.c_str();
}

// ---- UI thread ----
inline void set_pending(int phys_button, double x, double y, const mjrRect& rect) {
  std::lock_guard<std::mutex> lk(mu());
  pending() = {true, phys_button, x, y, rect};
}

inline bool has_pending() {
  std::lock_guard<std::mutex> lk(mu());
  return pending().active;
}

inline Click take_pending() {
  std::lock_guard<std::mutex> lk(mu());
  Click c = pending();
  pending().active = false;
  return c;
}

// ---- render thread (Simulate::Sync, sim mutex held) ----
inline void resolve(const mjModel* m, const mjData* d, const mjtNum selpnt[3],
                    int selgeom, int phys_button) {
  // ignore clicks on our own marker balls
  if (selgeom >= 0) {
    const char* gn = mj_id2name(m, mjOBJ_GEOM, selgeom);
    if (gn && std::strncmp(gn, "target_ball_", 12) == 0) return;
  }
  std::ostringstream line;
  if (phys_button == 2) {
    line << "default";
  } else {
    int torso = mj_name2id(m, mjOBJ_BODY, "torso_link");
    if (torso < 0) return;
    mjtNum rel[3], t[3];
    mju_sub3(rel, selpnt, d->xpos + 3 * torso);
    mju_mulMatTVec(t, d->xmat + 9 * torso, rel, 3, 3);
    line.setf(std::ios::fixed);
    line.precision(3);
    line << (phys_button == 0 ? "l " : "r ") << t[0] << " " << t[1] << " " << t[2];
  }
  std::ofstream f(targets_file(), std::ios::app);
  if (!f) {
    std::fprintf(stderr, "[click_target] cannot append to %s\n", targets_file());
    return;
  }
  f << line.str() << "\n";
  std::printf("[click_target] %s -> %s\n", line.str().c_str(), targets_file());
  // targets()/balls update via the file watch in update_balls() (single source
  // of truth: hand-typed echo lines behave identically).
}

// ---- physics thread (main.cc PhysicsLoop, sim mutex held) ----
inline void poll_file() {
  static time_t last_mtime = 0;
  struct stat st;
  if (stat(targets_file(), &st) != 0) return;
  if (st.st_mtime == last_mtime) return;
  last_mtime = st.st_mtime;
  std::ifstream f(targets_file());
  if (!f) return;
  Targets t;  // replay the whole file, last state wins (mirrors arm_ik_commander)
  std::string cmd;
  double x, y, z;
  while (f >> cmd) {
    if (cmd == "l" && (f >> x >> y >> z)) { t.has_l = true; t.l[0]=x; t.l[1]=y; t.l[2]=z; }
    else if (cmd == "r" && (f >> x >> y >> z)) { t.has_r = true; t.r[0]=x; t.r[1]=y; t.r[2]=z; }
    else if (cmd == "default") { t = Targets{}; }
    else { f.clear(); std::getline(f, cmd); }  // skip malformed rest-of-line
  }
  std::lock_guard<std::mutex> lk(mu());
  targets() = t;
}

inline void update_balls(const mjModel* m, mjData* d) {
  static int mocap_l = -2, mocap_r = -2, torso = -2;  // -2 = unresolved
  if (mocap_l == -2) {
    int bl = mj_name2id(m, mjOBJ_BODY, "target_ball_left");
    int br = mj_name2id(m, mjOBJ_BODY, "target_ball_right");
    mocap_l = (bl >= 0) ? m->body_mocapid[bl] : -1;
    mocap_r = (br >= 0) ? m->body_mocapid[br] : -1;
    torso = mj_name2id(m, mjOBJ_BODY, "torso_link");
  }
  if ((mocap_l < 0 && mocap_r < 0) || torso < 0) return;  // scene without balls
  poll_file();
  Targets t;
  {
    std::lock_guard<std::mutex> lk(mu());
    t = targets();
  }
  const mjtNum* xpos = d->xpos + 3 * torso;
  const mjtNum* xmat = d->xmat + 9 * torso;
  auto place = [&](int mid, bool has, const mjtNum* tt) {
    if (mid < 0) return;
    if (!has) {  // park below the floor
      d->mocap_pos[3 * mid + 0] = 0; d->mocap_pos[3 * mid + 1] = 0; d->mocap_pos[3 * mid + 2] = -1;
      return;
    }
    mjtNum w[3];
    mju_mulMatVec(w, xmat, tt, 3, 3);
    mju_addTo3(w, xpos);
    d->mocap_pos[3 * mid + 0] = w[0]; d->mocap_pos[3 * mid + 1] = w[1]; d->mocap_pos[3 * mid + 2] = w[2];
  };
  place(mocap_l, t.has_l, t.l);
  place(mocap_r, t.has_r, t.r);
}

}  // namespace click_target
