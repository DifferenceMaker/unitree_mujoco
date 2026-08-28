// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// !!! hack code: make glfw_adapter.window_ public
#define private public
#include "glfw_adapter.h"
#undef private

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <sstream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <sys/stat.h>

#include "click_target.h"
#include "anchor_pub.h"
#include <thread>

#include <mujoco/mujoco.h>
#include "simulate.h"
#include "array_safety.h"
#include "unitree_sdk2_bridge.h"
#include "param.h"
#include "policy_hud.h"
#include "arm_gui.h"
#include "grasp_sim.h"
#include <array>
#include <unitree/idl/ros2/String_.hpp>

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"
#define NUM_MOTOR_IDL_GO 20

extern "C"
{
#if defined(_WIN32) || defined(__CYGWIN__)
#include <windows.h>
#else
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
#include <sys/errno.h>
#include <unistd.h>
#endif
}

class ElasticBand
{
public:
  ElasticBand(){};
  void Advance(std::vector<double> x, std::vector<double> dx)
  {
    std::vector<double> delta_x = {0.0, 0.0, 0.0};
    delta_x[0] = point_[0] - x[0];
    delta_x[1] = point_[1] - x[1];
    delta_x[2] = point_[2] - x[2];
    double distance = sqrt(delta_x[0] * delta_x[0] + delta_x[1] * delta_x[1] + delta_x[2] * delta_x[2]);

    std::vector<double> direction = {0.0, 0.0, 0.0};
    direction[0] = delta_x[0] / distance;
    direction[1] = delta_x[1] / distance;
    direction[2] = delta_x[2] / distance;

    double v = dx[0] * direction[0] + dx[1] * direction[1] + dx[2] * direction[2];

    f_[0] = (stiffness_ * (distance - length_) - damping_ * v) * direction[0];
    f_[1] = (stiffness_ * (distance - length_) - damping_ * v) * direction[1];
    f_[2] = (stiffness_ * (distance - length_) - damping_ * v) * direction[2];
  }


  double stiffness_ = 200;
  double damping_ = 100;
  std::vector<double> point_ = {0, 0, 3};
  double length_ = 0.0;
  bool enable_ = true;
  std::vector<double> f_ = {0, 0, 0};
};
inline ElasticBand elastic_band;

// Decoupled arm-pose command publisher (sim -> controller on rt/arm_pose_cmd).
// Declared here so the bridge thread (which creates it) and user_key_cb (which
// uses it) both see it. Initialized in UnitreeSdk2BridgeThread after DDS init.
static std::shared_ptr<unitree::robot::ChannelPublisher<std_msgs::msg::dds_::String_>> g_arm_cmd_pub;
// Anchor point publisher (rt/anchor_point): base-frame anchor for 90-obs policies.
static std::shared_ptr<unitree::robot::ChannelPublisher<std_msgs::msg::dds_::String_>> g_anchor_pub;
// Lean command publisher (rt/lean_cmd): {"lean": rad} from the Lean slider — Desk6/6b lean class.
static std::shared_ptr<unitree::robot::ChannelPublisher<std_msgs::msg::dds_::String_>> g_lean_pub;

// Metrics zero command publisher (sim -> sidecar on rt/metrics_cmd). Lets a sim
// keypress reset the sidecar's lean/touchdown counters from the GUI.
static std::shared_ptr<unitree::robot::ChannelPublisher<std_msgs::msg::dds_::String_>> g_metrics_cmd_pub;

// Ground-truth base pose publisher (rt/sim_base_pose): world-frame base state
// + wrist positions for the sidecar reward LEDGER (2026-08-21). Sim2sim-only
// privilege — the reward functions need world pose, which rt/lowstate lacks.
// JSON: {"p":[xyz],"q":[wxyz],"v":[world lin],"w":[BODY ang],"lw":[xyz],"rw":[xyz]}
static std::shared_ptr<unitree::robot::ChannelPublisher<std_msgs::msg::dds_::String_>> g_sim_pose_pub;

// Scripted push (sim2sim harness): an instantaneous world-frame base velocity
// change requested from the stdin command thread, applied to the free-joint
// linear DoFs inside the physics lock. Free-joint translational qvel is
// expressed in the world frame, so this is a world-frame push by definition.
struct ScriptedPush
{
  std::mutex mtx;
  bool pending = false;
  double vx = 0.0;
  double vy = 0.0;
};
inline ScriptedPush scripted_push;

// Reads harness commands from stdin:
//   push <vx> <vy>   instantaneous base velocity change (m/s, world frame)
// Each applied push is printed with wall + sim timestamps so runs can be
// reproduced from the log.
void StdinCommandThread()
{
  std::string line;
  while (std::getline(std::cin, line))
  {
    std::istringstream iss(line);
    std::string cmd;
    if (!(iss >> cmd))
      continue;
    if (cmd == "push")
    {
      double vx, vy;
      if (!(iss >> vx >> vy))
      {
        std::printf("[PUSH] usage: push <vx> <vy>   (m/s, world frame)\n");
        continue;
      }
      std::lock_guard<std::mutex> lk(scripted_push.mtx);
      scripted_push.vx = vx;
      scripted_push.vy = vy;
      scripted_push.pending = true;
    }
    else
    {
      std::printf("[CMD] unknown: '%s' (commands: push <vx> <vy>)\n", cmd.c_str());
    }
  }
}


namespace
{
  namespace mj = ::mujoco;
  namespace mju = ::mujoco::sample_util;

  // constants
  const double syncMisalign = 0.1;       // maximum mis-alignment before re-sync (simulation seconds)
  const double simRefreshFraction = 0.7; // fraction of refresh available for simulation
  const int kErrorLength = 1024;         // load error string length

  // model and data
  mjModel *m = nullptr;
  mjData *d = nullptr;

  // control noise variables
  mjtNum *ctrlnoise = nullptr;

  using Seconds = std::chrono::duration<double>;

  //---------------------------------------- plugin handling -----------------------------------------

  // return the path to the directory containing the current executable
  // used to determine the location of auto-loaded plugin libraries
  std::string getExecutableDir()
  {
#if defined(_WIN32) || defined(__CYGWIN__)
    constexpr char kPathSep = '\\';
    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      DWORD buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
        if (written < buf_size)
        {
          success = true;
        }
        else if (written == buf_size)
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
        else
        {
          std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
          return "";
        }
      }
      return realpath.get();
    }();
#else
    constexpr char kPathSep = '/';
#if defined(__APPLE__)
    std::unique_ptr<char[]> buf(nullptr);
    {
      std::uint32_t buf_size = 0;
      _NSGetExecutablePath(nullptr, &buf_size);
      buf.reset(new char[buf_size]);
      if (!buf)
      {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }
      if (_NSGetExecutablePath(buf.get(), &buf_size))
      {
        std::cerr << "unexpected error from _NSGetExecutablePath\n";
      }
    }
    const char *path = buf.get();
#else
    const char *path = "/proc/self/exe";
#endif
    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      std::uint32_t buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        std::size_t written = readlink(path, realpath.get(), buf_size);
        if (written < buf_size)
        {
          realpath.get()[written] = '\0';
          success = true;
        }
        else if (written == -1)
        {
          if (errno == EINVAL)
          {
            // path is already not a symlink, just use it
            return path;
          }

          std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
          return "";
        }
        else
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
      }
      return realpath.get();
    }();
#endif

    if (realpath.empty())
    {
      return "";
    }

    for (std::size_t i = realpath.size() - 1; i > 0; --i)
    {
      if (realpath.c_str()[i] == kPathSep)
      {
        return realpath.substr(0, i);
      }
    }

    // don't scan through the entire file system's root
    return "";
  }

  // scan for libraries in the plugin directory to load additional plugins
  void scanPluginLibraries()
  {
    // check and print plugins that are linked directly into the executable
    int nplugin = mjp_pluginCount();
    if (nplugin)
    {
      std::printf("Built-in plugins:\n");
      for (int i = 0; i < nplugin; ++i)
      {
        std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
      }
    }

    // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
    const std::string sep = "\\";
#else
    const std::string sep = "/";
#endif

    // try to open the ${EXECDIR}/plugin directory
    // ${EXECDIR} is the directory containing the simulate binary itself
    const std::string executable_dir = getExecutableDir();
    if (executable_dir.empty())
    {
      return;
    }

    const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
    mj_loadAllPluginLibraries(
        plugin_dir.c_str(), +[](const char *filename, int first, int count)
                            {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        } });
  }

  //------------------------------------------- simulation -------------------------------------------

  mjModel *LoadModel(const char *file, mj::Simulate &sim)
  {
    // this copy is needed so that the mju::strlen call below compiles
    char filename[mj::Simulate::kMaxFilenameLength];
    mju::strcpy_arr(filename, file);

    // make sure filename is not empty
    if (!filename[0])
    {
      return nullptr;
    }

    // load and compile
    char loadError[kErrorLength] = "";
    mjModel *mnew = 0;
    if (mju::strlen_arr(filename) > 4 &&
        !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                      mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4))
    {
      mnew = mj_loadModel(filename, nullptr);
      if (!mnew)
      {
        mju::strcpy_arr(loadError, "could not load binary model");
      }
    }
    else
    {
      mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
      // remove trailing newline character from loadError
      if (loadError[0])
      {
        int error_length = mju::strlen_arr(loadError);
        if (loadError[error_length - 1] == '\n')
        {
          loadError[error_length - 1] = '\0';
        }
      }
    }

    mju::strcpy_arr(sim.load_error, loadError);

    if (!mnew)
    {
      std::printf("%s\n", loadError);
      return nullptr;
    }

    // compiler warning: print and pause
    if (loadError[0])
    {
      // mj_forward() below will print the warning message
      std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
      sim.run = 0;
    }

    return mnew;
  }

  // simulate in background thread (while rendering in main thread)
  void PhysicsLoop(mj::Simulate &sim)
  {
    // cpu-sim syncronization point
    std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;

    // ChannelFactory::Instance()->Init(0);
    // UnitreeDds ud(d);

    // run until asked to exit
    while (!sim.exitrequest.load())
    {
      if (sim.droploadrequest.load())
      {
        sim.LoadMessage(sim.dropfilename);
        mjModel *mnew = LoadModel(sim.dropfilename, sim);
        sim.droploadrequest.store(false);

        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.dropfilename);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          mj_forward(m, d);

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = (mjtNum *)malloc(sizeof(mjtNum) * m->nu);
          mju_zero(ctrlnoise, m->nu);
        }
        else
        {
          sim.LoadMessageClear();
        }
      }

      if (sim.uiloadrequest.load())
      {
        sim.uiloadrequest.fetch_sub(1);
        sim.LoadMessage(sim.filename);
        mjModel *mnew = LoadModel(sim.filename, sim);
        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.filename);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          mj_forward(m, d);

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
          mju_zero(ctrlnoise, m->nu);
        }
        else
        {
          sim.LoadMessageClear();
        }
      }

      // sleep for 1 ms or yield, to let main thread run
      //  yield results in busy wait - which has better timing but kills battery life
      if (sim.run && sim.busywait)
      {
        std::this_thread::yield();
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      {
        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        // run only if model is present
        if (m)
        {
          // running
          if (sim.run)
          {
            bool stepped = false;

            // scripted push (harness): apply pending base velocity change
            {
              std::lock_guard<std::mutex> plk(scripted_push.mtx);
              if (scripted_push.pending)
              {
                if (m->njnt > 0 && m->jnt_type[0] == mjJNT_FREE)
                {
                  d->qvel[0] += scripted_push.vx;
                  d->qvel[1] += scripted_push.vy;
                  auto now = std::chrono::system_clock::now();
                  std::time_t tt = std::chrono::system_clock::to_time_t(now);
                  char tbuf[32];
                  std::strftime(tbuf, sizeof(tbuf), "%H:%M:%S", std::localtime(&tt));
                  std::printf("[PUSH] wall=%s sim_t=%.3f dvx=%+.2f dvy=%+.2f m/s\n",
                              tbuf, d->time, scripted_push.vx, scripted_push.vy);
                  std::fflush(stdout);
                }
                else
                {
                  std::printf("[PUSH] ignored: model has no free-joint base\n");
                }
                scripted_push.pending = false;
              }
            }

            // desk reach-target balls: reflect the live arm-targets file
            // (alt-click or hand-typed lines) at the current torso pose.
            click_target::update_balls(m, d);
            // grasp rig: finger servos from the Inspire emulator + hand/object state out
            grasp_sim::step(m, d);
            // anchor: yellow ball + base-frame point on rt/anchor_point (~50 Hz)
            {
              double rel_b[3];
              if (anchor_pub::step(m, d, rel_b)) {
                static int anchor_cnt = 0;
                if (++anchor_cnt >= 10 && g_anchor_pub) {
                  anchor_cnt = 0;
                  char js[96];
                  std::snprintf(js, sizeof js, "{\"p\":[%.4f,%.4f,%.4f]}", rel_b[0], rel_b[1], rel_b[2]);
                  std_msgs::msg::dds_::String_ amsg;
                  amsg.data(js);
                  g_anchor_pub->Write(amsg, 0);
                }
              }
            }

            // ground-truth state for the sidecar reward ledger (~50 Hz).
            // v3 (2026-08-21): + click-target ball positions (desk_reach),
            // + per-body DESK-contact forces (undesired_contacts incl pelvis),
            // + hand-vs-desk force (table/desk_hit), + foot positions and
            // floor-contact forces (feet_slide / feet_too_near).
            {
              static int pose_cnt = 0;
              if (++pose_cnt >= 10 && g_sim_pose_pub) {
                pose_cnt = 0;
                static int fj = -1, lw_id = -2, rw_id = -2;
                static int tl_id = -2, tr_id = -2, desk_id = -2;
                static int fl_id = -2, fr_id = -2;
                // undesired scope: pelvis + torso + all hip links + knees
                static std::vector<int> und_ids;
                static std::vector<int> hand_l_ids, hand_r_ids;
                if (fj == -1) {
                  for (int j = 0; j < m->njnt; ++j)
                    if (m->jnt_type[j] == mjJNT_FREE) { fj = j; break; }
                  lw_id = mj_name2id(m, mjOBJ_BODY, "left_wrist_yaw_link");
                  rw_id = mj_name2id(m, mjOBJ_BODY, "right_wrist_yaw_link");
                  tl_id = mj_name2id(m, mjOBJ_BODY, "target_ball_left");
                  tr_id = mj_name2id(m, mjOBJ_BODY, "target_ball_right");
                  desk_id = mj_name2id(m, mjOBJ_BODY, "desk");
                  fl_id = mj_name2id(m, mjOBJ_BODY, "left_ankle_roll_link");
                  fr_id = mj_name2id(m, mjOBJ_BODY, "right_ankle_roll_link");
                  for (int b = 0; b < m->nbody; ++b) {
                    const char* bn = mj_id2name(m, mjOBJ_BODY, b);
                    if (!bn) continue;
                    const std::string s(bn);
                    if (s == "pelvis" || s == "torso_link" ||
                        s.find("hip") != std::string::npos ||
                        s.find("knee") != std::string::npos)
                      und_ids.push_back(b);
                    const bool handish =
                        s.find("wrist") != std::string::npos ||
                        s.find("palm") != std::string::npos ||
                        s.find("hand") != std::string::npos ||
                        s.find("thumb") != std::string::npos ||
                        s.find("index") != std::string::npos ||
                        s.find("middle") != std::string::npos ||
                        s.find("ring") != std::string::npos ||
                        s.find("pinky") != std::string::npos ||
                        s.find("little") != std::string::npos;
                    if (handish && s.find("left") != std::string::npos) hand_l_ids.push_back(b);
                    if (handish && (s.find("right") != std::string::npos ||
                                    s[0] == 'R')) hand_r_ids.push_back(b);
                  }
                }
                if (fj >= 0) {
                  // per-body desk-contact force + foot floor-contact force
                  std::vector<double> und_f(und_ids.size(), 0.0);
                  double hand_f[2] = {0.0, 0.0}, foot_f[2] = {0.0, 0.0};
                  for (int c = 0; c < d->ncon; ++c) {
                    const int b1 = m->geom_bodyid[d->contact[c].geom1];
                    const int b2 = m->geom_bodyid[d->contact[c].geom2];
                    mjtNum f6[6];
                    mj_contactForce(m, d, c, f6);
                    const double fn = std::sqrt(f6[0]*f6[0] + f6[1]*f6[1] + f6[2]*f6[2]);
                    const bool desk1 = (b1 == desk_id), desk2 = (b2 == desk_id);
                    const int other = desk1 ? b2 : (desk2 ? b1 : -1);
                    if (other >= 0) {
                      for (size_t k = 0; k < und_ids.size(); ++k)
                        if (und_ids[k] == other) und_f[k] += fn;
                      for (int hb : hand_l_ids) if (hb == other) hand_f[0] += fn;
                      for (int hb : hand_r_ids) if (hb == other) hand_f[1] += fn;
                    }
                    // feet vs anything (floor): world body id 0
                    if (b1 == fl_id || b2 == fl_id) foot_f[0] += fn;
                    if (b1 == fr_id || b2 == fr_id) foot_f[1] += fn;
                  }
                  double und_max = 0.0; int und_cnt = 0;
                  for (double f : und_f) { if (f > 1.0) ++und_cnt; if (f > und_max) und_max = f; }

                  const int qa = m->jnt_qposadr[fj], va = m->jnt_dofadr[fj];
                  char js[1024];
                  int n = std::snprintf(js, sizeof js,
                      "{\"p\":[%.4f,%.4f,%.4f],\"q\":[%.5f,%.5f,%.5f,%.5f],"
                      "\"v\":[%.4f,%.4f,%.4f],\"w\":[%.4f,%.4f,%.4f],"
                      "\"ucnt\":%d,\"umax\":%.1f,\"th\":[%.1f,%.1f],\"fc\":[%.1f,%.1f]",
                      d->qpos[qa], d->qpos[qa + 1], d->qpos[qa + 2],
                      d->qpos[qa + 3], d->qpos[qa + 4], d->qpos[qa + 5], d->qpos[qa + 6],
                      d->qvel[va], d->qvel[va + 1], d->qvel[va + 2],
                      d->qvel[va + 3], d->qvel[va + 4], d->qvel[va + 5],
                      und_cnt, und_max, hand_f[0], hand_f[1], foot_f[0], foot_f[1]);
                  auto add_body = [&](const char* key, int bid) {
                    if (bid >= 0 && n > 0 && n < (int)sizeof js - 96)
                      n += std::snprintf(js + n, sizeof js - n,
                          ",\"%s\":[%.4f,%.4f,%.4f]", key,
                          d->xpos[3 * bid], d->xpos[3 * bid + 1], d->xpos[3 * bid + 2]);
                  };
                  add_body("lw", lw_id); add_body("rw", rw_id);
                  add_body("fl", fl_id); add_body("fr", fr_id);
                  add_body("tl", tl_id); add_body("tr", tr_id);
                  if (n > 0 && n < (int)sizeof js - 2)
                    std::snprintf(js + n, sizeof js - n, "}");
                  std_msgs::msg::dds_::String_ pmsg;
                  pmsg.data(js);
                  g_sim_pose_pub->Write(pmsg, 0);
                }
              }
            }

            // record cpu time at start of iteration
            const auto startCPU = mj::Simulate::Clock::now();

            // elapsed CPU and simulation time since last sync
            const auto elapsedCPU = startCPU - syncCPU;
            double elapsedSim = d->time - syncSim;

            // inject noise
            if (sim.ctrl_noise_std)
            {
              // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
              mjtNum rate = mju_exp(-m->opt.timestep / mju_max(sim.ctrl_noise_rate, mjMINVAL));
              mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1 - rate * rate);

              for (int i = 0; i < m->nu; i++)
              {
                // update noise
                ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

                // apply noise
                d->ctrl[i] = ctrlnoise[i];
              }
            }

            // requested slow-down factor
            double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

            // misalignment condition: distance from target sim time is bigger than syncmisalign
            bool misaligned =
                mju_abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;

            // out-of-sync (for any reason): reset sync times, step
            if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
                misaligned || sim.speed_changed)
            {
              // re-sync
              syncCPU = startCPU;
              syncSim = d->time;
              sim.speed_changed = false;

              // run single step, let next iteration deal with timing
              mj_step(m, d);
              stepped = true;
            }

            // in-sync: step until ahead of cpu
            else
            {
              bool measured = false;
              mjtNum prevSim = d->time;

              double refreshTime = simRefreshFraction / sim.refresh_rate;

              // step while sim lags behind cpu and within refreshTime
              while (Seconds((d->time - syncSim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                     mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime))
              {
                // measure slowdown before first step
                if (!measured && elapsedSim)
                {
                  sim.measured_slowdown =
                      std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                  measured = true;
                }

                // elastic band on base link
                if (param::config.enable_elastic_band == 1)
                {
                  if (elastic_band.enable_)
                  {
                    std::vector<double> x = {d->qpos[0], d->qpos[1], d->qpos[2]};
                    std::vector<double> dx = {d->qvel[0], d->qvel[1], d->qvel[2]};

                    elastic_band.Advance(x, dx);

                    d->xfrc_applied[param::config.band_attached_link] = elastic_band.f_[0];
                    d->xfrc_applied[param::config.band_attached_link + 1] = elastic_band.f_[1];
                    d->xfrc_applied[param::config.band_attached_link + 2] = elastic_band.f_[2];
                  }
                }

                // call mj_step
                mj_step(m, d);
                stepped = true;

                // break if reset
                if (d->time < prevSim)
                {
                  break;
                }
              }
            }

            // save current state to history buffer
            if (stepped)
            {
              sim.AddToHistory();
            }
          }

          // paused
          else
          {
            // run mj_forward, to update rendering and joint sliders
            mj_forward(m, d);
            sim.speed_changed = true;
          }
        }
      } // release std::lock_guard<std::mutex>
    }
  }
} // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate *sim, const char *filename)
{
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr)
  {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m)
      d = mj_makeData(m);
    if (d)
    {
      sim->Load(m, d, filename);
      mj_forward(m, d);

      // ── startup banner: confirm which robot model actually loaded ──
      {
        const double total_mass = mj_getTotalmass(m);
        std::printf(
            "\n============================================================\n"
            " unitree_mujoco loaded  |  robot=%s\n"
            " scene=%s\n"
            " nq=%d  nv=%d  nbody=%d  total_mass=%.3f kg\n",
            param::config.robot.c_str(), filename, m->nq, m->nv, m->nbody,
            total_mass);
        if (param::config.robot == "h1_2")
          std::printf(
              " corrected h1_2 body expects 76.484 kg (D and SYM share it): %s\n",
              (total_mass > 76.0 && total_mass < 77.0) ? "OK" : "MISMATCH");
        std::printf(
            "============================================================\n\n");
        std::fflush(stdout);
      }

      // allocate ctrlnoise
      free(ctrlnoise);
      ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
      mju_zero(ctrlnoise, m->nu);
    }
    else
    {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);

  exit(0);
}

void *UnitreeSdk2BridgeThread(void *arg)
{
  // Wait for mujoco data
  while (true)
  {
    if (d)
    {
      std::cout << "Mujoco data is prepared" << std::endl;
      break;
    }
    usleep(500000);
  }

  unitree::robot::ChannelFactory::Instance()->Init(param::config.domain_id, param::config.interface);

  // Sim2sim HUD: subscribe to the controller's policy status and stash it for the
  // render overlay (see policy_hud.h / simulate.cc). Kept alive for the thread's life.
  static auto policy_status_sub =
      std::make_shared<unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_>>(
          "rt/policy_status", [](const void *msg) {
            policy_hud::set_from_json(
                reinterpret_cast<const std_msgs::msg::dds_::String_ *>(msg)->data());
          });
  policy_status_sub->InitChannel();

  // Sim2sim arm-command publisher (rt/arm_pose_cmd): keyboard presets (and later
  // the mjUI GUI) publish arm poses for the controller's External mode.
  g_arm_cmd_pub =
      std::make_shared<unitree::robot::ChannelPublisher<std_msgs::msg::dds_::String_>>(
          "rt/arm_pose_cmd");
  g_arm_cmd_pub->InitChannel();
  g_anchor_pub =
      std::make_shared<unitree::robot::ChannelPublisher<std_msgs::msg::dds_::String_>>(
          "rt/anchor_point");
  g_anchor_pub->InitChannel();
  g_lean_pub =
      std::make_shared<unitree::robot::ChannelPublisher<std_msgs::msg::dds_::String_>>(
          "rt/lean_cmd");
  g_lean_pub->InitChannel();
  g_sim_pose_pub =
      std::make_shared<unitree::robot::ChannelPublisher<std_msgs::msg::dds_::String_>>(
          "rt/sim_base_pose");
  g_sim_pose_pub->InitChannel();

  // GRASP RIG (ARCHB_GRASP=1): hand state out / closure commands in (see grasp_sim.h)
  if (grasp_sim::enabled()) {
    static auto grasp_state_pub =
        std::make_shared<unitree::robot::ChannelPublisher<std_msgs::msg::dds_::String_>>(
            "rt/sim_hand/state");
    grasp_state_pub->InitChannel();
    grasp_sim::publish_fn() = [](const std::string& js) {
      std_msgs::msg::dds_::String_ msg;
      msg.data(js);
      grasp_state_pub->Write(msg, 0);
    };
    static auto grasp_cmd_sub =
        std::make_shared<unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_>>(
            "rt/sim_hand/cmd", [](const void *msg) {
              grasp_sim::set_cmd_json(
                  reinterpret_cast<const std_msgs::msg::dds_::String_ *>(msg)->data());
            });
    grasp_cmd_sub->InitChannel();
    std::cout << "[GRASP] rt/sim_hand/state + rt/sim_hand/cmd channels up" << std::endl;
  }

  // Keyboard FSM control: digits in the sim window -> rt/fsm_cmd -> controller
  // FSMRequest (key map shown in the HUD's "FSM keys:" line).
  static auto fsm_cmd_pub =
      std::make_shared<unitree::robot::ChannelPublisher<std_msgs::msg::dds_::String_>>(
          "rt/fsm_cmd");
  fsm_cmd_pub->InitChannel();
  policy_hud::fsm_publish_fn() = [](char key) {
    std_msgs::msg::dds_::String_ m;
    m.data(std::string(1, key));
    fsm_cmd_pub->Write(m, 0);
    std::cout << "[SIM] fsm key '" << key << "' -> rt/fsm_cmd" << std::endl;
  };

  // Sidecar metrics HUD: subscribe to rt/balance_metrics (published by
  // balance_metrics.py) and stash it for the bottom-left overlay.
  static auto metrics_sub =
      std::make_shared<unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_>>(
          "rt/balance_metrics", [](const void *msg) {
            policy_hud::set_metrics_from_json(
                reinterpret_cast<const std_msgs::msg::dds_::String_ *>(msg)->data());
          });
  metrics_sub->InitChannel();

  // Metrics zero command (rt/metrics_cmd): sim keypress -> sidecar reset.
  g_metrics_cmd_pub =
      std::make_shared<unitree::robot::ChannelPublisher<std_msgs::msg::dds_::String_>>(
          "rt/metrics_cmd");
  g_metrics_cmd_pub->InitChannel();


  int body_id = mj_name2id(m, mjOBJ_BODY, "torso_link");
  if (body_id < 0) {
    body_id = mj_name2id(m, mjOBJ_BODY, "base_link");
  }
  param::config.band_attached_link = 6 * body_id;
  
  std::unique_ptr<UnitreeSDK2BridgeBase> interface = nullptr;
  if (m->nu > NUM_MOTOR_IDL_GO) {
    interface = std::make_unique<G1Bridge>(m, d);
  } else {
    interface = std::make_unique<Go2Bridge>(m, d);
  }
  interface->start();
  
  while (true)
  {
    sleep(1);
  }
}
//------------------------------------------ main --------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char *title, const char *msg);
static const char *rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char *msg)
{
  rosetta_error_msg = msg;
}
#endif

// Phase-B arm-pose presets: keyboard keys publish a 14-dim arm pose that the
// controller (ArmPosePublisher External mode) slews to. Phase D will replace these
// presets with mjUI sliders driving this same publisher/topic. DEBUG/test path only.
// 14-dim arm order: shPitch L R, shRoll L R, shYaw L R, elbPitch L R,
//                   elbRoll L R, wrPitch L R, wrYaw L R.
static void publish_arm_pose(const std::array<float, 14> &pose, float transition_s) {
  if (!g_arm_cmd_pub) return;
  std::ostringstream js;
  js << "{\"pose\":[";
  for (size_t i = 0; i < 14; ++i) js << (i ? "," : "") << pose[i];
  js << "],\"transition_s\":" << transition_s << "}";
  std_msgs::msg::dds_::String_ msg;
  msg.data(js.str());
  g_arm_cmd_pub->Write(msg, 0);
  std::cout << "[ARM_CMD] published preset arm pose (transition " << transition_s << "s)" << std::endl;
}

static void publish_lean(float lean_rad) {
  std::ostringstream js;
  js << "{\"lean\":" << lean_rad << "}";
  // FILE path (ARCHB_LEAN_FILE, shared mount): the ActionModule venv in the arch-B
  // container has no unitree_sdk2py, so lean_relay.py polls this file — the same
  // mechanism as .arm_targets/.arm_wish. Atomic write (tmp + rename).
  if (const char* lf = std::getenv("ARCHB_LEAN_FILE"); lf && *lf) {
    const std::string tmp = std::string(lf) + ".tmp";
    { std::ofstream f(tmp); f << js.str() << "\n"; }
    std::rename(tmp.c_str(), lf);
  }
  if (g_lean_pub) {
    std_msgs::msg::dds_::String_ msg;
    msg.data(js.str());
    g_lean_pub->Write(msg, 0);
  }
  std::cout << "[LEAN_CMD] lean " << lean_rad << " rad (file+dds)" << std::endl;
}

// user keyboard callback
void user_key_cb(GLFWwindow* window, int key, int scancode, int act, int mods) {
  if (act==GLFW_PRESS)
  {
    if(param::config.enable_elastic_band == 1) {
      if (key==GLFW_KEY_9) {
        elastic_band.enable_ = !elastic_band.enable_;
      } else if (key==GLFW_KEY_7 || key==GLFW_KEY_UP) {
        elastic_band.length_ -= 0.1;
      } else if (key==GLFW_KEY_8 || key==GLFW_KEY_DOWN) {
        elastic_band.length_ += 0.1;
      }
    }
    if (key==GLFW_KEY_L) {
      // reward-ledger side panel: hidden -> compact -> full (policy_hud)
      policy_hud::ledger_cycle();
    }
    if(key==GLFW_KEY_BACKSPACE) {
      mj_resetData(m, d);
      mj_forward(m, d);
    }

    // Keyboard FSM control: digits 0-6 -> rt/fsm_cmd (key map = HUD top-right
    // list; controller side derives the same map). 7/8/9 stay elastic-band.
    // NUMPAD 0-9: full FSM range without colliding with the harness keys
    // (main-row 7/8/9 = band up/lower/toggle). Main-row 0-6 kept as aliases.
    if (key >= GLFW_KEY_KP_0 && key <= GLFW_KEY_KP_9) {
      policy_hud::request_fsm(static_cast<char>('0' + (key - GLFW_KEY_KP_0)));
    } else if (key >= GLFW_KEY_0 && key <= GLFW_KEY_6) {
      policy_hud::request_fsm(static_cast<char>('0' + (key - GLFW_KEY_0)));
    }
    // Arm-pose presets -> rt/arm_pose_cmd (controller enters External mode and slews).
    // 14-dim: shPitch L R, shRoll L R, shYaw L R, elbPitch L R, elbRoll L R, wrPitch L R, wrYaw L R.
    // shoulder_pitch: 0=down, NEGATIVE=forward (-1.57=horizontal). elbow: 0=90deg-bent,
    // ~1.57=straight. (Tunable starting poses.)
    constexpr float kTrans = 3.0f;
    if (key==GLFW_KEY_J) {        // forward reach, ~just below shoulder, arm near-straight
      publish_arm_pose({-1.4f,-1.4f, 0.0f,0.0f, 0.0f,0.0f, 1.3f,1.3f, 0.0f,0.0f, 0.0f,0.0f, 0.0f,0.0f}, kTrans);
    } else if (key==GLFW_KEY_K) { // carry: upper arms forward-down, forearms forward-horizontal (~90deg)
      publish_arm_pose({-0.8f,-0.8f, 0.0f,0.0f, 0.0f,0.0f, 0.2f,0.2f, 0.0f,0.0f, 0.0f,0.0f, 0.0f,0.0f}, kTrans);
    } else if (key==GLFW_KEY_L) { // default arms (rest)
      publish_arm_pose({0.4f,0.4f, 0.0f,0.0f, 0.0f,0.0f, 0.3f,0.3f, 0.0f,0.0f, 0.0f,0.0f, 0.0f,0.0f}, kTrans);
    }

    // Hand payload (sim-local, Feature C): adjust mass added to BOTH wrist_yaw
    // bodies live.  ] = +0.5 kg/hand,  [ = -0.5 kg/hand,  \ = zero. Point-mass
    // approximation (inertia of the load is ignored). Matches training's
    // add_hand_payload (U(0,3) kg on *_wrist_yaw_link).
    {
      static int wl = -2, wr = -2;           // cached body ids (-2 = unresolved)
      static double base_l = 0.0, base_r = 0.0;
      static float payload = 0.0f;
      if (wl == -2) {
        wl = mj_name2id(m, mjOBJ_BODY, "left_wrist_yaw_link");
        wr = mj_name2id(m, mjOBJ_BODY, "right_wrist_yaw_link");
        if (wl >= 0) base_l = m->body_mass[wl];
        if (wr >= 0) base_r = m->body_mass[wr];
      }
      bool changed = false;
      if (key==GLFW_KEY_RIGHT_BRACKET)      { payload += 0.5f; changed = true; }
      else if (key==GLFW_KEY_LEFT_BRACKET)  { payload -= 0.5f; changed = true; }
      else if (key==GLFW_KEY_BACKSLASH)     { payload  = 0.0f; changed = true; }
      if (changed) {
        if (payload < 0.0f) payload = 0.0f;
        if (payload > 10.0f) payload = 10.0f;
        if (wl >= 0) m->body_mass[wl] = base_l + payload;  // picked up next mj_step
        if (wr >= 0) m->body_mass[wr] = base_r + payload;
        char buf[48];
        std::snprintf(buf, sizeof(buf), "payload: %.1f kg/hand", payload);
        policy_hud::set_payload(buf);
        std::cout << "[PAYLOAD] " << payload << " kg/hand" << std::endl;
      }
    }

    // Zero the sidecar's metrics counters from the GUI (Feature E).
    if (key==GLFW_KEY_Z && g_metrics_cmd_pub) {
      std_msgs::msg::dds_::String_ cmd;
      cmd.data("zero");
      g_metrics_cmd_pub->Write(cmd, 0);
      std::cout << "[METRICS] sent zero" << std::endl;
    }
  }
}

// run event loop
int main(int argc, char **argv)
{
#if defined(GLFW_PLATFORM_X11)
  // Force the X11/XWayland backend (2026-08-21): GLFW's Wayland backend
  // SEGFAULTS in libwayland-client after minutes of running (5 identical
  // kernel traces), and a native-Wayland window is invisible to
  // wmctrl/x11grab (--record). Neither unsetting WAYLAND_DISPLAY nor a
  // GLFW_PLATFORM env var helps — wl_display_connect(NULL) falls back to the
  // XDG_RUNTIME_DIR/wayland-0 socket, and the env var is not a GLFW API.
  // The init HINT is the only reliable switch (GLFW >= 3.4).
  glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
#endif

  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg)
  {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version())
  {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // Load simulation configuration
  std::filesystem::path proj_dir = std::filesystem::path(getExecutableDir()).parent_path();
  param::config.load_from_yaml(proj_dir / "config.yaml");
  param::helper(argc, argv);
  if (const char* g = std::getenv("ARCHB_GRASP"); g && g[0] == '1') grasp_sim::enabled() = true;
  if (const char* nb = std::getenv("ARCHB_NO_BAND"); nb && nb[0] == '1') {
    param::config.enable_elastic_band = 0;   // welded-pelvis scenes: qpos[0..2] is NOT the base
    std::cout << "[SIM] elastic band disabled (ARCHB_NO_BAND=1)" << std::endl;
  }

  // apply the elastic-band (suspension harness) config to the global band
  if (param::config.band_anchor.size() == 3)
    elastic_band.point_ = param::config.band_anchor;
  elastic_band.length_    = param::config.band_rest_length;
  elastic_band.stiffness_ = param::config.band_stiffness;
  elastic_band.damping_   = param::config.band_damping;

  if(param::config.robot_scene.is_relative()) {
    param::config.robot_scene = proj_dir.parent_path() / "unitree_robots" / param::config.robot / param::config.robot_scene;
  }

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
    std::make_unique<mj::GlfwAdapter>(),
    &cam, &opt, &pert, /* is_passive = */ false);

  std::thread unitree_thread(UnitreeSdk2BridgeThread, nullptr);

  // harness stdin command thread (push <vx> <vy>)
  std::thread stdin_thread(StdinCommandThread);
  stdin_thread.detach();

  // harness: auto-release the elastic band the moment the controller signals
  // policy engagement (it creates ARCHB_BAND_RELEASE_FILE after FixStand). So
  // the band supports spawn + the FixStand ramp, then the robot free-stands
  // under the policy — no manual key press, no host/container coupling beyond
  // a flag file on the shared mount.
  if (const char *band_flag = std::getenv("ARCHB_BAND_RELEASE_FILE"))
  {
    if (band_flag[0])
    {
      std::thread band_thread([flag = std::string(band_flag)]() {
        struct stat sb;
        while (stat(flag.c_str(), &sb) != 0)
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        elastic_band.enable_ = false;
        anchor_pub::engage_request() = true;   // plant the training anchor at the engage pose
        std::printf("[BAND] policy-engaged flag seen → elastic band released\n");
        std::fflush(stdout);
      });
      band_thread.detach();
    }
  }

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), param::config.robot_scene.c_str());
  // Feature D: route the Arm Cmd sliders' publishes through the same arm-pose
  // publisher the keyboard presets use (no-ops until the bridge thread inits it).
  arm_gui::publish_fn() = publish_arm_pose;
  arm_gui::lean_publish_fn() = publish_lean;    // Lean slider -> rt/lean_cmd
  // start simulation UI loop (blocking call)
  glfwSetKeyCallback(static_cast<mj::GlfwAdapter*>(sim->platform_ui.get())->window_,user_key_cb);
  sim->RenderLoop();
  physicsthreadhandle.join();

  pthread_exit(NULL);
  return 0;
}
