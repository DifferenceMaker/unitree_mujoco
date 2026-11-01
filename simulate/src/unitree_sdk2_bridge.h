#pragma once

#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/ros2/String_.hpp>

#include <iostream>
#include <cstring>
#include <random>
#include <cmath>

#include "param.h"
#include "physics_joystick.h"
#include "walk_hud.h"
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <deque>

// Lock-step (2026-10-28): count received lowcmd messages so the headless loop can step one
// physics frame per command (post_communication runs inside the subscription's lock).
// lock-step SYNCHRONOUS lowstate publish (2026-10-29): the dds_wrapper RealTimePublisher hands msg_ to a background thread that polls
// with 1 ms sleeps; at a ~1 ms frame rate its trylock() fails under load, the frame's lowstate is never published, the Bridge setter's
// 0.5 s wait times out and ticks on the stale state -> one policy frame skipped per ~100-200 s per rig when rigs share the box (the
// cc_1..4 traces). In lock-step the loop writes msg_ under the lock and publishes it itself on the same topic, synchronously.
inline bool& lockstep_sync_publish() { static bool v = false; return v; }

// lock-step wake-up: the headless loop sleeps on this condition variable and is woken by the DDS receive thread the moment the
// next lowcmd lands (2026-10-28 evening: replaces a 20 us sleep-poll whose real granularity was ~70 us per frame)
inline std::mutex& lockstep_cmd_mutex() { static std::mutex m; return m; }
inline std::condition_variable& lockstep_cmd_cv() { static std::condition_variable cv; return cv; }

// lock-step COMPACT transport (2026-11-01, exam speed; SIM_LOCKSTEP_COMPACT=1 with BRIDGE_LOCKSTEP_COMPACT=1): every frame publishes
// rt/lockstep/state (hex of uint32 tick + float32 q[num_motor]) and the full rt/lowstate only on POLICY frames (tick %
// SIM_LOCKSTEP_POLICY_MS == 0: the only frames the Bridge getter forwards to MovementModule); the Bridge setter answers on
// rt/lockstep/cmd (hex of uint32 tick + float32 q[n] + float32 tau[n]), written onto the last full rt/lowcmd (kp / kd / dq / mode
// stay). The setter's Python DDS work per 2 ms frame was LowState deserialize 348 us + CRC 86 us + LowCmd serialize 184 us = most
// of the 0.81 ms the sim waited per frame (10 rigs, rtf 1.70); the hex String_ costs ~6 us each way. The same float32 values travel
// on both paths, so the physics is unchanged.
inline bool& lockstep_compact() { static bool v = false; return v; }
inline int& lockstep_policy_ms() { static int v = 20; return v; }

template <class Base>
class CountedSub : public Base
{
public:
    using Base::Base;
    std::atomic<uint64_t> seq{0};
protected:
    void post_communication() override
    {
        { const std::lock_guard<std::mutex> lk(lockstep_cmd_mutex()); seq.fetch_add(1, std::memory_order_release); }
        lockstep_cmd_cv().notify_all();
    }
};

#define MOTOR_SENSOR_NUM 3

class UnitreeSDK2BridgeBase
{
public:
    UnitreeSDK2BridgeBase(mjModel *model, mjData *data)
    : mj_model_(model), mj_data_(data)
    {
        _check_sensor();
        if(param::config.print_scene_information == 1) {
            printSceneInformation();
        }
        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

    }

    virtual void start() {}
    // Lock-step hooks (2026-10-28): the headless loop drives the bridge instead of the 1 kHz RecurrentThread.
    virtual uint64_t lowcmd_seq() { return 0; }
    virtual void apply_lowcmd(int delay_frames) {}   // PD from the lowcmd received delay_frames frames ago -> ctrl
    virtual void publish_state() {}                   // one lowstate (+ highstate / IMU / bms) from the current mj_data
    int num_motor() const { return num_motor_; }
    std::vector<double> q_target_;                    // exam #2 (2026-10-30): the joint targets of the lowcmd last applied ("qt" in rt/sim_base_pose)

    void printSceneInformation()
    {
        auto printObjects = [this](const char* title, int count, int type, auto getIndex) {
            std::cout << "<<------------- " << title << " ------------->> " << std::endl;
            for (int i = 0; i < count; i++) {
                const char* name = mj_id2name(mj_model_, type, i);
                if (name) {
                    std::cout << title << "_index: " << getIndex(i) << ", " << "name: " << name;
                    if (type == mjOBJ_SENSOR) {
                        std::cout << ", dim: " << mj_model_->sensor_dim[i];
                    }
                    std::cout << std::endl;
                }
            }
            std::cout << std::endl;
        };
    
        printObjects("Link", mj_model_->nbody, mjOBJ_BODY, [](int i) { return i; });
        printObjects("Joint", mj_model_->njnt, mjOBJ_JOINT, [](int i) { return i; });
        printObjects("Actuator", mj_model_->nu, mjOBJ_ACTUATOR, [](int i) { return i; });
    
        int sensorIndex = 0;
        printObjects("Sensor", mj_model_->nsensor, mjOBJ_SENSOR, [&](int i) {
            int currentIndex = sensorIndex;
            sensorIndex += mj_model_->sensor_dim[i];
            return currentIndex;
        });
    }

protected:
    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    mjData *mj_data_;
    mjModel *mj_model_;

    // Sensor data indices
    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_acc_adr_ = -1;

    // --- OU IMU-noise state (param::config.imu_noise_std_deg / _tau_ms) ---
    // A body-frame orientation-estimate error evolves as an OU process in sim
    // time; the published quat is composed with it, and gyro/acc are rotated
    // into the same perturbed frame (structural coupling, like a real IMU).
    double ou_rpy_[3] = {0.0, 0.0, 0.0};
    double ou_last_t_ = -1.0;
    double err_quat_[4] = {1.0, 0.0, 0.0, 0.0};
    std::mt19937 ou_rng_{12345};
    std::normal_distribution<double> ou_n_{0.0, 1.0};

    void rotate_into_observed_frame(double v[3]) const
    {
        // rotate v (true body frame) by the inverse of err_quat_
        const double w = err_quat_[0], x = -err_quat_[1], y = -err_quat_[2], z = -err_quat_[3];
        const double uvx = 2 * (y * v[2] - z * v[1]);
        const double uvy = 2 * (z * v[0] - x * v[2]);
        const double uvz = 2 * (x * v[1] - y * v[0]);
        const double o0 = v[0] + w * uvx + (y * uvz - z * uvy);
        const double o1 = v[1] + w * uvy + (z * uvx - x * uvz);
        const double o2 = v[2] + w * uvz + (x * uvy - y * uvx);
        v[0] = o0; v[1] = o1; v[2] = o2;
    }
    int frame_pos_adr_ = -1;
    int frame_vel_adr_ = -1;

    int secondary_imu_quat_adr_ = -1;
    int secondary_imu_gyro_adr_ = -1;
    int secondary_imu_acc_adr_ = -1;

    std::shared_ptr<unitree::common::UnitreeJoystick> joystick = nullptr;

    void _check_sensor()
    {
        // BODY motors only (2026-08-28, grasp rig): actuators whose name carries a
        // namespace prefix ("rh:drv_index" — the attached Inspire hand's finger
        // drivers) are NOT lowstate/lowcmd motors: the real hand is a separate
        // Modbus device, and the sim's grasp_sim.h drives those actuators itself.
        // They are appended AFTER the 27 body actuators, so the positional
        // motor<->sensor map below (sensordata[i], [i+n], [i+2n]) stays intact.
        num_motor_ = 0;
        for (int a = 0; a < mj_model_->nu; ++a) {
            const char* an = mj_id2name(mj_model_, mjOBJ_ACTUATOR, a);
            if (an && std::strchr(an, ':')) break;
            ++num_motor_;
        }
        if (num_motor_ != mj_model_->nu)
            std::cout << "[bridge] " << num_motor_ << " body motors on lowstate/lowcmd, "
                      << (mj_model_->nu - num_motor_) << " namespaced actuators left to the sim" << std::endl;
        dim_motor_sensor_ = MOTOR_SENSOR_NUM * num_motor_;
    
        // Find sensor addresses by name
        int sensor_id = -1;
        
        // IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_quat");
        if (sensor_id >= 0) {
            imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_gyro");
        if (sensor_id >= 0) {
            imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_acc");
        if (sensor_id >= 0) {
            imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame position
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_pos");
        if (sensor_id >= 0) {
            frame_pos_adr_ = mj_model_->sensor_adr[sensor_id];
        }
        
        // Frame velocity
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "frame_vel");
        if (sensor_id >= 0) {
            frame_vel_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU quaternion
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_quat");
        if (sensor_id >= 0) {
            secondary_imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU gyroscope
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_gyro");
        if (sensor_id >= 0) {
            secondary_imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];
        }

        // Secondary IMU accelerometer
        sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "secondary_imu_acc");
        if (sensor_id >= 0) {
            secondary_imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];
        }
    }
};

template <typename LowCmd_t, typename LowState_t>
class RobotBridge : public UnitreeSDK2BridgeBase
{
using HighState_t = unitree::robot::go2::publisher::SportModeState;
using WirelessController_t = unitree::robot::go2::publisher::WirelessController;

public:
    RobotBridge(mjModel *model, mjData *data) : UnitreeSDK2BridgeBase(model, data)
    {
        lowcmd = std::make_shared<LowCmd_t>("rt/lowcmd");
        lowstate = std::make_unique<LowState_t>();
        lowstate->joystick = joystick;
        highstate = std::make_unique<HighState_t>();
        wireless_controller = std::make_unique<WirelessController_t>();
        wireless_controller->joystick = joystick;
    }

    void start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "unitree_bridge", UT_CPU_ID_NONE, 1000, [this]() { this->run(); });
    }

    uint64_t lowcmd_seq() override { return lowcmd->seq.load(std::memory_order_relaxed); }

    void pd_from(const typename LowCmd_t::MsgType& msg)
    {
        if ((int)q_target_.size() != num_motor_) q_target_.assign(num_motor_, 0.0);
        for(int i(0); i<num_motor_; i++) {
            auto & m = msg.motor_cmd()[i];
            q_target_[i] = m.q();
            mj_data_->ctrl[i] = m.tau() +
                                m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
        }
    }

    // PD from the lowcmd on the wire. delay_frames > 0 (lock-step) = a wire delay: the command snapshotted
    // delay_frames calls ago is applied (the robot's obs-age + action-path latency, ~4 ms = 2 frames at 500 Hz).
    void apply_lowcmd(int delay_frames) override
    {
        if(!mj_data_) return;
        if (delay_frames <= 0) {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            pd_from(lowcmd->msg_);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            delay_q_.push_back(lowcmd->msg_);
        }
        while ((int)delay_q_.size() > delay_frames + 1) delay_q_.pop_front();
        if ((int)delay_q_.size() == delay_frames + 1) pd_from(delay_q_.front());
    }

    virtual void run()
    {
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }
        apply_lowcmd(0);
        publish_state();
    }

    std::shared_ptr<unitree::robot::ChannelPublisher<typename LowState_t::MsgType>> lowstate_direct_;   // lock-step: synchronous rt/lowstate

    // lock-step COMPACT transport (see lockstep_compact())
    std::shared_ptr<unitree::robot::ChannelPublisher<std_msgs::msg::dds_::String_>> compact_state_pub_;
    std::shared_ptr<unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_>> compact_cmd_sub_;
    std::atomic<uint64_t> compact_bad_{0};

    static void hex_append(std::string& out, const void* p, size_t n)
    {
        static const char* d = "0123456789abcdef";
        const uint8_t* b = static_cast<const uint8_t*>(p);
        for (size_t i = 0; i < n; i++) { out.push_back(d[b[i] >> 4]); out.push_back(d[b[i] & 15]); }
    }

    void on_compact_cmd(const std_msgs::msg::dds_::String_& s)
    {
        const std::string& h = s.data();
        auto nib = [](char c) -> int { return c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1; };
        const size_t nbytes = h.size() / 2;
        if (h.size() % 2 || nbytes < 4 || (nbytes - 4) % 8) { ++compact_bad_; return; }
        std::vector<uint8_t> b(nbytes);
        for (size_t i = 0; i < nbytes; i++) {
            const int hi = nib(h[2 * i]), lo = nib(h[2 * i + 1]);
            if (hi < 0 || lo < 0) { ++compact_bad_; return; }
            b[i] = static_cast<uint8_t>((hi << 4) | lo);
        }
        const int m = static_cast<int>((nbytes - 4) / 8);   // motors on the wire: q[m] then tau[m]
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            for (int i = 0; i < m && i < num_motor_; i++) {
                float q, tau;
                std::memcpy(&q, &b[4 + 4 * i], 4);
                std::memcpy(&tau, &b[4 + 4 * m + 4 * i], 4);
                lowcmd->msg_.motor_cmd()[i].q() = q;
                lowcmd->msg_.motor_cmd()[i].tau() = tau;
            }
        }
        { const std::lock_guard<std::mutex> lk(lockstep_cmd_mutex()); lowcmd->seq.fetch_add(1, std::memory_order_release); }
        lockstep_cmd_cv().notify_all();
    }

    void publish_state() override
    {
        if(!mj_data_) return;
        // lowstate
        const bool sync = lockstep_sync_publish();
        if (sync && !lowstate_direct_) {
            lowstate_direct_ = std::make_shared<unitree::robot::ChannelPublisher<typename LowState_t::MsgType>>("rt/lowstate");
            lowstate_direct_->InitChannel();
        }
        const bool compact = sync && lockstep_compact();
        if (compact && !compact_state_pub_) {
            compact_state_pub_ = std::make_shared<unitree::robot::ChannelPublisher<std_msgs::msg::dds_::String_>>("rt/lockstep/state");
            compact_state_pub_->InitChannel();
            compact_cmd_sub_ = std::make_shared<unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_>>("rt/lockstep/cmd");
            compact_cmd_sub_->InitChannel([this](const void* p) { this->on_compact_cmd(*static_cast<const std_msgs::msg::dds_::String_*>(p)); }, 1);
            std::printf("[LOCKSTEP] compact transport: rt/lockstep/state every frame, rt/lowstate on policy frames (every %d ms), "
                        "commands on rt/lockstep/cmd or rt/lowcmd\n", lockstep_policy_ms());
        }
        std::string compact_hex;
        if (sync) lowstate->lock();
        if(sync || lowstate->trylock()) {
            // walk HUD: actual yaw-frame base velocity (cmd-vs-actual bars).
            walk_hud::update_actual(mj_model_, mj_data_);
            // stamp tick with sim time (ms) — the real robot's tick advances,
            // and BridgeModule derives its obs timestamp from it. Left at 0 the
            // stamp freezes at startup and every displayed obs_age grows
            // monotonically with wall time (the 'obscene obs_age' artifact,
            // 2026-07-26). uint32 ms wraps at ~49.7 days of sim time — fine.
            lowstate->msg_.tick() = static_cast<uint32_t>(mj_data_->time * 1000.0);
            for(int i(0); i<num_motor_; i++) {
                lowstate->msg_.motor_state()[i].q() = mj_data_->sensordata[i];
                lowstate->msg_.motor_state()[i].dq() = mj_data_->sensordata[i + num_motor_];
                lowstate->msg_.motor_state()[i].tau_est() = mj_data_->sensordata[i + 2 * num_motor_];
            }
            
            if(imu_quat_adr_ >= 0) {
                double w = mj_data_->sensordata[imu_quat_adr_ + 0];
                double x = mj_data_->sensordata[imu_quat_adr_ + 1];
                double y = mj_data_->sensordata[imu_quat_adr_ + 2];
                double z = mj_data_->sensordata[imu_quat_adr_ + 3];

                if (param::config.imu_noise_std_deg > 0.0) {
                    const double tau = param::config.imu_noise_tau_ms * 1e-3;
                    const double std_rad = param::config.imu_noise_std_deg * M_PI / 180.0;
                    const double dt = (ou_last_t_ < 0.0) ? 0.0 : mj_data_->time - ou_last_t_;
                    ou_last_t_ = mj_data_->time;
                    if (dt > 0.0 && dt < 0.1) {
                        const double sw = std_rad * std::sqrt(2.0 / tau);
                        for (int k = 0; k < 3; k++)
                            ou_rpy_[k] += -(ou_rpy_[k] / tau) * dt + sw * std::sqrt(dt) * ou_n_(ou_rng_);
                    }
                    const double cr = cos(ou_rpy_[0] / 2), sr = sin(ou_rpy_[0] / 2);
                    const double cp = cos(ou_rpy_[1] / 2), sp = sin(ou_rpy_[1] / 2);
                    const double cy = cos(ou_rpy_[2] / 2), sy = sin(ou_rpy_[2] / 2);
                    err_quat_[0] = cr * cp * cy + sr * sp * sy;
                    err_quat_[1] = sr * cp * cy - cr * sp * sy;
                    err_quat_[2] = cr * sp * cy + sr * cp * sy;
                    err_quat_[3] = cr * cp * sy - sr * sp * cy;
                    // q_obs = q_true (x) q_err : error lives in the body frame
                    const double ow = w * err_quat_[0] - x * err_quat_[1] - y * err_quat_[2] - z * err_quat_[3];
                    const double ox = w * err_quat_[1] + x * err_quat_[0] + y * err_quat_[3] - z * err_quat_[2];
                    const double oy = w * err_quat_[2] - x * err_quat_[3] + y * err_quat_[0] + z * err_quat_[1];
                    const double oz = w * err_quat_[3] + x * err_quat_[2] - y * err_quat_[1] + z * err_quat_[0];
                    w = ow; x = ox; y = oy; z = oz;
                }

                lowstate->msg_.imu_state().quaternion()[0] = w;
                lowstate->msg_.imu_state().quaternion()[1] = x;
                lowstate->msg_.imu_state().quaternion()[2] = y;
                lowstate->msg_.imu_state().quaternion()[3] = z;

                lowstate->msg_.imu_state().rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                lowstate->msg_.imu_state().rpy()[1] = asin(2 * (w * y - z * x));
                lowstate->msg_.imu_state().rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }
            
            if(imu_gyro_adr_ >= 0) {
                double g[3] = {mj_data_->sensordata[imu_gyro_adr_ + 0],
                               mj_data_->sensordata[imu_gyro_adr_ + 1],
                               mj_data_->sensordata[imu_gyro_adr_ + 2]};
                if (param::config.imu_noise_std_deg > 0.0) rotate_into_observed_frame(g);
                lowstate->msg_.imu_state().gyroscope()[0] = g[0];
                lowstate->msg_.imu_state().gyroscope()[1] = g[1];
                lowstate->msg_.imu_state().gyroscope()[2] = g[2];
            }

            if(imu_acc_adr_ >= 0) {
                double a[3] = {mj_data_->sensordata[imu_acc_adr_ + 0],
                               mj_data_->sensordata[imu_acc_adr_ + 1],
                               mj_data_->sensordata[imu_acc_adr_ + 2]};
                if (param::config.imu_noise_std_deg > 0.0) rotate_into_observed_frame(a);
                lowstate->msg_.imu_state().accelerometer()[0] = a[0];
                lowstate->msg_.imu_state().accelerometer()[1] = a[1];
                lowstate->msg_.imu_state().accelerometer()[2] = a[2];
            }
            
            lowstate->msg_.tick() = std::round(mj_data_->time / 1e-3);
            if (compact) {
                const uint32_t tick = lowstate->msg_.tick();
                compact_hex.reserve(2 * (4 + 4 * num_motor_));
                hex_append(compact_hex, &tick, 4);
                for (int i = 0; i < num_motor_; i++) { const float q = lowstate->msg_.motor_state()[i].q(); hex_append(compact_hex, &q, 4); }
            }
            const bool full = !compact || (lockstep_policy_ms() > 0 && lowstate->msg_.tick() % lockstep_policy_ms() == 0);
            if (sync) { if (full) lowstate_direct_->Write(lowstate->msg_, 0); lowstate->unlock(); }   // published NOW, never skipped
            else lowstate->unlockAndPublish();
        }
        if (compact && !compact_hex.empty()) {   // after the full lowstate: the getter's chain to MovementModule starts first
            std_msgs::msg::dds_::String_ s;
            s.data() = std::move(compact_hex);
            compact_state_pub_->Write(s, 0);
        }
        // highstate
        if(highstate->trylock()) {
            if(frame_pos_adr_ >= 0) {
                highstate->msg_.position()[0] = mj_data_->sensordata[frame_pos_adr_ + 0];
                highstate->msg_.position()[1] = mj_data_->sensordata[frame_pos_adr_ + 1];
                highstate->msg_.position()[2] = mj_data_->sensordata[frame_pos_adr_ + 2];
            }
            if(frame_vel_adr_ >= 0) {
                highstate->msg_.velocity()[0] = mj_data_->sensordata[frame_vel_adr_ + 0];
                highstate->msg_.velocity()[1] = mj_data_->sensordata[frame_vel_adr_ + 1];
                highstate->msg_.velocity()[2] = mj_data_->sensordata[frame_vel_adr_ + 2];
            }
            highstate->unlockAndPublish();
        }
        // wireless_controller
        if(wireless_controller->joystick) {
            wireless_controller->unlockAndPublish();
        }
    }

    std::unique_ptr<HighState_t> highstate;
    std::unique_ptr<WirelessController_t> wireless_controller;
    std::shared_ptr<LowCmd_t> lowcmd;
    std::deque<typename LowCmd_t::MsgType> delay_q_;   // lock-step wire delay (apply_lowcmd)
    std::unique_ptr<LowState_t> lowstate;
    
private:
    unitree::common::RecurrentThreadPtr thread_;
};

using Go2Bridge = RobotBridge<CountedSub<unitree::robot::go2::subscription::LowCmd>, unitree::robot::go2::publisher::LowState>;

class G1Bridge : public RobotBridge<CountedSub<unitree::robot::g1::subscription::LowCmd>, unitree::robot::g1::publisher::LowState>
{
public:
    G1Bridge(mjModel *model, mjData *data) : RobotBridge(model, data)
    {
        if (param::config.robot.find("g1") != std::string::npos) {
            auto* g1_lowstate = dynamic_cast<unitree::robot::g1::publisher::LowState*>(lowstate.get());
            if (g1_lowstate) {
                auto scene = param::config.robot_scene.filename().string();
                g1_lowstate->msg_.mode_machine() = scene.find("23") != std::string::npos ? 4 : 5;
            }
        }

        bmsstate = std::make_unique<BmsState_t>("rt/lf/bmsstate");
        bmsstate->msg_.soc() = 100;

        secondary_imustate = std::make_unique<IMUState_t>("rt/secondary_imu");
    }

    void publish_state() override
    {
        RobotBridge::publish_state();

        // secondary IMU state
        if (secondary_imustate->trylock()) {
            if(secondary_imu_quat_adr_ >= 0) {
                secondary_imustate->msg_.quaternion()[0] = mj_data_->sensordata[secondary_imu_quat_adr_ + 0];
                secondary_imustate->msg_.quaternion()[1] = mj_data_->sensordata[secondary_imu_quat_adr_ + 1];
                secondary_imustate->msg_.quaternion()[2] = mj_data_->sensordata[secondary_imu_quat_adr_ + 2];
                secondary_imustate->msg_.quaternion()[3] = mj_data_->sensordata[secondary_imu_quat_adr_ + 3];

                double w = secondary_imustate->msg_.quaternion()[0];
                double x = secondary_imustate->msg_.quaternion()[1];
                double y = secondary_imustate->msg_.quaternion()[2];
                double z = secondary_imustate->msg_.quaternion()[3];

                secondary_imustate->msg_.rpy()[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
                secondary_imustate->msg_.rpy()[1] = asin(2 * (w * y - z * x));
                secondary_imustate->msg_.rpy()[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            }

            if(secondary_imu_gyro_adr_ >= 0) {
                secondary_imustate->msg_.gyroscope()[0] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 0];
                secondary_imustate->msg_.gyroscope()[1] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 1];
                secondary_imustate->msg_.gyroscope()[2] = mj_data_->sensordata[secondary_imu_gyro_adr_ + 2];
            }

            if(secondary_imu_acc_adr_ >= 0) {
                secondary_imustate->msg_.accelerometer()[0] = mj_data_->sensordata[secondary_imu_acc_adr_ + 0];
                secondary_imustate->msg_.accelerometer()[1] = mj_data_->sensordata[secondary_imu_acc_adr_ + 1];
                secondary_imustate->msg_.accelerometer()[2] = mj_data_->sensordata[secondary_imu_acc_adr_ + 2];
            }

            secondary_imustate->unlockAndPublish();
        }

        // In practice, bmsstate is sent at a low frequency; here it is sent with the main loop
        bmsstate->unlockAndPublish();
    }

    using BmsState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::BmsState_>;
    using IMUState_t = unitree::robot::RealTimePublisher<unitree_hg::msg::dds_::IMUState_>;
    std::unique_ptr<BmsState_t> bmsstate;
    std::unique_ptr<IMUState_t> secondary_imustate;
};
