#pragma once

#include <mujoco/mujoco.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/idl/hg/BmsState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

#include <iostream>
#include <cstring>
#include <random>
#include <cmath>

#include "param.h"
#include "physics_joystick.h"
#include "walk_hud.h"

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

    virtual void run()
    {
        if(!mj_data_) return;
        if(lowstate->joystick) { lowstate->joystick->update(); }
        // lowcmd
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            for(int i(0); i<num_motor_; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            }
        }

        // lowstate
        if(lowstate->trylock()) {
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
            lowstate->unlockAndPublish();
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
    std::unique_ptr<LowState_t> lowstate;
    
private:
    unitree::common::RecurrentThreadPtr thread_;
};

using Go2Bridge = RobotBridge<unitree::robot::go2::subscription::LowCmd, unitree::robot::go2::publisher::LowState>;

class G1Bridge : public RobotBridge<unitree::robot::g1::subscription::LowCmd, unitree::robot::g1::publisher::LowState>
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

    void run() override
    {
        RobotBridge::run();

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
