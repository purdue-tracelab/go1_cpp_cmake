#ifndef GO1_DATA_INTERFACE_H
#define GO1_DATA_INTERFACE_H

#include <chrono>
#include "go1State.h"

struct lowLevelDataReader {
    virtual ~lowLevelDataReader() = default;
    virtual void pullSensorData(go1State &state) = 0;
};

struct mujocoDataReader : lowLevelDataReader {
    public:
        mujocoDataReader(mjModel* model, mjData* data, 
                        const std::string &base_body_name,
                        const std::string &imu_acc_name,
                        const std::string &imu_gyro_name,
                        const std::string &contact_FR_name,
                        const std::string &contact_FL_name,
                        const std::string &contact_RR_name,
                        const std::string &contact_RL_name)
                        : mujoco_model(model), mujoco_data(data), 
                        base_id(mj_name2id(mujoco_model, mjOBJ_BODY, base_body_name.c_str())), 
                        imu_acc_adr(mujoco_model->sensor_adr[mj_name2id(mujoco_model, mjOBJ_SENSOR, imu_acc_name.c_str())]), 
                        imu_gyro_adr(mujoco_model->sensor_adr[mj_name2id(mujoco_model, mjOBJ_SENSOR, imu_gyro_name.c_str())]),
                        contact_FR_adr(mujoco_model->sensor_adr[mj_name2id(mujoco_model, mjOBJ_SENSOR, contact_FR_name.c_str())]),
                        contact_FL_adr(mujoco_model->sensor_adr[mj_name2id(mujoco_model, mjOBJ_SENSOR, contact_FL_name.c_str())]),
                        contact_RR_adr(mujoco_model->sensor_adr[mj_name2id(mujoco_model, mjOBJ_SENSOR, contact_RR_name.c_str())]),
                        contact_RL_adr(mujoco_model->sensor_adr[mj_name2id(mujoco_model, mjOBJ_SENSOR, contact_RL_name.c_str())])
        {
                if (base_id < 0 || imu_acc_adr < 0 || imu_gyro_adr < 0 
                    || contact_FR_adr < 0 || contact_FL_adr < 0 
                    || contact_RR_adr < 0 || contact_RL_adr < 0 ) {
                    std::cerr << "Error: Invalid sensor or body name!" << std::endl;
                    return;
                }
        }

        void pullSensorData(go1State &state) override {
            if (!mujoco_data) {
                std::cerr << "Error: data is NULL!" << std::endl;
                return;
            }

            // pull sensor data from MuJoCo
            state.root_lin_acc_meas = Eigen::Map<const Eigen::Vector3d>(mujoco_data->sensordata + imu_acc_adr);
            state.root_ang_vel_meas = Eigen::Map<const Eigen::Vector3d>(mujoco_data->sensordata + imu_gyro_adr);
            state.joint_pos = Eigen::Map<const Eigen::Matrix<double, 12, 1>>(mujoco_data->qpos + 7);
            state.joint_vel = Eigen::Map<const Eigen::Matrix<double, 12, 1>>(mujoco_data->qvel + 6);
            state.est_contacts << mujoco_data->sensordata[contact_FR_adr],
                                    mujoco_data->sensordata[contact_FL_adr],
                                    mujoco_data->sensordata[contact_RR_adr],
                                    mujoco_data->sensordata[contact_RL_adr];
        
            // collect ground truth information from MuJoCo
            state.root_pos = Eigen::Map<const Eigen::Vector3d>(mujoco_data->qpos + 0);
            state.root_lin_vel = Eigen::Map<const Eigen::Vector3d>(mujoco_data->qvel + 0);
            state.root_lin_acc = Eigen::Map<const Eigen::Vector3d>(mujoco_data->cacc + 3*base_id);
            state.root_quat.w() = mujoco_data->qpos[3];
            state.root_quat.x() = mujoco_data->qpos[4];
            state.root_quat.y() = mujoco_data->qpos[5];
            state.root_quat.z() = mujoco_data->qpos[6];
            state.root_quat.normalize();
            state.root_rpy = quat2Euler(state.root_quat);
            state.root_ang_vel = Eigen::Map<const Eigen::Vector3d>(mujoco_data->qvel + 3);

            // Copy ground truth initial position into estimated for no jumps in experiment 
            if (state.init) {
                state.root_pos_est = state.root_pos;
            }

            // calculate foot position & Jacobian before state estimation
            const auto& root_rpy_ctrl = USE_EST_FOR_CONTROL ? state.root_rpy_est : state.root_rpy;
            const auto& root_pos_ctrl = USE_EST_FOR_CONTROL ? state.root_pos_est : state.root_pos;
            Eigen::Matrix3d rootRotMat = rotZ(root_rpy_ctrl(2))*rotY(root_rpy_ctrl(1))*rotX(root_rpy_ctrl(0));

            state.foot_pos_world_rot = go1FwdKin(state.joint_pos, root_rpy_ctrl);
            state.foot_pos_abs = state.foot_pos_world_rot.colwise() + root_pos_ctrl;
            state.foot_pos_old = state.foot_pos;
            state.foot_pos = rootRotMat.transpose() * state.foot_pos_world_rot;
            state.contactJacobian = go1ContactJacobian(state.joint_pos, root_rpy_ctrl);

            // copy current values into desired and old states to avoid jumping
            if (state.init) {
                state.foot_pos_old = state.foot_pos;
                state.foot_pos_d = state.foot_pos;
                state.joint_pos_init = state.joint_pos;
                state.init = false;
            }
        }

    private:
        mjModel* mujoco_model;
        mjData* mujoco_data;
        int base_id;
        int imu_acc_adr;
        int imu_gyro_adr;
        int contact_FR_adr;
        int contact_FL_adr;
        int contact_RR_adr;
        int contact_RL_adr;
};

struct hardwareDataReader : lowLevelDataReader {
    public:
        hardwareDataReader(UNITREE_LEGGED_SDK::LowState &lowState, 
                                    UNITREE_LEGGED_SDK::UDP &udp) : extUDP(udp),
                                                                    extLowState(lowState) {}

        void pullSensorData(go1State &state) override {
            // pull sensor data from UNITREE_LEGGED_SDK::LowState
            state.root_lin_acc_meas << extLowState.imu.accelerometer[0], extLowState.imu.accelerometer[1], extLowState.imu.accelerometer[2];
            state.root_ang_vel_meas << extLowState.imu.gyroscope[0], extLowState.imu.gyroscope[1], extLowState.imu.gyroscope[2];
            state.est_contacts << extLowState.footForce[0], extLowState.footForce[1], extLowState.footForce[2], extLowState.footForce[3];
            
            for (int i = 0; i < 3*NUM_LEG; i++) {
                state.joint_pos(i, 0) = extLowState.motorState[i].q;
                state.joint_vel(i, 0) = extLowState.motorState[i].dq;
            }

            // calculate foot position & Jacobian before state estimation
            Eigen::Matrix3d rootRotMat = rotZ(state.root_rpy_est(2))*rotY(state.root_rpy_est(1))*rotX(state.root_rpy_est(0));
            state.foot_pos_world_rot = go1FwdKin(state.joint_pos, state.root_rpy_est);
            state.foot_pos_abs = state.foot_pos_world_rot.colwise() + state.root_pos_est;
            state.foot_pos_old = state.foot_pos;
            state.foot_pos = rootRotMat.transpose() * state.foot_pos_world_rot;
            state.contactJacobian = go1ContactJacobian(state.joint_pos, state.root_rpy_est);

            // Copy current values into desired and old states to avoid jumping
            if (state.init) {
                state.foot_pos_old = state.foot_pos;
                state.foot_pos_d = state.foot_pos;
                state.joint_pos_init = state.joint_pos;
                state.init = false;
            }
        }

    private:
        UNITREE_LEGGED_SDK::LowState &extLowState;
        UNITREE_LEGGED_SDK::UDP &extUDP;
};

struct lowLevelCommandSender {
    virtual ~lowLevelCommandSender() = default;
    virtual void setCommand(const go1State &state) = 0;
};

struct mujocoCommandSender : lowLevelCommandSender {
    public:
        mujocoCommandSender(mjModel* model, mjData* data) : mujoco_model(model), mujoco_data(data) {}

        void setCommand(const go1State &state) override {
            if (!mujoco_data) {
                std::cerr << "Error: data is NULL!" << std::endl;
                return;
            }

            // send control commands to MuJoCo
            for (int i = 0; i < 3*NUM_LEG; i++) {
                mujoco_data->ctrl[i] = state.joint_torques(i%3, i/3);
            }
        }

    private:
        mjModel* mujoco_model;
        mjData* mujoco_data;
};

struct hardwareCommandSender : lowLevelCommandSender {
    public:
        hardwareCommandSender(UNITREE_LEGGED_SDK::LowState &lowState, 
                                        UNITREE_LEGGED_SDK::UDP &udp,
                                        UNITREE_LEGGED_SDK::LowCmd &lowCmd) : safe(UNITREE_LEGGED_SDK::LeggedType::Go1), 
                                                                                extUDP(udp),
                                                                                extLowState(lowState),
                                                                                extLowCmd(lowCmd) {}
        
        void setCommand(const go1State &state) override {
            for (int i = 0; i < 3*NUM_LEG; i++) {
                // extLowCmd.motorCmd[i].mode = 0x0A; // not sure if required
                extLowCmd.motorCmd[i].q = UNITREE_LEGGED_SDK::PosStopF;
                extLowCmd.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF;
                extLowCmd.motorCmd[i].Kp = 0;
                extLowCmd.motorCmd[i].Kd = 0;
                extLowCmd.motorCmd[i].tau = state.joint_torques(i % 3, i / 3);
            }
        }

    private:
        UNITREE_LEGGED_SDK::LowState &extLowState;
        UNITREE_LEGGED_SDK::UDP &extUDP;
        UNITREE_LEGGED_SDK::LowCmd &extLowCmd;
        UNITREE_LEGGED_SDK::Safety safe;
};

#endif // GO1_DATA_INTERFACE_H