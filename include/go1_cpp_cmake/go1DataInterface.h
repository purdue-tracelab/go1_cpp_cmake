#ifndef GO1_DATA_INTERFACE_H
#define GO1_DATA_INTERFACE_H

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

            // Copy current values into desired and old states to avoid jumping
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

struct lowLevelCommandSender {
    virtual ~lowLevelCommandSender() = default;
    virtual void sendCommand(const go1State &state) = 0;
};

struct mujocoCommandSender : lowLevelCommandSender {
    public:
        mujocoCommandSender(mjModel* model, mjData* data) : mujoco_model(model), mujoco_data(data) {}

        void sendCommand(const go1State &state) override {
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

#endif // GO1_DATA_INTERFACE_H