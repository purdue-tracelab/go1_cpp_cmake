#ifndef GO1_STATE_H
#define GO1_STATE_H

// Open-source libraries
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>
#include "OsqpEigen/OsqpEigen.h"
#include <mujoco/mujoco.h>

// Package-specific header files
#include "go1Params.h"
#include "go1FK.h"
#include "go1Utils.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"


class go1State {
    public:
        // functions
        go1State();
        void resetState();
        void updateStateFromMujoco(const mjtNum* q_vec, const mjtNum* q_vel, const Eigen::Vector3d& lin_acc);
        void updateHardwareState(UNITREE_LEGGED_SDK::LowState& state);
        void convertForcesToTorquesMujoco(const mjtNum* q_vec);
        void convertForcesToTorquesHardware(const Eigen::VectorXd& jointPos);
        void raibertHeuristic(bool withCapturePoint = false);
        void amirHLIP();
        void swingPD(int leg_idx, Eigen::Vector3d footPosRef, Eigen::Vector3d footVelRef, bool absolute = false);
        Eigen::Vector3d bezierPos();
        Eigen::Vector3d bezierVel();

        // default information
        Eigen::Matrix3d go1_lumped_inertia;

        // state variables
        Eigen::Vector3d root_pos;
        Eigen::Vector3d root_pos_old;
        Eigen::Vector3d root_pos_d;
        Eigen::Vector3d root_pos_est;
        Eigen::Vector3d root_lin_vel;
        Eigen::Vector3d root_lin_vel_old;      
        Eigen::Vector3d root_lin_vel_d;
        Eigen::Vector3d root_lin_vel_est;
        Eigen::Vector3d root_lin_acc;
        Eigen::Vector3d root_lin_acc_est;

        Eigen::Quaterniond root_quat;
        Eigen::Quaterniond root_quat_old;
        Eigen::Quaterniond root_quat_est;
        Eigen::Vector3d root_rpy;
        Eigen::Vector3d root_rpy_old;
        Eigen::Vector3d root_rpy_d;
        Eigen::Vector3d root_rpy_est;
        Eigen::Vector3d root_ang_vel;
        Eigen::Vector3d root_ang_vel_old;      
        Eigen::Vector3d root_ang_vel_d;
        Eigen::Vector3d root_ang_vel_est;

        Eigen::Matrix<double, 1, 12> jointPos;      // added

        // actuation-related variables (ORDER IS FR, FL, RR, RL)
        Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
        Eigen::Matrix<double, 3, NUM_LEG> foot_forces_swing;
        Eigen::Matrix<double, 3, NUM_LEG> joint_torques;

        // gait phase
        bool walking_mode;
        bool contacts[NUM_LEG];
        int swing_phase;
        bool init;

        // swing leg PD control-related variables (ORDER IS FR, FL, RR, RL)
        double foot_deltaX;
        double foot_deltaY;
        Eigen::Matrix<double, 3, NUM_LEG> default_foot_pos; // relative frame
        Eigen::Matrix<double, 3, NUM_LEG> root_hip_pos; // relative frame
        Eigen::Matrix<double, 3, NUM_LEG> root_hip_pos_abs;
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos; // relative frame
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_old; // relative frame
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs;
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs_est;
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_d;
        Eigen::Matrix<double, 3, NUM_LEG> foot_vel_d;
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_liftoff; // absolute frame

        // Amir's HLIP
        Eigen::Vector2d stanceFeetSumXY;
        Eigen::VectorXd x_footprint_default, y_footprint_default;
        double x_footprint_offset, y_footprint_offset;
        Eigen::Vector2d StanceFeetSumXY;
        Eigen::Vector2d StanceFeetMP, StanceFeetMP_prev;
        Eigen::Vector2d rel_error_pos;
        Eigen::Vector2d rel_error_vel;
        Eigen::Vector2d u_ref;
        Eigen::Matrix2d Phi_rem; // state transisiton matrix for the nominal remaining time in swing phase
        Eigen::Vector2d X_hlip, Y_hlip, X_hlip_minus, Y_hlip_minus, X_hlip_ref_minus, Y_hlip_ref_minus, X_hlip_error_minus, Y_hlip_error_minus; // HLIP state variables
        Eigen::SparseMatrix<double> hlip_hessian;
        Eigen::Matrix<double, 4, 4> hlip_dense_hessian;
        Eigen::SparseMatrix<double> hlip_linearConst;
        Eigen::Matrix<double, 6, 4> hlip_dense_linearConst;
        Eigen::VectorXd hlip_gradient;
        Eigen::VectorXd hlip_ub; 
        Eigen::VectorXd hlip_lb;
        Eigen::VectorXd GainsHLIP; // HLIP-based stepping controller gains
    
};

#endif //GO1_STATE_H