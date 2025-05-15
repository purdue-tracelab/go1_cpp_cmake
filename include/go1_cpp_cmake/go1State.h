#ifndef GO1_STATE_H
#define GO1_STATE_H

// Open-source libraries
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>
#include "OsqpEigen/OsqpEigen.h"
#include <mujoco/mujoco.h>
#include <mutex>
#include <atomic>
#include <array>

// Package-specific header files
#include "go1Params.h"
#include "go1FK.h"
#include "go1Utils.h"

struct go1StateSnapshot {
/*
    A snapshot of the current go1State of the robot, used to
    send information to the go1MPC solver without blocking the
    necessary loops for footstep planning and state estimation.
*/
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // collect information from go1State for go1MPC
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_world_rot; // relative frame
    Eigen::Matrix3d go1_lumped_inertia;
    Eigen::Vector3d root_pos;
    Eigen::Vector3d root_pos_d;
    Eigen::Vector3d root_rpy;
    Eigen::Vector3d root_rpy_d;
    Eigen::Vector3d root_lin_vel;
    Eigen::Vector3d root_lin_vel_d;
    Eigen::Vector3d root_ang_vel;
    Eigen::Vector3d root_ang_vel_d;
    bool walking_mode;
    int swing_phase;
    std::array<bool, NUM_LEG> contacts;

    // collect from go1MPC for go1State.foot_forces_grf
    Eigen::Matrix<double, 3, NUM_LEG> grf_forces;
};

class go1State {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // functions
        go1State();
        void resetState();

        // top-level planning & conversion functions
        void updateLocomotionPlan();
        void convertForcesToTorques();
        void swingPD(int leg_idx, Eigen::Vector3d footPosRef, Eigen::Vector3d footVelRef);        

        // footstep planning functions
        void raibertHeuristic(bool withCapturePoint = false);
        void amirHLIP();

        // foot trajectory functions
        Eigen::Vector3d bezierPos(int leg_idx);
        Eigen::Vector3d bezierVel(int leg_idx);
        Eigen::Vector3d sinusoidalPos(int leg_idx);
        Eigen::Vector3d sinusoidalVel(int leg_idx);

        // startup/shutdown functions
        bool isStartupComplete() const;
        bool isShutdownComplete() const;
        void computeStartupPD();
        void computeShutdownPD();
        void jointPD(int joint_idx, double jointPos, double jointVel, bool startup = true);

        // snapshot getter/setter functions for go1MPC
        go1StateSnapshot getSnapshot() const;
        void retrieveGRF(const Eigen::Matrix<double, 3, NUM_LEG> &grf);

        // general getters/setters
        bool getWalkingMode() const { return walking_mode; } // feels useless
        void setWalkingMode(bool v) { walking_mode = v; } // feels useless
        int getSwingPhase() const { return swing_phase; } // feels useless

        // default information
        Eigen::Matrix3d go1_lumped_inertia;

        // state variables
        Eigen::Vector3d root_pos;
        Eigen::Vector3d root_pos_d;
        Eigen::Vector3d root_lin_vel;
        Eigen::Vector3d root_lin_vel_d;
        Eigen::Vector3d root_lin_acc;
        Eigen::Quaterniond root_quat;
        Eigen::Vector3d root_rpy;
        Eigen::Vector3d root_rpy_d;
        Eigen::Vector3d root_ang_vel;
        Eigen::Vector3d root_ang_vel_d;

        // actuation-related variables (ORDER IS FR, FL, RR, RL)
        Eigen::MatrixXd contactJacobian;
        Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
        Eigen::Matrix<double, 12, 1> foot_forces_grf_stacked;
        Eigen::Matrix<double, 3, NUM_LEG> foot_forces_swing;
        Eigen::Matrix<double, 3, NUM_LEG> joint_torques_swing;
        Eigen::Matrix<double, 3, NUM_LEG> joint_torques;
        Eigen::Matrix<double, 12, 1> joint_torques_stacked;
        Eigen::Matrix<double, 12, 1> joint_torques_limits;
        Eigen::Matrix<double, 12, 1> joint_pos_init;

        // movement mode trackers
        bool walking_mode;
        double squat_prog;
        std::array<bool, NUM_LEG> contacts;
        std::array<bool, NUM_LEG> contacts_old;
        int swing_phase;
        bool init;

        // swing leg PD control-related variables (ORDER IS FR, FL, RR, RL)
        double foot_deltaX;
        double foot_deltaY;
        Eigen::Matrix<double, 3, NUM_LEG> default_foot_pos; // relative frame
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_world_rot; // relative frame, but world orientation
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos; // relative frame & relatve orientation
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_old;
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs; // world frame & world orientation
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs_est;
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_d;
        Eigen::Matrix<double, 3, NUM_LEG> foot_vel_d;
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_liftoff;

        // sensor/estimated variables
        Eigen::Vector3d root_pos_est;
        Eigen::Vector3d root_lin_vel_est;
        Eigen::Vector3d root_lin_acc_est;
        Eigen::Vector3d root_lin_acc_meas;
        Eigen::Quaterniond root_quat_est;
        Eigen::Vector3d root_rpy_est;
        Eigen::Vector3d root_ang_vel_est;
        Eigen::Vector3d root_ang_vel_meas;
        Eigen::Matrix<double, 12, 1> joint_pos;
        Eigen::Matrix<double, 12, 1> joint_pos_d;
        Eigen::Matrix<double, 12, 1> joint_vel;
        Eigen::Matrix<double, 12, 1> joint_vel_d;
        Eigen::Vector4d est_contacts;

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
    
    private:
        mutable std::mutex mtx_; // maybe experiment with adding more private variables?
};

#endif //GO1_STATE_H