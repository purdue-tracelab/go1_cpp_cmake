#ifndef GO1_STATE_H
#define GO1_STATE_H

// Open-source libraries
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>
#include "OsqpEigen/OsqpEigen.h"
#include <mujoco/mujoco.h>
#include <mutex>

// Package-specific header files
#include "go1Params.h"
#include "go1FK.h"
#include "go1Utils.h"

struct go1StateSnapshot {
    // grab from go1State object
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos; // relative frame
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

    // send to go1State.foot_forces_grf
    Eigen::Matrix<double, 3, NUM_LEG> grf_forces;
};
<<<<<<< HEAD
=======

>>>>>>> 50a3885e9b70685f8a8136f2940bb4345f173823

class go1State {
    public:
        // functions
        go1State();
        void resetState();
        void updateStateFromMujoco(const mjtNum* q_vec, const mjtNum* q_vel, const Eigen::Vector3d &lin_acc);
        void convertForcesToTorquesMujoco(const mjtNum* q_vec);
        void convertForcesToTorquesHardware(const Eigen::VectorXd &jointPos);
        void raibertHeuristic(bool withCapturePoint = false);
        void amirHLIP();
        void swingPD(int leg_idx, Eigen::Vector3d footPosRef, Eigen::Vector3d footVelRef, bool absolute = false);
        void jointPD(int joint_idx, double jointPos, double jointVel, bool startup = true);
        Eigen::Vector3d bezierPos();
        Eigen::Vector3d bezierVel();

        // experimental functions for FSM
        bool isStartupComplete() const;
        bool isShutdownComplete() const;
        void computeStartupPDMujoco(const mjtNum* q_vec, const mjtNum* q_vel);
        void computeShutdownPDMujoco(const mjtNum* q_vec, const mjtNum* q_vel);
        bool getWalkingMode() const { return walking_mode; }
        void setWalkingMode(bool v) { walking_mode = v; }
        int getSwingPhase() const { return swing_phase; }
        go1StateSnapshot getSnapshot() const;
        void setGRFForces(const Eigen::Matrix<double, 3, NUM_LEG> &grf);

        // default information
        Eigen::Matrix3d go1_lumped_inertia;

        // state variables
        Eigen::Vector3d root_pos;
        Eigen::Vector3d root_pos_old;
        Eigen::Vector3d root_pos_d;
        Eigen::Vector3d root_pos_est;
        Eigen::Vector3d root_lin_vel;
        Eigen::Vector3d root_lin_vel_d;
        Eigen::Vector3d root_lin_vel_est;
        Eigen::Vector3d root_lin_acc;
        Eigen::Vector3d root_lin_acc_est;
        Eigen::Vector3d root_lin_acc_meas;

        Eigen::Quaterniond root_quat;
        Eigen::Quaterniond root_quat_old;
        Eigen::Quaterniond root_quat_est;
        Eigen::Vector3d root_rpy;
        Eigen::Vector3d root_rpy_old;
        Eigen::Vector3d root_rpy_d;
        Eigen::Vector3d root_rpy_est;
        Eigen::Vector3d root_ang_vel;
        Eigen::Vector3d root_ang_vel_d;
        Eigen::Vector3d root_ang_vel_est;
        Eigen::Vector3d root_ang_vel_meas;

        // actuation-related variables (ORDER IS FR, FL, RR, RL)
        Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
        Eigen::Matrix<double, 3, NUM_LEG> foot_forces_swing;
        Eigen::Matrix<double, 3, NUM_LEG> joint_torques;
        Eigen::Matrix<double, 12, 1> joint_pos_init;

<<<<<<< HEAD
        // movement mode trackers
=======
        // gait phase
        double squat_prog;
>>>>>>> 50a3885e9b70685f8a8136f2940bb4345f173823
        bool walking_mode;
        double squat_prog;
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
    
<<<<<<< HEAD
    private:
        mutable std::mutex mtx_; // maybe experiment with adding more private variables?
=======
        private:
            mutable std::mutex mtx_; // maybe experiment with adding more private variables?
>>>>>>> 50a3885e9b70685f8a8136f2940bb4345f173823
};

#endif //GO1_STATE_H