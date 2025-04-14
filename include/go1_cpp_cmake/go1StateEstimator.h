#ifndef GO1_STATE_EST_H
#define GO1_STATE_EST_H

// Open-source libraries
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>
#include <string>

// Package-specific header files
#include "go1Params.h"
#include "go1FK.h"
#include "go1Utils.h"
#include "go1State.h"

class go1StateEstimator {
    public:
        // functions
        // go1StateEstimator(go1State& state);
        go1StateEstimator();
        void collectInitialState(const go1State& state);
        void estimateState(go1State& state, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro);
        void naiveKalmanFilter(go1State& state, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro);
        void kinematicKalmanFilter(go1State& state, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro);
        void twoStageKalmanFilter(go1State& state, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro);
        void extendedKalmanFilter(go1State& state, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro);
        
        // filter vector and matrices
        Eigen::VectorXd x_k;
        Eigen::VectorXd x_k1;
        Eigen::VectorXd u_k;
        Eigen::VectorXd z_k;
        Eigen::VectorXd y_res;
        Eigen::MatrixXd F_k;
        Eigen::MatrixXd B_k;
        Eigen::MatrixXd H_k;
        Eigen::MatrixXd P_k;
        Eigen::MatrixXd P_k1;
        Eigen::MatrixXd Q_k;
        Eigen::MatrixXd R_k;
        Eigen::MatrixXd S_k;
        Eigen::MatrixXd K_k;
        Eigen::Matrix3d eye3;

};

inline Eigen::VectorXd fState(const Eigen::VectorXd& x_k, const Eigen::Vector3d& f_meas, const Eigen::Vector3d& omg_meas) {
/*
    fState represents the nonlinear state transition function 
    between the current and new state for the nonlinear
    extendedKalmanFilter function.
*/
    Eigen::VectorXd x_k1 = x_k;
    Eigen::Vector3d grav (0, 0, -9.81);
    Eigen::Matrix3d C_k = quat2RotM(x_k.segment<4>(6));

    // r_k+1
    x_k1.segment<3>(0) = x_k.segment<3>(0) + DT_CTRL * x_k.segment<3>(3) + 0.5 * std::pow(DT_CTRL, 2) * (C_k.transpose() * f_meas + grav);

    // v_k+1
    x_k1.segment<3>(3) = x_k.segment<3>(3) + DT_CTRL * (C_k.transpose() * f_meas + grav);

    // q_k+1
    Eigen::Quaterniond q(x_k(6), x_k(7), x_k(8), x_k(9));
    q.normalize();
    Eigen::Quaterniond dq;

    Eigen::Vector3d deltaRPY = DT_CTRL * omg_meas;
    double angle = deltaRPY.norm();         

    if (angle < 1e-6) {
        dq = Eigen::Quaterniond(1.0, 0.5 * deltaRPY(0), 0.5 * deltaRPY(1), 0.5 * deltaRPY(2));
    } else {
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, deltaRPY / angle));
    }

    Eigen::Quaterniond q_k1 = q * dq;
    q_k1.normalize();

    x_k1.segment<4>(6) << q_k1.w(), q_k1.x(), q_k1.y(), q_k1.z();

    return x_k1;
}

inline Eigen::VectorXd hState(const Eigen::VectorXd& x_k) {
/*
    hState represents the nonlinear measurement model for the 
    current state for the extendedKalmanFilter function.
*/
    Eigen::VectorXd z_k(12);
    Eigen::Matrix3d C_k = quat2RotM(x_k.segment<4>(6));

    z_k.segment<3>(0) = C_k * (x_k.segment<3>(10) - x_k.segment<3>(0));
    z_k.segment<3>(3) = C_k * (x_k.segment<3>(13) - x_k.segment<3>(0));
    z_k.segment<3>(6) = C_k * (x_k.segment<3>(16) - x_k.segment<3>(0));
    z_k.segment<3>(9) = C_k * (x_k.segment<3>(19) - x_k.segment<3>(0));

    return z_k;

}

#endif // GO1_STATE_EST_H