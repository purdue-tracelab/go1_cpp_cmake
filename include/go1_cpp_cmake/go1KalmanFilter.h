#ifndef GO1_KALMFILT_H
#define GO1_KALMfILT_H

// Open-source libraries
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>
#include "OsqpEigen/OsqpEigen.h"
#include <mujoco/mujoco.h>
#include <string>

// Package-specific header files
#include "go1Params.h"
#include "go1FK.h"
#include "go1Utils.h"
#include "go1State.h"

class go1KalmanFilter {
    public:
    // functions
    go1KalmanFilter(const char* sensor_model = "accelerometer");
    void initFilter(const char* sensor_model);
    void simpleKalmanFilterMujoco(go1State &state, Eigen::Vector3d &lin_accel_meas);

    // filter vector and matrices
    Eigen::VectorXd x_curr;
    Eigen::VectorXd x_old;
    Eigen::VectorXd z_curr;
    Eigen::VectorXd y_res;
    Eigen::MatrixXd phi;
    Eigen::MatrixXd H;
    Eigen::MatrixXd P_curr;
    Eigen::MatrixXd P_old;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd S_curr;
    Eigen::MatrixXd K_curr;

};


#endif // GO1_KALMFILT_H