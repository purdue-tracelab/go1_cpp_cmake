#include "go1_cpp_cmake/go1KalmanFilter.h"

/*
    go1KalmanFilter.cpp handles all the state estimation of our Go1
    using linear Kalman filters. There are/will be multiple versions
    based on which sensors we want to use and what we want to estimate.
    ShuoYang's repo names its filter as EKF, but it largely has the linear
    structure required of a Kalman Filter, meaning it is susceptible to
    its nature to drift since it is essentially integration across time.
    This will be used in simuation and hardware experiments for the
    evaluation/improvement of the overall control structure.
*/

go1KalmanFilter::go1KalmanFilter(const char* sensor_model) {
    initFilter(sensor_model);
}

void go1KalmanFilter::initFilter(const char* sensor_model) { // make 2d plot if wanting to show movement during experiment
/*
    Sets up the discrete system model based on the state vector,
    input, and measurement structure as follows:

    state vector x = [p_x, p_y, p_z, v_x, v_y, v_z, a_x, a_y, a_z]^T
    input u is unclear so exclude from model for now
    measurements are [a_x, a_y, a_z]^T
*/
    std::string classStr(sensor_model);
    if (classStr == "accelerometer") {
        // Resize all matrices and vectors
        x_curr.resize(9);
        x_old.resize(9);
        z_curr.resize(3);
        y_res.resize(3);
        phi.resize(9, 9);
        H.resize(3, 9);
        P_curr.resize(9, 9);
        P_old.resize(9, 9);
        Q.resize(9, 9);
        R.resize(3, 3);
        S_curr.resize(3, 3);
        K_curr.resize(3, 3);

        // Set initial state
        x_old << 0, 0, 0.27, 0, 0, 0, 0, 0, 0;
        
        // set noise parameters to default
        Eigen::Matrix<double, 9, 1> p_weights;
        p_weights << 100, 100, 100, 10, 10, 10, 1, 1, 1;
        P_old = p_weights.asDiagonal();
        Q.setIdentity();
        Q = 0.1 * Q;
        R.setIdentity();
        R = 0.1 * R;

        // Set up discrete system model
        phi.setIdentity();
        phi.block<3, 3>(0, 3) = DT_CTRL * Eigen::Matrix3d::Identity();
        phi.block<3, 3>(0, 6) = 0.5 * std::pow(DT_CTRL, 2) * Eigen::Matrix3d::Identity();
        phi.block<3, 3>(3, 6) = DT_CTRL * Eigen::Matrix3d::Identity();

        H.setZero();
        H.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
    }
}

void go1KalmanFilter::simpleKalmanFilterMujoco(go1State &state, Eigen::Vector3d &lin_accel_meas) { 
/*
    Executes the Kalman Filter algorithm according to the model
    specified in the filter initialization parameter.
*/
    // Predict next state
    x_curr = phi * x_old;
    P_curr = phi * P_old * phi.transpose() + Q;

    // Check residual to calculate optimal Kalman gain
    z_curr << lin_accel_meas[0], lin_accel_meas[1], lin_accel_meas[2] - 9.81;
    y_res = z_curr - H * x_old;
    S_curr = H * P_curr * H.transpose() + R;
    S_curr = (S_curr + S_curr.transpose())/2.0;
    K_curr = P_curr * H.transpose() * S_curr.inverse();

    // Update state estimation
    x_curr = x_curr + K_curr * y_res;
    P_curr = (Eigen::Matrix<double, 9, 9>::Identity() - K_curr * H) * P_curr;

    // Send state estimation to go1State object
    state.root_pos_est = x_curr.segment<3>(0);
    state.root_lin_vel_est = x_curr.segment<3>(3);
    state.root_lin_acc_est = x_curr.segment<3>(6);

    double root_pitch_est = asin(x_curr(6)/9.81);
    double root_roll_est = asin(-x_curr(7)/(9.81 * cos(root_pitch_est)));

    state.root_rpy_est << root_roll_est, root_pitch_est, 0;

    // Propagate current estimation to old estimation
    x_old = x_curr;
    P_old = P_curr;
}