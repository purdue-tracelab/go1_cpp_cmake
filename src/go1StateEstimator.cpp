#include "go1_cpp_cmake/go1StateEstimator.h"

/*
    go1ExtendedKalmanFilter.cpp handles all the state estimation of our Go1
    using an extended Kalman filter. This is based on Zijian's presentation
    outlining the EKF, which is a summary of the EKF paper from ETH Zurich
    listed in my onboarding document and thesis. The only modification is that
    there are four feet for the quadruped, 2 in contact (ideally) at each time
    step. This will be used in simuation and hardware experiments for the
    evaluation/improvement of the overall control structure.
*/

// go1StateEstimator::go1StateEstimator(go1State& state) {
go1StateEstimator::go1StateEstimator() {
/*
    Constructs the state estimation filter based on current
    values in go1State for position, velocity, and joint position,
    as well as STATE_EST_SELECT in the go1Params.h file.
*/
    Eigen::Matrix3d eye3 = Eigen::Matrix3d::Identity();

    switch (STATE_EST_SELECT) {
        case 0: // naive Kalman filter
        /*
            x_k: [p_x, p_y, p_z, v_x, v_y, v_z, a_x, a_y, a_z]^T
            u_k: N/A
            z_k: [a_x, a_y, a_z]^T
        */
            // Resize
            x_k.resize(9);
            x_k1.resize(9);
            z_k.resize(3);
            y_res.resize(3);
            F_k.resize(9, 9);
            H_k.resize(3, 9);
            P_k.resize(9, 9);
            P_k1.resize(9, 9);
            Q_k.resize(9, 9);
            R_k.resize(3, 3);
            S_k.resize(3, 3);
            K_k.resize(3, 3);

            // Initialize covariances
            P_k = (Eigen::Matrix<double, 9, 1>() << 100, 100, 100, 10, 10, 10, 1, 1, 1).finished().asDiagonal();
            Q_k.setIdentity();
            Q_k *= 0.1;
            R_k.setIdentity();
            R_k *= 0.1;

            // Set up discrete-time dynamics and measurement models
            F_k.setIdentity();
            F_k.block<3, 3>(0, 3) = DT_CTRL * eye3;
            F_k.block<3, 3>(0, 6) = 0.5 * std::pow(DT_CTRL, 2) * eye3;
            F_k.block<3, 3>(3, 6) = DT_CTRL * eye3;

            H_k.setZero();
            H_k.block<3, 3>(0, 6) = eye3;
            
            break;

        case 1: // kinematic Kalman filter
        /*
            x_k: [p, v, p_FR, p_FL, p_RR, p_RL]^T
            u_k: imu_accel
            z_k: [p_FR, p_FL, p_RR, p_RL, v_FR, v_FL, v_RR, v_RL, fut_height]^T
        */
            // Resize
            x_k.resize(18);
            x_k1.resize(18);
            u_k.resize(3);
            z_k.resize(28);
            y_res.resize(28);
            F_k.resize(18, 18);
            B_k.resize(18, 3);
            H_k.resize(28, 18);
            P_k.resize(18, 18);
            P_k1.resize(18, 18);
            Q_k.resize(18, 18);
            R_k.resize(28, 28);
            S_k.resize(28, 28);
            K_k.resize(18, 28);

            // Initialize covariances
            Q_k.setIdentity();
            Q_k.block<3, 3>(0, 0) = 0.01 * DT_CTRL * eye3 / 20.0;
            Q_k.block<3, 3>(3, 3) = 0.01 * DT_CTRL * 9.81 * eye3 / 20.0;

            for (int i = 0; i < NUM_LEG; i++) {
                Q_k.block<3, 3>(6 + i*3, 6 + i*3) = 0.03 * eye3;
            }

            R_k.setIdentity();
            for (int i = 0; i < NUM_LEG; i++) {
                R_k.block<3, 3>(i*3, i*3) = 0.001 * eye3;
                R_k.block<3, 3>(NUM_LEG*3 + i*3, NUM_LEG*3 + i*3) = 0.1 * eye3;
                R_k(NUM_LEG*6 + i, NUM_LEG*6 + i) = 0.001;
            }

            P_k.setIdentity();
            P_k *= 3;

            // Set up discrete-time dynamics and measurement models
            F_k.setIdentity();
            F_k.block<3, 3>(0, 3) = DT_CTRL * eye3;
            B_k.setZero();
            B_k.block<3, 3>(3, 0) = DT_CTRL * eye3;

            H_k.setZero();
            for (int i = 0; i < NUM_LEG; i++) {
                H_k.block<3, 3>(i*3, 0) = -eye3;
                H_k.block<3, 3>(i*3, 6 + i*3) = eye3;
                H_k.block<3, 3>(NUM_LEG*3 + i*3, 3) = eye3;
                H_k(NUM_LEG*6 + i, 8 + i*3) = 1;
            }

            break;

        case 2: // two-stage Kalman filter
        /*
            x_k: [p, v, p_FR, p_FL, p_RR, p_RL]^T
            u_k: imu_accel
            z_k: [p_FR, p_FL, p_RR, p_RL, v_FR, v_FL, v_RR, v_RL, fut_height]^T
        */
            // Resize
            x_k.resize(18);
            x_k1.resize(18);
            u_k.resize(3);
            z_k.resize(28);
            y_res.resize(28);
            F_k.resize(18, 18);
            B_k.resize(18, 3);
            H_k.resize(28, 18);
            P_k.resize(18, 18);
            P_k1.resize(18, 18);
            Q_k.resize(18, 18);
            R_k.resize(28, 28);
            S_k.resize(28, 28);
            K_k.resize(18, 28);

            // Initialize covariances
            Q_k.setIdentity();
            Q_k.block<3, 3>(0, 0) = 0.01 * DT_CTRL * eye3 / 20.0;
            Q_k.block<3, 3>(3, 3) = 0.01 * DT_CTRL * 9.81 * eye3 / 20.0;

            for (int i = 0; i < NUM_LEG; i++) {
                Q_k.block<3, 3>(6 + i*3, 6 + i*3) = 0.03 * eye3;
            }

            R_k.setIdentity();
            for (int i = 0; i < NUM_LEG; i++) {
                R_k.block<3, 3>(i*3, i*3) = 0.001 * eye3;
                R_k.block<3, 3>(NUM_LEG*3 + i*3, NUM_LEG*3 + i*3) = 0.1 * eye3;
                R_k(NUM_LEG*6 + i, NUM_LEG*6 + i) = 0.001;
            }

            P_k.setIdentity();
            P_k *= 3;

            // Set up discrete-time dynamics and measurement models
            F_k.setIdentity();
            F_k.block<3, 3>(0, 3) = DT_CTRL * eye3;
            B_k.setZero();
            B_k.block<3, 3>(3, 0) = DT_CTRL * eye3;

            H_k.setZero();
            for (int i = 0; i < NUM_LEG; i++) {
                H_k.block<3, 3>(i*3, 0) = -eye3;
                H_k.block<3, 3>(i*3, 6 + i*3) = eye3;
                H_k.block<3, 3>(NUM_LEG*3 + i*3, 3) = eye3;
                H_k(NUM_LEG*6 + i, 8 + i*3) = 1;
            }

            break;

        case 3: // extended Kalman filter
        /*
            x_k: [p, v, q, p_FR, p_FL, p_RR, p_RL]^T, q = root_quat (start using this or convert from )
            u_k: N/A
            z_k: [p_FR, p_FL, p_RR, p_RL]^T
        */
            //Resize
            x_k.resize(22);
            x_k1.resize(22);
            z_k.resize(12);
            y_res.resize(12);
            F_k.resize(22, 22);
            H_k.resize(12, 22);
            P_k.resize(22, 22);
            P_k1.resize(22, 22);
            Q_k.resize(22, 22);
            R_k.resize(12, 12);
            S_k.resize(12, 12);
            K_k.resize(22, 12);

            // Initialize covariances
            Q_k.setIdentity();
            Q_k *= 0.01;

            R_k.setIdentity();
            P_k.setIdentity();
            P_k *= 10.0;

            break;
        
        default:
            std::cerr << "Error: invalid STATE_EST_SELECT value, double check go1Params.h" << std::endl;
            return;
    }

}

void go1StateEstimator::collectInitialState(const go1State& state) {
/*
    Collects the initial state for estimation here. Mainly made this
    so I can initialize the estimator at the top of the file instead
    of in the middle to make code clear and consistent. If a valid
    STATE_EST_SELECT value is picked, it will go through; if not, it
    will throw an error.
*/
    switch (STATE_EST_SELECT) {
        case 0: // naive Kalman filter
            // x_k: [p_x, p_y, p_z, v_x, v_y, v_z, a_x, a_y, a_z]^T
            x_k << state.root_pos, 
                    state.root_lin_vel, 
                    state.root_lin_acc;
            break;

        case 1: // kinematic Kalman filter
            // x_k: [p, v, p_FR, p_FL, p_RR, p_RL]^T
            x_k << state.root_pos, 
                    state.root_lin_vel, 
                    state.foot_pos_abs.col(0),
                    state.foot_pos_abs.col(1),
                    state.foot_pos_abs.col(2),
                    state.foot_pos_abs.col(3);
            break;

        case 2: // two-stage Kalman filter
            // x_k: [p, v, p_FR, p_FL, p_RR, p_RL]^T
            x_k << state.root_pos, 
                    state.root_lin_vel, 
                    state.foot_pos_abs.col(0),
                    state.foot_pos_abs.col(1),
                    state.foot_pos_abs.col(2),
                    state.foot_pos_abs.col(3);
            break;

        case 3: // extended Kalman filter
            // x_k: [p, v, q, p_FR, p_FL, p_RR, p_RL]^T, q = root_quat
            x_k << state.root_pos, 
                    state.root_lin_vel, 
                    state.root_quat.w(), 
                    state.root_quat.x(), 
                    state.root_quat.y(), 
                    state.root_quat.z(), 
                    state.foot_pos_abs.col(0),
                    state.foot_pos_abs.col(1),
                    state.foot_pos_abs.col(2),
                    state.foot_pos_abs.col(3);
            break;

        default:
            std::cerr << "Error: invalid STATE_EST_SELECT value, double check go1Params.h" << std::endl;
            return;
    }
}

void go1StateEstimator::estimateState(go1State& state, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro) {
/*
    Runs the corresponding state estimation function based on
    STATE_EST_SELECT. If valid, it will move forward; if not,
    it will throw an error.
*/
    switch (STATE_EST_SELECT) {
        case 0:
            naiveKalmanFilter(state, accel, gyro);
            break;

        case 1:
            kinematicKalmanFilter(state, accel, gyro);
            break;

        case 2:
            twoStageKalmanFilter(state, accel, gyro);
            break;

        case 3:
            extendedKalmanFilter(state, accel, gyro);
            break;

        default:
            std::cerr << "Error: invalid STATE_EST_SELECT value, double check go1Params.h" << std::endl;
            return;
    }
}

void go1StateEstimator::naiveKalmanFilter(go1State& state, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro) {
/*
    Executes the naive Kalman filter by double integrating acceleration
    measurements from the IMU to estimate trunk position and velocity and
    gyroscope measurements from the IMU to estimate trunk orientation and
    angular velocity. This method is poor since it neglects the nonlinear
    relationship between rotation and position, leading to drifting.
*/
    // Predict next state
    x_k1 = F_k * x_k;
    P_k1 = F_k * P_k * F_k.transpose() + Q_k;

    // Check residual to calculate optimal Kalman gain
    Eigen::Matrix3d rootRot = rotZ(state.root_rpy(2))*rotY(state.root_rpy(1))*rotX(state.root_rpy(0));
    z_k = accel + rootRot.transpose() * Eigen::Vector3d(0, 0, -9.81);
    y_res = z_k - H_k * x_k;
    S_k = H_k * P_k1 * H_k.transpose() + R_k;
    S_k = (S_k + S_k.transpose()) / 2.0;
    K_k = P_k * H_k.transpose() * S_k.inverse();

    // Update estimation and covariance
    x_k1 = x_k1 + K_k * y_res;
    P_k1 = (Eigen::Matrix<double, 9, 9>::Identity() - K_k * H_k) * P_k1;

    // Send integrated estimates to go1State object
    state.root_pos_est = x_k1.segment<3>(0);
    state.root_lin_vel_est = x_k1.segment<3>(3);
    state.root_lin_acc_est = x_k1.segment<3>(6);
    state.root_rpy_est = state.root_rpy + DT_CTRL * gyro;
    state.root_ang_vel_est = gyro;

    // Push back
    x_k = x_k1;
    P_k = P_k1;

}

void go1StateEstimator::kinematicKalmanFilter(go1State& state, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro) {
/*
    Executes the kinematic Kalman filter by using integrated sensor measurements
    like the naiveKalmanFilter, but also estimates foot positions using forward
    kinematics of Go1, adding an "anchor" to resist typical drift since using
    accurate and precise joint encoders is typically stable and adds observability
    to the filter as a whole. Copied from ShuoYang's filter, which is similar to
    MIT's two-state controller.
*/
    // Collect input
    Eigen::Matrix3d rootRot = rotZ(state.root_rpy(2))*rotY(state.root_rpy(1))*rotX(state.root_rpy(0));
    u_k = rootRot * accel + Eigen::Vector3d(0, 0, -9.81);

    // Update covariances based on foot contact
    for (int i = 0; i < NUM_LEG; i++) {
        Q_k.block<3, 3>(6 + i*3, 6 + i*3) = (1 + (1 - state.contacts[i]) * 1e3) * DT_CTRL * 0.03 * eye3;
        R_k.block<3, 3>(i*3, i*3) = (1 + (1 - state.contacts[i]) * 1e3) * 0.001 * eye3;
        R_k.block<3, 3>(NUM_LEG*3 + i*3, NUM_LEG*3 + i*3) = (1 + (1 - state.contacts[i]) * 1e3) * 0.1 * eye3;
        R_k(NUM_LEG*6 + i, NUM_LEG*6 + i) = (1 + (1 - state.contacts[i]) * 1e3) * 0.001;
    }

    // Predict next state
    x_k1 = F_k * x_k + B_k * u_k;
    P_k1 = F_k * P_k * F_k.transpose() + Q_k;

    // Find current measurement
    for (int i = 0; i < NUM_LEG; i++) {
        z_k.block<3, 1>(i*3, 0) = rootRot * state.foot_pos.col(i);
        Eigen::Vector3d v_fut = (state.foot_pos_old.col(i) - state.foot_pos.col(i)) / DT_CTRL - skew(gyro) * state.foot_pos.col(i);
        z_k.block<3, 1>(NUM_LEG*3 + i*3, 0) = (1 - state.contacts[i]) * x_k.segment<3>(3) + state.contacts[i] * rootRot * v_fut;
        z_k(NUM_LEG*6 + i) = (1 - state.contacts[i]) * (x_k(2) + state.foot_pos(i, 2));
    }

    // Check residual for calculating optimal Kalman gain
    y_res = z_k - H_k * x_k;
    S_k = H_k * P_k1 * H_k.transpose() + R_k;
    S_k = (S_k + S_k.transpose()) / 2.0;
    K_k = P_k * H_k.transpose() * S_k.inverse();

    // Update estimation and covariance
    x_k1 = x_k1 + K_k * y_res;
    P_k1 = (Eigen::Matrix<double, 18, 18>::Identity() - K_k * H_k) * P_k1;

    if (P_k1.block<2, 2>(0, 0).determinant() > 1e-6) {
        P_k1.block<2, 16>(0, 2).setZero();
        P_k1.block<16, 2>(2, 0).setZero();
        P_k1.block<2, 2>(0, 0) /= 10.0;
    }

    // Send estimates to go1State object
    state.root_pos_est = x_k1.segment<3>(0);
    state.root_lin_vel_est = x_k1.segment<3>(3);
    state.root_lin_acc_est = accel;
    state.root_rpy_est = state.root_rpy + DT_CTRL * gyro;
    state.root_ang_vel_est = gyro;
    
    // Push back
    x_k = x_k1;
    P_k = P_k1;

}

void go1StateEstimator::twoStageKalmanFilter(go1State& state, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro) {
/*
    Executes the two-state Kalman filter method by first estimating
    the orientation change of Go1, then using said estimations for
    estimating position and velocity changes. Found in MIT's Cheetah
    3 Convex MPC paper. Attempts to alleviate deviations due to
    negligence of nonlinear relationship between position and rotation.
*/
    // Estimate orientation
    double kappa = 0.1 * std::clamp(1.0 - (accel - Eigen::Vector3d(0, 0, 9.81)).norm() / 9.81, 0.0, 1.0);
    Eigen::Matrix3d rootRot = rotZ(state.root_rpy(2))*rotY(state.root_rpy(1))*rotX(state.root_rpy(0));
    Eigen::Vector3d omg_corr = skew(accel.normalized()) * rootRot * Eigen::Vector3d(0, 0, 1);
    Eigen::Matrix3d rootRotEst = rootRot * (eye3 + skew(DT_CTRL*(gyro + kappa * omg_corr)));

    // Send orientation estimates to go1State object
    state.root_rpy_est = rotM2Euler(rootRotEst);
    state.root_ang_vel_est = gyro;

    // Calculate input
    u_k = rootRot * accel + Eigen::Vector3d(0, 0, -9.81);

    // Update covariances based on foot contact
    for (int i = 0; i < NUM_LEG; i++) {
        Q_k.block<3, 3>(6 + i*3, 6 + i*3) = (1 + (1 - state.contacts[i]) * 1e3) * DT_CTRL * 0.03 * eye3;
        R_k.block<3, 3>(i*3, i*3) = (1 + (1 - state.contacts[i]) * 1e3) * 0.001 * eye3;
        R_k.block<3, 3>(NUM_LEG*3 + i*3, NUM_LEG*3 + i*3) = (1 + (1 - state.contacts[i]) * 1e3) * 0.1 * eye3;
        R_k(NUM_LEG*6 + i, NUM_LEG*6 + i) = (1 + (1 - state.contacts[i]) * 1e3) * 0.001;
    }

    // Predict next state
    x_k1 = F_k * x_k + B_k * u_k;
    P_k1 = F_k * P_k * F_k.transpose() + Q_k;

    // Find current measurement
    for (int i = 0; i < NUM_LEG; i++) {
        z_k.block<3, 1>(i*3, 0) = rootRot * state.foot_pos.col(i);
        Eigen::Vector3d v_fut = (state.foot_pos_old.col(i) - state.foot_pos.col(i)) / DT_CTRL - skew(gyro) * state.foot_pos.col(i);
        z_k.block<3, 1>(NUM_LEG*3 + i*3, 0) = (1 - state.contacts[i]) * x_k.segment<3>(3) + state.contacts[i] * rootRot * v_fut;
        z_k(NUM_LEG*6 + i) = (1 - state.contacts[i]) * (x_k(2) + state.foot_pos(i, 2));
    }

    // Check residual for calculating optimal Kalman gain
    y_res = z_k - H_k * x_k;
    S_k = H_k * P_k1 * H_k.transpose() + R_k;
    S_k = (S_k + S_k.transpose()) / 2.0;
    K_k = P_k * H_k.transpose() * S_k.inverse();

    // Update estimation and covariance
    x_k1 = x_k1 + K_k * y_res;
    P_k1 = (Eigen::Matrix<double, 18, 18>::Identity() - K_k * H_k) * P_k1;

    if (P_k1.block<2, 2>(0, 0).determinant() > 1e-6) {
        P_k1.block<2, 16>(0, 2).setZero();
        P_k1.block<16, 2>(2, 0).setZero();
        P_k1.block<2, 2>(0, 0) /= 10.0;
    }

    // Send estimates to go1State object
    state.root_pos_est = x_k1.segment<3>(0);
    state.root_lin_vel_est = x_k1.segment<3>(3);
    state.root_lin_acc_est = accel;
    
    // Push back
    x_k = x_k1;
    P_k = P_k1;
}

void go1StateEstimator::extendedKalmanFilter(go1State& state, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro) {
/*
    Executes the extended Kalman filter by linearizing the nonlinear
    dynamics and relationship between position and orientation by
    finding the Jacobian of the continuous state dynamics at each
    state according to fState & hState in the header file. Should be
    more accurate than all the linear Kalman filters, but not perfect.
    Based on the ETH Zurich paper showed to me by Zijian & Falak.
*/
    // Predict next state
    x_k1 = fState(x_k, accel, gyro);
    Eigen::VectorXd foot_pos_abs = state.foot_pos_abs.reshaped();

    // Update covariances based on foot contact
    for (int i = 0; i < NUM_LEG; i++) {
        Q_k.block<3, 3>(10 + i*3, 10 + i*3) = (1 + (1 - state.contacts[i]) * 1e10) * 0.01 * eye3;
    }

    // Check residual for calculation optimal Kalman gain
    F_k = numericalJacobian(fState, x_k, 1e-6, accel, gyro);
    P_k1 = F_k * P_k * F_k.transpose() + Q_k;
    y_res = foot_pos_abs - hState(x_k);
    H_k = numericalJacobian(hState, x_k);
    S_k = H_k * P_k1 * H_k.transpose() + R_k;
    S_k = (S_k + S_k.transpose()) / 2.0;
    K_k = P_k * H_k.transpose() * S_k.inverse();

    // Update estimation and covariance
    x_k1 = x_k1 + K_k * y_res;
    P_k1 = (Eigen::Matrix<double, 22, 22>::Identity() - K_k * H_k) * P_k1;

    // Send estimates to go1State object
    state.root_pos_est = x_k1.segment<3>(0);
    state.root_lin_vel_est = x_k1.segment<3>(3);
    state.root_lin_acc_est = accel;
    state.root_quat_est = Eigen::Quaterniond(x_k1(6), x_k1(7), x_k1(8), x_k1(9));
    state.root_rpy_est = rotM2Euler(state.root_quat_est.toRotationMatrix());
    state.root_ang_vel_est = gyro;

    // Push back
    x_k = x_k1;
    P_k = P_k1;

}

