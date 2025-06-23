#include "go1_cpp_cmake/go1StateEstimator.h"

NaiveKF::NaiveKF() {
/*
    NaiveKF executes the naive Kalman filter by integrating acceleration
    measurements from the IMU to estimate trunk position and velocity and
    gyroscope measurements from the IMU to estimate trunk orientation and
    angular velocity. This method is poor since it neglects the nonlinear
    relationship between rotation and position, leading to drifting.

    x_k: [p_x, p_y, p_z, v_x, v_y, v_z, a_x, a_y, a_z]^T
    u_k: N/A
    z_k: [a_x, a_y, a_z]^T
*/
    eye3.setIdentity();

    // Clear internal variables
    x_k.setZero();
    x_k1.setZero();
    z_k.setZero();
    y_res.setZero();
    P_k1.setZero();
    S_k.setZero();
    K_k.setZero();

    // Initialize covariances
    P_k = (Eigen::Matrix<double, 9, 1>() << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3).finished().asDiagonal();
    Q_k = (Eigen::Matrix<double, 9, 1>() << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished().asDiagonal();
    R_k.setIdentity();
    R_k *= 1e-4;

    // Set up discrete-time dynamics and measurement models
    F_k.setIdentity();
    F_k.block<3, 3>(0, 3) = DT_CTRL * eye3;
    F_k.block<3, 3>(0, 6) = 0.5 * std::pow(DT_CTRL, 2) * eye3;
    F_k.block<3, 3>(3, 6) = DT_CTRL * eye3;

    H_k.setZero();
    H_k.block<3, 3>(0, 6) = eye3;
}

void NaiveKF::collectInitialState(const go1State& state) {
    x_k << state.root_pos, state.root_lin_vel, state.root_lin_acc;
}

void NaiveKF::estimateState(go1State& state) {
    // Predict next state
    x_k1 = F_k * x_k;
    P_k1 = F_k * P_k * F_k.transpose() + Q_k;

    // Check residual to calculate optimal Kalman gain
    Eigen::Matrix3d rootRot = rotZ(state.root_rpy_est(2))*rotY(state.root_rpy_est(1))*rotX(state.root_rpy_est(0));
    z_k = rootRot * state.root_lin_acc_meas + Eigen::Vector3d(0, 0, -9.81);
    y_res = z_k - H_k * x_k1;
    S_k = H_k * P_k1 * H_k.transpose() + R_k;
    S_k = (S_k + S_k.transpose()) / 2.0;
    K_k = P_k1 * H_k.transpose() * S_k.inverse();

    // Update estimation and covariance
    x_k1 += K_k * y_res;
    P_k1 -= K_k * H_k * P_k1;

    // Send integrated estimates to go1State object
    state.root_pos_est = x_k1.segment<3>(0);
    state.root_lin_vel_est = x_k1.segment<3>(3);
    state.root_lin_acc_est = x_k1.segment<3>(6);
    state.root_rpy_est = state.root_rpy_est + DT_CTRL * state.root_ang_vel_meas;
    state.root_ang_vel_est = state.root_ang_vel_meas;

    // Push back
    x_k = x_k1;
    P_k = P_k1;
}

/////////////////////////////////////////////

KinematicKF::KinematicKF() {
/*
    KinematicKF executes the kinematic Kalman filter by using integrated sensor 
    measurements like NaiveKF, but also estimates foot positions using forward
    kinematics of Go1, adding an "anchor" to resist typical drift since using
    accurate and precise joint encoders is typically stable and adds observability
    to the filter as a whole. Copied from ShuoYang's filter, which is similar to
    MIT's two-state controller.

    x_k: [p, v, p_FR, p_FL, p_RR, p_RL]^T
    u_k: imu_accel
    z_k: [p_FR - p, p_FL - p, p_RR - p, p_RL, v_FR - v, v_FL - v, v_RR - v, v_RL - v, fut_height]^T
*/
    eye3.setIdentity();

    // Clear internal variables
    x_k.setZero();
    x_k1.setZero();
    u_k.setZero();
    z_k.setZero();
    y_res.setZero();
    P_k1.setZero();
    S_k.setZero();
    K_k.setZero();

    // Initialize covariances
    Q_k.setIdentity();
    Q_k.block<3, 3>(0, 0) = 0.01 * DT_CTRL * eye3 / 20.0;
    Q_k.block<3, 3>(3, 3) = 0.01 * DT_CTRL * 9.81 * eye3 / 20.0;

    for (int i = 0; i < NUM_LEG; i++) {
        Q_k.block<3, 3>(6 + i*3, 6 + i*3) = 0.03 * DT_CTRL * eye3;
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
        H_k.block<3, 3>(NUM_LEG*3 + i*3, 3) = -eye3;
        H_k(NUM_LEG*6 + i, 8 + i*3) = 1;
    }
}

void KinematicKF::collectInitialState(const go1State& state) {
    x_k << state.root_pos, 
        state.root_lin_vel, 
        state.foot_pos_abs.col(0),
        state.foot_pos_abs.col(1),
        state.foot_pos_abs.col(2),
        state.foot_pos_abs.col(3);
}

void KinematicKF::setFootHeightResidual(double h) {
    for (int i = 0; i < NUM_LEG; i++) {
        z_k(NUM_LEG*6 + i) = h;
    }
}

void KinematicKF::estimateState(go1State& state) {
    // Collect input
    Eigen::Matrix3d rootRot = rotZ(state.root_rpy_est(2))*rotY(state.root_rpy_est(1))*rotX(state.root_rpy_est(0));
    u_k = rootRot * state.root_lin_acc_meas + Eigen::Vector3d(0, 0, -9.81);

    // Update covariances based on foot contact
    double trust;
    for (int i = 0; i < NUM_LEG; i++) {
        trust = state.contacts[i] ? 1.0 : 1e6;
        Q_k.block<3, 3>(6 + i*3, 6 + i*3) = trust * DT_CTRL * 0.03 * eye3;
        R_k.block<3, 3>(i*3, i*3) = trust * 0.001 * eye3;
        R_k.block<3, 3>(NUM_LEG*3 + i*3, NUM_LEG*3 + i*3) = trust * 0.1 * eye3;
        R_k(NUM_LEG*6 + i, NUM_LEG*6 + i) = trust * 0.001;
    }

    // Predict next state
    x_k1 = F_k * x_k + B_k * u_k;
    P_k1 = F_k * P_k * F_k.transpose() + Q_k;

    // Find current measurement
    for (int i = 0; i < NUM_LEG; i++) {
        z_k.block<3, 1>(i*3, 0) = state.foot_pos_world_rot.col(i);
        Eigen::Vector3d v_fut = rootRot * (state.foot_pos.col(i) - state.foot_pos_old.col(i)) / DT_CTRL - skew(state.root_ang_vel_meas) * state.foot_pos_world_rot.col(i);
        z_k.block<3, 1>(NUM_LEG*3 + i*3, 0) = v_fut;
    }

    // Check residual for calculating optimal Kalman gain
    y_res = z_k - H_k * x_k1;
    S_k = H_k * P_k1 * H_k.transpose() + R_k;
    S_k = (S_k + S_k.transpose()) / 2.0;
    K_k = P_k1 * H_k.transpose() * S_k.inverse();

    // Update estimation and covariance
    x_k1 += K_k * y_res;
    P_k1 -= K_k * H_k * P_k1;

    // Optional adjustment, not sure why it's used
    if (P_k1.block<2, 2>(0, 0).determinant() > 1e-6) {
        P_k1.block<2, 16>(0, 2).setZero();
        P_k1.block<16, 2>(2, 0).setZero();
        P_k1.block<2, 2>(0, 0) /= 10.0;
    }

    // Send estimates to go1State object
    state.root_pos_est = x_k1.segment<3>(0);
    state.root_lin_vel_est = x_k1.segment<3>(3);
    state.root_lin_acc_est = state.root_lin_acc_meas;
    state.root_rpy_est = state.root_rpy + DT_CTRL * state.root_ang_vel_meas;
    state.root_ang_vel_est = state.root_ang_vel_meas;
    
    // Push back
    x_k = x_k1;
    P_k = P_k1;
}

/////////////////////////////////////////////

TwoStageKF::TwoStageKF() {
/*
    TwoStageKF executes the two-state Kalman filter method by first
    estimating the orientation change of Go1, then using said estimations
    for estimating position and velocity changes. Found in MIT's Cheetah
    3 Convex MPC paper. Attempts to alleviate deviations due to
    negligence of nonlinear relationship between position and rotation.

    x_k: [p, v, p_FR, p_FL, p_RR, p_RL]^T
    u_k: imu_accel
    z_k: [p_FR - p, p_FL - p, p_RR - p, p_RL, v_FR - v, v_FL - v, v_RR - v, v_RL - v, fut_height]^T
*/
    eye3.setIdentity();

    // Clear internal variables
    x_k.setZero();
    x_k1.setZero();
    u_k.setZero();
    z_k.setZero();
    y_res.setZero();
    P_k1.setZero();
    Q_k.setZero();
    R_k.setZero();
    S_k.setZero();
    K_k.setZero();

    ////////////////////////////
    // Initialize covariances //
    /////////////////////////////

    // My Q values (try increasing)
    Q_k.setIdentity();
    Q_k.block<3, 3>(0, 0) = DT_CTRL * eye3 / 20.0;
    Q_k.block<3, 3>(3, 3) = DT_CTRL * 9.81 * eye3 / 20.0;
    // Q_k.block<3, 3>(0, 0) = 0.02 * DT_CTRL * eye3;
    // Q_k.block<3, 3>(3, 3) = 0.1 * DT_CTRL * 9.81 * eye3;

    for (int i = 0; i < NUM_LEG; i++) {
        Q_k.block<3, 3>(6 + i*3, 6 + i*3) = 0.03 * DT_CTRL * eye3;
    }

    // // Muqun's Q values
    // Q_0.setIdentity();
    // Q_0.block<3, 3>(0, 0) = DT_CTRL / 20.0 * eye3;
    // Q_0.block<3, 3>(3, 3) = DT_CTRL * 9.81 / 20.0 * eye3;
    // Q_0.block<12, 12>(6, 6) = DT_CTRL * Eigen::Matrix<double, 12, 12>::Identity();

    // My R values
    R_k.setIdentity();
    for (int i = 0; i < NUM_LEG; i++) {
        R_k.block<3, 3>(i*3, i*3) = 0.001 * eye3;
        R_k.block<3, 3>(NUM_LEG*3 + i*3, NUM_LEG*3 + i*3) = 0.1 * eye3;
        R_k(NUM_LEG*6 + i, NUM_LEG*6 + i) = 0.001;
    }

    // // Muqun's R values
    // R_0.setIdentity();

    P_k.setIdentity();

    // My P values
    P_k.block<3, 3>(0, 0) = 0.01 * eye3;
    P_k.block<3, 3>(3, 3) = 0.01 * eye3;

    for (int i = 0;i < NUM_LEG; i++) {
        P_k.block<3, 3>(6 + i*3, 6 + i*3) = 0.01 * eye3;
    }

    // // Muqun's P values
    // P_k *= 100.0;

    // Set up discrete-time dynamics and measurement models
    F_k.setIdentity();
    F_k.block<3, 3>(0, 3) = DT_CTRL * eye3;
    B_k.setZero();
    B_k.block<3, 3>(3, 0) = DT_CTRL * eye3;

    H_k.setZero();
    // My measurement model
    for (int i = 0; i < NUM_LEG; i++) {
        H_k.block<3, 3>(i*3, 0) = -eye3;
        H_k.block<3, 3>(i*3, 6 + i*3) = eye3;
        H_k.block<3, 3>(NUM_LEG*3 + i*3, 3) = -eye3;
        H_k(NUM_LEG*6 + i, 8 + i*3) = 1;
    }

    // // Muqun's measurement model
    // for (int i = 0; i < NUM_LEG; i++) {
    //     H_k.block<3, 3>(i*3, 0) = eye3;
    //     H_k.block<3, 3>(i*3, 6 + i*3) = -eye3;
    //     H_k.block<3, 3>(NUM_LEG*3 + i*3, 3) = eye3;
    //     H_k(NUM_LEG*6 +i, 8 + i*3) = 1;
    // }
}

void TwoStageKF::collectInitialState(const go1State& state) {
    x_k << state.root_pos,
            state.root_lin_vel,
            state.foot_pos_abs.col(0),
            state.foot_pos_abs.col(1),
            state.foot_pos_abs.col(2),
            state.foot_pos_abs.col(3);
}

void TwoStageKF::setFootHeightResidual(double h) {
    for (int i = 0; i < NUM_LEG; i++) {
        z_k(NUM_LEG*6 + i) = h;
    }
}

void TwoStageKF::estimateState(go1State& state) {
    // Estimate orientation
    double kappa = 0.1 * std::clamp(1.0 - (state.root_lin_acc_meas - Eigen::Vector3d(0, 0, 9.81)).norm() / 9.81, 0.0, 1.0);
    Eigen::Matrix3d rootRot = rotZ(state.root_rpy_est(2))*rotY(state.root_rpy_est(1))*rotX(state.root_rpy_est(0));
    Eigen::Vector3d omg_corr = skew(state.root_lin_acc_meas.normalized()) * rootRot.transpose() * Eigen::Vector3d(0, 0, 1);
    Eigen::Matrix3d rootRotEst = rootRot * (eye3 + skew(DT_CTRL * (state.root_ang_vel_meas + kappa * omg_corr)));

    // Send orientation estimates to go1State object
    state.root_rpy_est = rotM2Euler(rootRotEst);
    state.root_ang_vel_est = state.root_ang_vel_meas;

    // Calculate input
    u_k = rootRotEst * state.root_lin_acc_meas + Eigen::Vector3d(0, 0, -9.81);

    //////////////////////////////////////////////
    // Update covariances based on foot contact //
    //////////////////////////////////////////////

    // // Muqun's default Q_k and R_k values
    // Q_k.setIdentity();
    // Q_k.block<3, 3>(0, 0) = process_noise_pimu * Q_0.block<3, 3>(0, 0);
    // Q_k.block<3, 3>(3, 3) = process_noise_vimu * Q_0.block<3, 3>(3, 3);
    // Q_k.block<12, 12>(6, 6) = process_noise_pfoot * Q_0.block<12, 12>(6, 6);

    // R_k.setIdentity();
    // R_k.block<12, 12>(0, 0) = sensor_noise_pimu_rel_foot * R_0.block<12, 12>(0, 0);
    // R_k.block<12, 12>(12, 12) = sensor_noise_vimu_rel_foot * R_0.block<12, 12>(12, 12);
    // R_k.block<4, 4>(24, 24) = sensor_noise_zfoot * R_0.block<4, 4>(24, 24);

    double trust;
    // My Q_k and R_k contact-based values
    for (int i = 0; i < NUM_LEG; i++) {
        trust = state.contacts[i] ? 1.0 : 1e6;
        Q_k.block<3, 3>(6 + i*3, 6 + i*3) = trust * DT_CTRL * 0.03 * eye3;
        R_k.block<3, 3>(i*3, i*3) = trust * 0.005 * eye3;
        R_k.block<3, 3>(NUM_LEG*3 + i*3, NUM_LEG*3 + i*3) = trust * 2.0 * eye3; //trust * 0.6 * eye3; //trust * 0.3 * eye3;
        R_k(NUM_LEG*3 + i*3 + 2, NUM_LEG*3 + i*3 + 2) = trust * 0.3; // z height tuning
        R_k(NUM_LEG*6 + i, NUM_LEG*6 + i) = trust * 0.001;
    } 

    // // Muqun's Q_k and R_k contact-based values
    // for (int i = 0; i < NUM_LEG; i++) {
    //     trust = state.contacts[i] ? 1.0 : 100.0;
    //     Q_k.block<3, 3>(6 + i*3, 6 + i*3) = trust * Q_k.block<3, 3>(6 + i*3, 6 + i*3);
    //     R_k.block<3, 3>(NUM_LEG*3 + i*3, NUM_LEG*3 + i*3) = trust * R_k.block<3, 3>(NUM_LEG*3 + i*3, NUM_LEG*3 + i*3);
    //     R_k(NUM_LEG*6 + i, NUM_LEG*6 + i) = trust * R_k(NUM_LEG*6 + i, NUM_LEG*6 + i);
    // }

    //////////////////////////////////////////////

    // Predict next state
    x_k1 = F_k * x_k + B_k * u_k;
    P_k1 = F_k * P_k * F_k.transpose() + Q_k;

    //////////////////////////////
    // Find current measurement //
    //////////////////////////////

    // My measurement model
    for (int i = 0; i < NUM_LEG; i++) {
        z_k.block<3, 1>(i*3, 0) = state.foot_pos_world_rot.col(i);
        Eigen::Vector3d v_fut = rootRot * (state.foot_pos.col(i) - state.foot_pos_old.col(i)) / DT_CTRL 
                                + rootRot * skew(state.root_ang_vel_meas) * state.foot_pos.col(i);
        z_k.block<3, 1>(NUM_LEG*3 + i*3, 0) = v_fut;
        z_k(NUM_LEG*6 + i) = 0.02; // removes z-height offset for foot position
    }

    // // Muqun's measurement model
    // double trust_2;
    // for (int i = 0; i < NUM_LEG; i++) {
    //     trust_2 = state.contacts[i] ? 1.0 : 0.0;
    //     z_k.block<3, 1>(i*3, 0) = -state.foot_pos_world_rot.col(i);
    //     Eigen::Vector3d v_fut_rel = (state.foot_pos.col(i) - state.foot_pos_old.col(i)) / DT_CTRL;
    //     Eigen::Vector3d v_fut = rootRotEst.transpose() * (skew(state.root_ang_vel_meas) * state.foot_pos.col(i) + v_fut_rel);
    //     z_k.block<3, 1>(NUM_LEG*3 + i*3, 0) = (1.0 - trust_2) * state.root_lin_vel - trust_2 * v_fut;
    //     z_k(NUM_LEG*6 + i) = (1.0 - trust_2) * (state.root_pos_est.z() + state.foot_pos.col(i).z());
    // }

    // Check residual for calculating optimal Kalman gain
    y_res = z_k - H_k * x_k1;
    S_k = H_k * P_k1 * H_k.transpose() + R_k;
    S_k = (S_k + S_k.transpose()) / 2.0;
    K_k = P_k1 * H_k.transpose() * S_k.inverse();

    // Update estimation and covariance
    x_k1 += K_k * y_res;
    P_k1 -= K_k * H_k * P_k1;
    P_k1 = (P_k1 + P_k1.transpose()) / 2.0; // Muqun's P_k1 update

    // Optional adjustment, not sure why it's used
    if (P_k1.block<2, 2>(0, 0).determinant() > 1e-6) {
        P_k1.block<2, 16>(0, 2).setZero();
        P_k1.block<16, 2>(2, 0).setZero();
        P_k1.block<2, 2>(0, 0) /= 10.0;
    }

    // Send estimates to go1State object
    state.root_pos_est = x_k1.segment<3>(0);
    state.root_lin_vel_est = x_k1.segment<3>(3);
    state.root_lin_acc_est = state.root_lin_acc_meas;
    
    // Push back
    x_k = x_k1;
    P_k = P_k1;
}

/////////////////////////////////////////////

ExtendedKF::ExtendedKF() {
/*
    ExtendedKF executes the extended Kalman filter by linearizing the
    nonlinear dynamics and relationship between position and orientation by
    finding the Jacobian of the continuous state dynamics at each
    state according to fState & hState in the header file. Should be
    more accurate than all the linear Kalman filters, but not perfect.
    Based on the ETH Zurich paper showed to me by Zijian & Falak.

    x_k: [p, v, q, p_FR, p_FL, p_RR, p_RL]^T, q = root_quat (start using this or convert from)
    u_k: N/A
    z_k: [p_FR - p, p_FL - p, p_RR - p, p_RL - p]^T
*/
    eye3.setIdentity();

    //////////////////////////////
    // Clear internal variables //
    //////////////////////////////

    x_k.setZero();
    x_k1.setZero(); delta_x_k1_man.setZero();
    z_k.setZero();
    foot_pos.setZero();
    y_res.setZero();
    S_k.setZero();

    // // Numerical method-related variables
    // F_k.setZero();
    // H_k.setZero();
    // P_k1.setZero();
    // K_k.setZero();

    // Analytical method-related variables
    F_k_man.setZero();
    H_k_man.setZero();
    P_k1_man.setZero();
    K_k_man.setZero();

    ////////////////////////////
    // Initialize covariances //
    ////////////////////////////

    // // Numerical method-related covariances
    // Q_k.setIdentity();
    // Q_k *= 0.01;

    // R_k.setIdentity();
    // P_k.setIdentity();
    // P_k *= 10.0;

    // Analytical method-related covariances
    Q_k_man.setIdentity();
    Q_k_man *= 0.01;

    R_k.setIdentity();
    P_k_man.setIdentity();
    P_k_man.block<3, 3>(0, 0) *= 100.0; // position
    P_k_man.block<3, 3>(3, 3) *= 100.0; // velocity
    P_k_man.block<3, 3>(6, 6) *= 10.0; // orientation
    P_k_man.block<12, 12>(9, 9) *= 100.0; // foot positions
}

void ExtendedKF::collectInitialState(const go1State& state) {
    x_k << state.root_pos,
            state.root_lin_vel,
            state.root_quat.w(), state.root_quat.x(), state.root_quat.y(), state.root_quat.z(),
            state.foot_pos_abs.col(0),
            state.foot_pos_abs.col(1),
            state.foot_pos_abs.col(2),
            state.foot_pos_abs.col(3);
}

/* // Numertical EKF state estimation
void ExtendedKF::estimateState(go1State& state) {
    // Predict next state (make sure to use relative foot position as measurement)
    x_k1 = fState(x_k, state.root_lin_acc_meas, state.root_ang_vel_meas);
    // foot_pos_abs << state.foot_pos_abs.col(0), 
    //                 state.foot_pos_abs.col(1), 
    //                 state.foot_pos_abs.col(2), 
    //                 state.foot_pos_abs.col(3);
    foot_pos << state.foot_pos_world_rot.col(0), 
                state.foot_pos_world_rot.col(1), 
                state.foot_pos_world_rot.col(2), 
                state.foot_pos_world_rot.col(3);

    // Update covariances based on foot contact
    for (int i = 0; i < NUM_LEG; i++) {
        Q_k.block<3, 3>(10 + i*3, 10 + i*3) = (1.0 + (1 - state.contacts[i]) * 1.0e10) * 0.01 * eye3;
    }

    // Find model and measurement Jacobians numerically
    F_k = numericalJacobianFixedSize<22, 22>(fState, x_k, 1e-6, false, state.root_lin_acc_meas, state.root_ang_vel_meas);
    H_k = numericalJacobianFixedSize<12, 22>(hState, x_k1, 1e-6, false);

    // Check residual for calculating optimal Kalman gain
    F_P_k.noalias() = F_k * P_k;
    P_k1.noalias() = F_P_k * F_k.transpose();
    P_k1 += Q_k;

    // y_res = foot_pos_abs - hState(x_k1);
    y_res = foot_pos - hState(x_k1);
    S_k.noalias() = H_k * P_k1 * H_k.transpose();
    S_k += R_k;

    // Invert S_k w/ LDLT instead of S_k.inverse() for speed + stability
    S_k_ldlt.compute(S_k.selfadjointView<Eigen::Upper>());

    if (S_k_ldlt.info() != Eigen::Success) {
        std::ostringstream oss;
        oss << "NaN/Inf in S_k ";
        throw std::runtime_error(oss.str());
    }

    H_P.noalias() = H_k * P_k1;
    S_k_invT = S_k_ldlt.solve(H_P);
    K_k = S_k_invT.transpose();

    // Update estimation and covariance
    x_k1.noalias() = x_k1 + K_k * y_res;
    K_H_P_k.noalias() = K_k * H_k * P_k1;
    P_k1 -= K_H_P_k;

    // Send estimates to go1State object
    state.root_pos_est = x_k1.segment<3>(0);
    state.root_lin_vel_est = x_k1.segment<3>(3);
    state.root_lin_acc_est = state.root_lin_acc_meas;
    Eigen::Quaterniond temp_q(x_k1(6), x_k1(7), x_k1(8), x_k1(9));
    state.root_quat_est = temp_q.normalized();
    state.root_rpy_est = quat2Euler(state.root_quat_est);
    state.root_ang_vel_est = state.root_ang_vel_meas;

    // Push back
    x_k = x_k1;
    P_k = P_k1;
}
*/

// Analytical EKF state estimation
void ExtendedKF::estimateState(go1State& state) {
    // Predict next state
    x_k1 = fState(x_k, state.root_lin_acc_meas, state.root_ang_vel_meas);
    
    // Pull out measurement and useful rotation variables
    foot_pos << state.foot_pos.col(0), 
                state.foot_pos.col(1), 
                state.foot_pos.col(2), 
                state.foot_pos.col(3);

    Eigen::Matrix3d C_kT_plus = quat2RotM(x_k.segment<4>(6)).transpose(); // make note this rotation should be from x_k, not x_k1
    Eigen::Matrix3d C_k_minus = quat2RotM(x_k1.segment<4>(6)); // make note this rotation should be from x_k1, not x_k

    // Update covariances based on foot contact (fix this later)
    double trust;
    for (int i = 0; i < NUM_LEG; i++) {
        trust = state.contacts[i] ? 1.0 : 1e6;
        Q_k_man.block<3, 3>(9 + i*3, 9 + i*3) = trust * 2.0 * eye3;
        R_k.block<3, 3>(i*3, i*3) = trust * 0.001 * eye3;
    }

    // Find model Jacobian analytically (w/o bias states)
    F_k_man.setIdentity();
    F_k_man.block<3, 3>(0, 3) = DT_CTRL * eye3;
    Eigen::Matrix3d skew_acc = skew(state.root_lin_acc_meas);
    Eigen::Matrix3d C_kT_plus_f = -DT_CTRL * C_kT_plus * skew_acc;
    F_k_man.block<3, 3>(0, 6) = 0.5 * DT_CTRL * C_kT_plus_f;
    F_k_man.block<3, 3>(3, 6) = C_kT_plus_f;
    F_k_man.block<3, 3>(6, 6) = computeGamma0(state.root_ang_vel_meas).transpose();

    // Find measurement Jacobian analytically (w/o bias states)
    for (int i = 0; i < NUM_LEG; i++) {
        H_k_man.block<3, 3>(i*3, 0) = -C_k_minus;
        H_k_man.block<3, 3>(i*3, 6) = skew(C_k_minus * (x_k1.segment<3>(10 + i*3) - x_k1.segment<3>(0)));
        H_k_man.block<3, 3>(i*3, 9 + i*3) = C_k_minus;
    }

    // Check residual for calculating optimal Kalman gain
    P_k1_man = F_k_man * P_k_man * F_k_man.transpose() + Q_k_man;
    y_res = foot_pos - hState(x_k1);
    S_k = H_k_man * P_k1_man * H_k_man.transpose() + R_k;
    S_k = (S_k + S_k.transpose()) / 2.0;
    K_k_man = P_k1_man * H_k_man.transpose() * S_k.inverse();

    // Update estimation and covariance
    P_k1_man -= K_k_man * H_k_man * P_k1_man;
    delta_x_k1_man = K_k_man * y_res;
    
    // Bring back to full-sized state vector after update (rpy -> quat)
    x_k1.segment<3>(0) += delta_x_k1_man.segment<3>(0);
    x_k1.segment<3>(3) += delta_x_k1_man.segment<3>(3);
    x_k1.segment<12>(10) += delta_x_k1_man.segment<12>(9);

    Eigen::Quaterniond dq;
    double delta_phi = delta_x_k1_man.segment<3>(6).norm();
    if (delta_phi < 1e-8) {
        dq = Eigen::Quaterniond(1, 0.5*delta_x_k1_man(6), 0.5*delta_x_k1_man(7), 0.5*delta_x_k1_man(8));
    } else {
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(delta_phi, delta_x_k1_man.segment<3>(6).normalized()));
    }

    Eigen::Quaterniond q_k = Eigen::Quaterniond(x_k1(6), x_k1(7), x_k1(8), x_k1(9));
    Eigen::Quaterniond q_k1 = q_k * dq;

    // Send estimates to go1State object
    state.root_pos_est = x_k1.segment<3>(0);
    state.root_lin_vel_est = x_k1.segment<3>(3);
    state.root_lin_acc_est = state.root_lin_acc_meas;
    state.root_quat_est = q_k1.normalized();
    state.root_rpy_est = quat2Euler(state.root_quat_est);
    state.root_ang_vel_est = state.root_ang_vel_meas;

    // Push back
    x_k = x_k1;
    P_k_man = P_k1_man;
}