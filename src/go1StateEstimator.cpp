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
    x_k1.setZero(); x_k1_getter.setZero();
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
    x_k1_getter = x_k1; // Store x_k1 before the Kalman update
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

MIT_TwoStageKF::MIT_TwoStageKF() {
/*
    MIT_TwoStageKF executes the two-state Kalman filter method by first
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
    x_k1.setZero(); x_k1_getter.setZero();
    u_k.setZero();
    z_k.setZero();
    y_res.setZero();
    P_k1.setZero();
    Q_k.setZero();
    R_k.setZero();
    S_k.setZero();
    K_k.setZero();

    // Initialize covariances
    Q_k.setIdentity();
    Q_k.block<3, 3>(0, 0) = DT_CTRL * eye3 / 20.0;
    Q_k.block<3, 3>(3, 3) = DT_CTRL * 9.81 * eye3 / 20.0;
    Q_k.block<12, 12>(6, 6) = DT_CTRL * Eigen::Matrix<double, 12, 12>::Identity();

    R_k.setIdentity();
    // R_k *= 0.001; // trying new idea of always trusting the measurements consistently

    P_k.setIdentity();
    P_k *= 0.01;

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
}

void MIT_TwoStageKF::collectInitialState(const go1State& state) {
    x_k << state.root_pos,
            state.root_lin_vel,
            state.foot_pos_abs.col(0),
            state.foot_pos_abs.col(1),
            state.foot_pos_abs.col(2),
            state.foot_pos_abs.col(3);
}

void MIT_TwoStageKF::setFootHeightResidual(double h) {
    for (int i = 0; i < NUM_LEG; i++) {
        z_k(NUM_LEG*6 + i) = h;
    }
}

void MIT_TwoStageKF::estimateState(go1State& state) {
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

    // Update covariances based on foot contact
    double trust;
    for (int i = 0; i < NUM_LEG; i++) {
        // trust = state.contacts[i] ? 1.0 : 1e6;
        trust = (state.est_contacts[i] > state.thresh) ? 1.0 : 1e6;
        Q_k.block<3, 3>(6 + i*3, 6 + i*3) = trust * DT_CTRL * 0.06 * eye3;

        // New idea: why is R being affected by contact if FK for rigid body is accurate? So keep R constant?
        R_k.block<3, 3>(i*3, i*3) = trust * 0.002 * eye3;
        R_k.block<3, 3>(NUM_LEG*3 + i*3, NUM_LEG*3 + i*3) = trust * 0.5 * eye3; // trust * 2.0 * eye3;
        R_k(NUM_LEG*3 + i*3 + 2, NUM_LEG*3 + i*3 + 2) = trust * 0.03; // z velocity tuning
        R_k(NUM_LEG*6 + i, NUM_LEG*6 + i) = trust * 0.01;
    }

    // Predict next state
    x_k1 = F_k * x_k + B_k * u_k;
    x_k1_getter = x_k1; // Store x_k1 before the Kalman update
    P_k1 = F_k * P_k * F_k.transpose() + Q_k;

    // Find current measurement
    for (int i = 0; i < NUM_LEG; i++) {
        z_k.block<3, 1>(i*3, 0) = state.foot_pos_world_rot.col(i);
        Eigen::Vector3d v_fut = rootRotEst * (state.foot_pos.col(i) - state.foot_pos_old.col(i)) / DT_CTRL 
                                + rootRotEst * skew(state.root_ang_vel_meas) * state.foot_pos.col(i);
        z_k.block<3, 1>(NUM_LEG*3 + i*3, 0) = v_fut;
        z_k(NUM_LEG*6 + i) = 0.02; // removes z-height offset for foot position
    }

    // Check residual for calculating optimal Kalman gain
    y_res = z_k - H_k * x_k1;
    S_k = H_k * P_k1 * H_k.transpose() + R_k;
    S_k = (S_k + S_k.transpose()) / 2.0; // reinforces symmetry
    K_k = P_k1 * H_k.transpose() * S_k.inverse();

    // Update estimation and covariance
    x_k1 += K_k * y_res;
    P_k1 -= K_k * H_k * P_k1;
    P_k1 = (P_k1 + P_k1.transpose()) / 2.0; // reinforces symmetry

    // Optional adjustment, not sure why it's used (prevent state covariance collapse?)
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

ETHZ_EKF_man::ETHZ_EKF_man() {
/*
    ETHZ_EKF_man executes the extended Kalman filter by linearizing the
    nonlinear dynamics and relationship between position and orientation by
    finding the Jacobian of the continuous state dynamics at each
    state according to fState_ETHZ & hState_ETHZ in the header file. Should be
    more accurate than all the linear Kalman filters, but not perfect.
    Based on the ETH Zurich paper showed to me by Zijian & Falak.

    x_k: [p, v, q, p_FR, p_FL, p_RR, p_RL]^T, q = root_quat (start using this or convert from)
    u_k: N/A
    z_k: [p_FR - p, p_FL - p, p_RR - p, p_RL - p]^T
*/
    eye3.setIdentity();

    // Clear internal variables
    x_k.setZero();
    x_k1.setZero(); delta_x_k1.setZero(); x_k1_getter.setZero();
    z_k.setZero();
    y_res.setZero();
    S_k.setZero();
    F_k.setZero();
    H_k.setZero();
    P_k1.setZero();
    K_k.setZero();

    ////////////////////////////
    // Initialize covariances //
    ////////////////////////////

    P_k.setIdentity();
    P_k *= 100.0;

    Q_k.setIdentity();
    R_k.setIdentity();

    // // Best position Q_k gains (hand-made)
    // Q_k.block<3, 3>(0, 0) = DT_CTRL * eye3 / 20.0;
    // Q_k.block<3, 3>(3, 3) = DT_CTRL * 9.81 * eye3 / 20.0;
    // Q_k.block<3, 3>(3, 3) *= 0.01;
    // R_k *= 0.001; // comment out to match Zijian's gain

    // // Best orientation Q_k gains (hand-made)
    // Q_k *= 0.01;
    // R_k *= 0.001; // comment out to match Zijian's gain

    // Best overall gains so far (following paper structure & through MATLAB)

    // // Using old Gamma_0 version
    // q_f = 9.03581;
    // q_w = 0.00834;
    // q_p = 0.00061;
    // r_meas = 0.00105;

    // Using new Gamma_0 version
    q_f = 5.47242;
    q_w = 0.00048;
    q_p = 0.00319;
    r_meas = 0.01190;

    Q_k.block<3, 3>(0, 0) = std::pow(DT_CTRL, 3) * q_f / 3.0 * eye3;
    Q_k.block<3, 3>(0, 3) = std::pow(DT_CTRL, 2) * q_f / 2.0 * eye3;
    Q_k.block<3, 3>(3, 0) = std::pow(DT_CTRL, 2) * q_f / 2.0 * eye3;
    Q_k.block<3, 3>(3, 3) = DT_CTRL * q_f * eye3;
    Q_k.block<3, 3>(6, 6) = DT_CTRL * q_w * eye3;
    R_k *= r_meas;

}

void ETHZ_EKF_man::collectInitialState(const go1State& state) {
    x_k << state.root_pos,
            state.root_lin_vel,
            state.root_quat.w(), state.root_quat.x(), state.root_quat.y(), state.root_quat.z(),
            state.foot_pos_abs.col(0),
            state.foot_pos_abs.col(1),
            state.foot_pos_abs.col(2),
            state.foot_pos_abs.col(3);

    C_k = quat2RotM(x_k.segment<4>(6));
}

void ETHZ_EKF_man::estimateState(go1State& state) {
    // Predict next state
    C_k = quat2RotM(x_k.segment<4>(6));
    x_k1 = fState_ETHZ(x_k, state.root_lin_acc_meas, state.root_ang_vel_meas, C_k);
    x_k1_getter = x_k1; // Store x_k1 before the Kalman update
    
    // Pull out measurement and useful rotation variables
    z_k << state.foot_pos.col(0), 
            state.foot_pos.col(1), 
            state.foot_pos.col(2), 
            state.foot_pos.col(3);

    Eigen::Matrix3d C_kT_plus = C_k.transpose(); // make note this rotation should be from x_k, not x_k1
    Eigen::Matrix3d C_k_minus = quat2RotM(x_k1.segment<4>(6)); // make note this rotation should be from x_k1, not x_k

    // Update covariances based on foot contact
    double trust;
    for (int i = 0; i < NUM_LEG; i++) {
        trust = state.contacts[i] ? 1.0 : 1e6;
        // // Best position gains (hand-made)
        // Q_k.block<3, 3>(9 + i*3, 9 + i*3) = trust * DT_CTRL * 0.06 * eye3;
        // R_k.block<3, 3>(i*3, i*3) = trust * 0.002 * eye3;

        // // Best orientation gains (hand-made)
        // Q_k.block<3, 3>(9 + i*3, 9 + i*3) = trust * DT_CTRL * 2.0 * eye3;
        // R_k.block<3, 3>(i*3, i*3) = trust * 0.001 * eye3; // best orientation R_k gains
        // Q_k.block<3, 3>(9 + i*3, 9 + i*3) = trust * eye3; // matching Zijian's gains
        
        // Best overall gains so far (following paper structure & through MATLAB)
        Q_k.block<3, 3>(9 + i*3, 9 + i*3) = DT_CTRL * C_kT_plus * q_p * eye3 * C_kT_plus.transpose();        
    }

    // Find model Jacobian analytically (w/o bias states)
    F_k.setIdentity();
    F_k.block<3, 3>(0, 3) = DT_CTRL * eye3;
    Eigen::Matrix3d skew_acc = skew(state.root_lin_acc_meas);
    Eigen::Matrix3d C_kT_plus_f = -DT_CTRL * C_kT_plus * skew_acc;
    F_k.block<3, 3>(0, 6) = 0.5 * DT_CTRL * C_kT_plus_f;
    F_k.block<3, 3>(3, 6) = C_kT_plus_f;
    F_k.block<3, 3>(6, 6) = computeGamma0(state.root_ang_vel_meas).transpose();

    // Find measurement Jacobian analytically (w/o bias states)
    for (int i = 0; i < NUM_LEG; i++) {
        H_k.block<3, 3>(i*3, 0) = -C_k_minus;
        H_k.block<3, 3>(i*3, 6) = skew(C_k_minus * (x_k1.segment<3>(10 + i*3) - x_k1.segment<3>(0)));
        H_k.block<3, 3>(i*3, 9 + i*3) = C_k_minus;
    }

    // Check residual for calculating optimal Kalman gain
    P_k1 = F_k * P_k * F_k.transpose() + Q_k;
    y_res = z_k - hState_ETHZ(x_k1, C_k);
    S_k = H_k * P_k1 * H_k.transpose() + R_k;
    S_k = (S_k + S_k.transpose()) / 2.0;
    K_k = P_k1 * H_k.transpose() * S_k.inverse();

    // Update estimation and covariance
    P_k1 -= K_k * H_k * P_k1;
    delta_x_k1 = K_k * y_res;
    
    // Bring back to full-sized state vector after update (rpy -> quat)
    x_k1.segment<3>(0) += delta_x_k1.segment<3>(0);
    x_k1.segment<3>(3) += delta_x_k1.segment<3>(3);
    x_k1.segment<12>(10) += delta_x_k1.segment<12>(9);

    Eigen::Quaterniond dq;
    double delta_phi = delta_x_k1.segment<3>(6).norm();
    if (delta_phi < 1e-8) {
        dq = Eigen::Quaterniond(1, 0.5*delta_x_k1(6), 0.5*delta_x_k1(7), 0.5*delta_x_k1(8));
    } else {
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(delta_phi, delta_x_k1.segment<3>(6).normalized()));
    }

    Eigen::Quaterniond q_k = Eigen::Quaterniond(x_k1(6), x_k1(7), x_k1(8), x_k1(9));
    Eigen::Quaterniond q_k1 = q_k * dq;
    x_k1.segment<4>(6) << q_k1.w(), q_k1.x(), q_k1.y(), q_k1.z();

    // Send estimates to go1State object
    state.root_pos_est = x_k1.segment<3>(0);
    state.root_lin_vel_est = x_k1.segment<3>(3);
    state.root_lin_acc_est = state.root_lin_acc_meas;
    state.root_quat_est = q_k1.normalized();
    state.root_rpy_est = quat2Euler(state.root_quat_est);
    state.root_ang_vel_est = state.root_ang_vel_meas;

    // Push back
    x_k = x_k1;
    P_k = P_k1;
}

/////////////////////////////////////////////

ETHZ_EKF_num::ETHZ_EKF_num() {
/*
    ETHZ_EKF executes the extended Kalman filter by linearizing the
    nonlinear dynamics and relationship between position and orientation by
    finding the Jacobian of the continuous state dynamics at each
    state according to fState_ETHZ & hState_ETHZ in the header file. Should be
    more accurate than all the linear Kalman filters, but not perfect.
    Based on the ETH Zurich paper showed to me by Zijian & Falak.

    x_k: [p, v, q, p_FR, p_FL, p_RR, p_RL]^T, q = root_quat (start using this or convert from)
    u_k: N/A
    z_k: [p_FR - p, p_FL - p, p_RR - p, p_RL - p]^T
*/
    eye3.setIdentity();

    // Clear internal variables
    x_k.setZero();
    x_k1.setZero(); x_k1_getter.setZero();
    z_k.setZero();
    y_res.setZero();
    S_k.setZero();
    F_k.setZero();
    H_k.setZero(); H_P_trans.setZero(); K_k_trans.setZero();
    P_k1.setZero();
    K_k.setZero();

    // Initialize covariances
    Q_k.setIdentity();
    Q_k *= 0.01;

    Eigen::Matrix<double, 12, 1> r_diag;
    double r_x = 10;
    double r_y = 5;
    // r_z values in sim are in range [0.175, 0.55] for walking around on flat ground or all hardware cases
    // r_z values in hardware are in range [xx, yy] for walking around on flat ground or all hardware cases
    double r_z = 0.55; // HARDWARE VALUES: 0.005 is okay
    r_diag << r_x, r_y, r_z, r_x, r_y, r_z, r_x, r_y, r_z;
    R_k = r_diag.asDiagonal();

    P_k.setIdentity();
    P_k *= 100.0;
}

void ETHZ_EKF_num::collectInitialState(const go1State& state) {
    x_k << state.root_pos,
            state.root_lin_vel,
            state.root_quat.w(), state.root_quat.x(), state.root_quat.y(), state.root_quat.z(),
            state.foot_pos_abs.col(0),
            state.foot_pos_abs.col(1),
            state.foot_pos_abs.col(2),
            state.foot_pos_abs.col(3);

    C_k = quat2RotM(x_k.segment<4>(6));
}

void ETHZ_EKF_num::estimateState(go1State& state) {
    // Predict next state
    C_k = quat2RotM(x_k.segment<4>(6));
    x_k1 = fState_ETHZ(x_k, state.root_lin_acc_meas, state.root_ang_vel_meas, C_k);
    x_k1_getter = x_k1; // Store x_k1 before the Kalman update
    
    // Pull out measurement
    for (int i = 0; i < NUM_LEG; i++) {
        z_k.block<3, 1>(i*3, 0) = state.foot_pos.col(i);
    }

    // Update covariances based on foot contact
    double trust;
    for (int i = 0; i < NUM_LEG; i++) {
        trust = (state.est_contacts[i] > state.thresh) ? 0.01 : 1e10;
        Q_k.block<3, 3>(10 + i*3, 10 + i*3) = trust * eye3;
    }

    // // Reduce velocity estimate drift on hardware
    // double vel_est_cov = 1.0;
    // if (state.thresh == UNITREE_SDK_CONTACT_THRESH && Q_k(3, 3) != vel_est_cov) {
    //     Q_k.block<3, 3>(3, 3) = vel_est_cov * eye3;
    // }

    // Find process and measurement Jacobians numerically (w/o bias states)
    F_k = numericalJacobianFixedSize<22, 22>(fState_ETHZ, x_k, 1e-6, false, state.root_lin_acc_meas, state.root_ang_vel_meas, C_k);
    H_k = numericalJacobianFixedSize<12, 22>(hState_ETHZ, x_k1, 1e-6, false, C_k);

    // Check residual for calculating optimal Kalman gain
    P_k1 = F_k * P_k * F_k.transpose() + Q_k;
    y_res = z_k - hState_ETHZ(x_k1, C_k);
    S_k = H_k * P_k1 * H_k.transpose() + R_k;
    S_k = (S_k + S_k.transpose()) / 2.0;
    
    // K_k = P_k1 * H_k.transpose() * S_k.inverse();
    H_P_trans = H_k * P_k1.transpose();
    K_k_trans = S_k.ldlt().solve(H_P_trans);
    K_k = K_k_trans.transpose();

    // Update estimation and covariance
    x_k1 += K_k * y_res;
    Eigen::Quaterniond new_quat = Eigen::Quaterniond(x_k1(6), x_k1(7), x_k1(8), x_k1(9));
    new_quat.normalize();
    P_k1 -= K_k * H_k * P_k1;

    // Send estimates to go1State object
    state.root_pos_est = x_k1.segment<3>(0);
    state.root_lin_vel_est = x_k1.segment<3>(3);
    state.root_lin_acc_est = state.root_lin_acc_meas;
    state.root_quat_est = new_quat;
    state.root_rpy_est = quat2Euler(state.root_quat_est);
    state.root_ang_vel_est = state.root_ang_vel_meas;

    // Push back
    x_k = x_k1;
    P_k = P_k1;
}