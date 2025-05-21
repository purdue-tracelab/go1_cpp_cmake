#include "go1_cpp_cmake/go1State.h"

/*
    go1State.cpp handles the swing leg PD control, and stores all the
    necessary information for all other related functions in the go1MPC
    and go1StateEstimator classes. It relies on the lowLevelDataInterface
    struct to access data from the MuJoCo simulation or UNITREE_LEGGED_SDK
    hardware data.
*/

go1State::go1State() {
/*
    Sets the average lumped inertia to macro values defined in go1Params.h,
    sets default foot positions to xy positions of the hips and the walk
    height negated, and then resets the go1State object to free up important
    information for swing PD planning and state estimation.
*/
    go1_lumped_inertia << GO1_LUMPED_INERTIA_XX, GO1_LUMPED_INERTIA_XY, GO1_LUMPED_INERTIA_XZ,
                            GO1_LUMPED_INERTIA_XY, GO1_LUMPED_INERTIA_YY, GO1_LUMPED_INERTIA_YZ,
                            GO1_LUMPED_INERTIA_XZ, GO1_LUMPED_INERTIA_YZ, GO1_LUMPED_INERTIA_ZZ;

    default_foot_pos << 0.1881, 0.1881, -0.1881, -0.1881,
                        -0.12675, 0.12675, -0.12675, 0.12675,
                        -WALK_HEIGHT, -WALK_HEIGHT, -WALK_HEIGHT, -WALK_HEIGHT;

    joint_torques_limits << TORQUE_MAX_HIP, TORQUE_MAX_THIGH, TORQUE_MAX_CALF,
                            TORQUE_MAX_HIP, TORQUE_MAX_THIGH, TORQUE_MAX_CALF,
                            TORQUE_MAX_HIP, TORQUE_MAX_THIGH, TORQUE_MAX_CALF,
                            TORQUE_MAX_HIP, TORQUE_MAX_THIGH, TORQUE_MAX_CALF;

    resetState();
}

void go1State::resetState() {
/*
    Sets all the positions/velocities to zero, all the rotations to identity, 
    all the forces to zero, and marks all foot contacts as true (Go1 is likely
    laying on the ground, so highly probable all the feet are touching the floor).
*/
    // Root "truth" variables
    root_pos.setZero();
    root_pos_d.setZero();
    root_lin_vel.setZero();
    root_lin_vel_d.setZero();
    root_lin_acc << 0, 0, 9.81;
    root_lin_acc_meas.setZero();
    root_ang_vel_meas.setZero();
    root_rpy.setZero();
    root_rpy_d.setZero();
    root_ang_vel.setZero();
    root_ang_vel_d.setZero();

    // Root "estimation" variables
    root_pos_est.setZero();
    root_lin_vel_est.setZero();
    root_lin_acc_est.setZero();
    root_quat_est = Eigen::Quaterniond::Identity();
    root_ang_vel_est.setZero();

    // Foot tracking variables
    // VERY IMPORTANT FOOT ORDER: FR, FL, RR, RL
    foot_pos_world_rot.setZero();
    foot_pos_old.setZero();
    foot_pos_abs.setZero();
    foot_pos_abs_est.setZero();
    foot_pos_liftoff.setZero();
    foot_pos_d.setZero();
    foot_vel_d.setZero();

    // Foot control variables
    contactJacobian.setZero();
    foot_forces_grf.setZero();
    foot_forces_grf_stacked.setZero();
    foot_forces_swing.setZero();
    joint_torques_swing.setZero();
    joint_torques.setZero();
    joint_torques_stacked.setZero();
    joint_pos_init.setZero();
    joint_pos.setZero();
    joint_pos_d.setZero();
    joint_vel.setZero();
    joint_vel_d.setZero();

    // Movement mode trackers
    walking_mode = false;
    squat_prog = 0.0;
    squat_flag = true;
    thresh = MUJOCO_CONTACT_THRESH; // by default, use MuJoCo threshold; change with FSM initialization
    swing_phase = 0; // ranges from 0 to SWING_PHASE_MAX
    foot_deltaX = 0;
    foot_deltaY = 0;

    std::fill(std::begin(contacts), std::end(contacts), true);
    std::fill(std::begin(contacts_old), std::end(contacts_old), true);
    est_contacts.setZero();
    init = true;
}

void go1State::updateLocomotionPlan() {
/*
    Steps forward the locomotion plan for the Go1 robot if in walking mode.
    Uses the state update from the pullSensorData to calculate foot positions
    and contact Jacobians, as well as update the general walking plan.
*/
    // Calculate foot position & Jacobian after state estimation
    const auto& root_rpy_ctrl = USE_EST_FOR_CONTROL ? root_rpy_est : root_rpy;
    const auto& root_pos_ctrl = USE_EST_FOR_CONTROL ? root_pos_est : root_pos;
    Eigen::Matrix3d rootRotMat = rotZ(root_rpy_ctrl(2))*rotY(root_rpy_ctrl(1))*rotX(root_rpy_ctrl(0));

    foot_pos_world_rot = go1BaseFrameWorldRotFwdKin(joint_pos, root_rpy_ctrl); // implemented go1FwdKin using Muqun and Leo's work
    foot_pos_abs = foot_pos_world_rot.colwise() + root_pos_ctrl;
    // foot_pos_old = foot_pos; // incorrect double calculation
    foot_pos = rootRotMat.transpose() * foot_pos_world_rot;
    contactJacobian = go1WorldFrameContactJacobian(joint_pos, root_rpy_ctrl);

    // Add early contacts based on sensor measurements
    contacts_old = contacts;

    // Update walking plan if walking_mode is requested
    if (walking_mode) {
        // Determine foot contacts based on swing_phase
        if (swing_phase >= 0 && swing_phase <= SWING_PHASE_MAX/2) {
            contacts[0] = true;  // FR stance
            contacts[1] = false;  // FL swing
            contacts[2] = false;  // RR swing
            contacts[3] = true;  // RL stance

        } else if (swing_phase > SWING_PHASE_MAX/2 && swing_phase <= SWING_PHASE_MAX) {
            contacts[0] = false;  // FR swing
            contacts[1] = true;  // FL stance
            contacts[2] = true;  // RR stance
            contacts[3] = false;  // RL swing
        }

        if (swing_phase == 0 || swing_phase == SWING_PHASE_MAX + 1) {
            swing_phase = 0;

            contacts[0] = true;  // FR stance
            contacts[1] = false;  // FL swing
            contacts[2] = false;  // RR swing
            contacts[3] = true;  // RL stance

            foot_forces_swing.setZero();
            joint_pos_d.setZero();
            joint_vel_d.setZero();
            joint_torques_swing.setZero();
            joint_torques.setZero();
            foot_pos_liftoff.setZero();
        
            for (int i = 0; i < NUM_LEG; i++) {
                if (!contacts[i]) {
                    foot_pos_liftoff.col(i) = foot_pos.col(i);
                }
            }

        } else if (swing_phase == SWING_PHASE_MAX/2 + 1) { // swap swing legs and contact legs
            contacts[0] = false;  // FR swing
            contacts[1] = true;  // FL stance
            contacts[2] = true;  // RR stance
            contacts[3] = false;  // RL swing

            foot_forces_swing.setZero();
            joint_pos_d.setZero();
            joint_vel_d.setZero();
            joint_torques_swing.setZero();
            joint_torques.setZero();
            foot_pos_liftoff.setZero();
        
            for (int i = 0; i < NUM_LEG; i++) {
                if (!contacts[i]) {
                    foot_pos_liftoff.col(i) = foot_pos.col(i);
                }
            }
        }

        // Pick planner based on PLANNER_SELECT
        switch (PLANNER_SELECT) {
            case 0: raibertHeuristic(false); break;
            case 1: raibertHeuristic(); break;
            case 2: amirHLIP(); break;
            default: 
                std::cerr << "Error: Invalid PLANNER_SELECT value, check go1Params.h" << std::endl; 
                return;
        }

        // Find desired foot positions and velocities based on swing_phase
        Eigen::Vector3d futPosRef;
        Eigen::Vector3d futVelRef;

        for (int i = 0; i < NUM_LEG; i++) {
            if (SWING_TRAJ_SELECT == 0) {
                futPosRef = bezierPos(i);
                futVelRef = bezierVel(i);
                
            } else if (SWING_TRAJ_SELECT == 1) {
                futPosRef = sinusoidalPos(i);
                futVelRef = sinusoidalVel(i);
            }

            if (!contacts[i]) { // only relative frame tracking for swing PD (use default foot pos?)
                foot_pos_d.col(i) = futPosRef;
                foot_vel_d.col(i) = futVelRef;

                swingPD(i, foot_pos_d.col(i), foot_vel_d.col(i));
            }
        }

        if (!init) {
            swing_phase++;
        }
    }

}

void go1State::convertForcesToTorques() {
/*
    Converts all the foot forces from MPC and swing leg PD to torques for
    the lowest level of control, our joint motor actuators. Based on the
    joint positions relayed to it by MuJoCo simulation.
*/
    // Convert stance leg forces to torques
    foot_forces_grf_stacked << foot_forces_grf.col(0), // FR
                                foot_forces_grf.col(1), // FL
                                foot_forces_grf.col(2), // RR
                                foot_forces_grf.col(3); // RL

    Eigen::Matrix<double, 18, 1> tau_grf_stacked = contactJacobian.transpose() * foot_forces_grf_stacked;
    joint_torques_stacked = tau_grf_stacked.tail<12>();

    // Convert swing leg forces to torques if using Cartesian space PD
    if (SWING_PD_SELECT == 0) {
        Eigen::Vector3d root_rpy_ctrl = USE_EST_FOR_CONTROL ? root_rpy_est : root_rpy;
        Eigen::Matrix3d rootRotMat = rotZ(root_rpy_ctrl(2))*rotY(root_rpy_ctrl(1))*rotX(root_rpy_ctrl(0));
        Eigen::Matrix3d rootRotMatT = rootRotMat.transpose();

        for (int i = 0; i < NUM_LEG; i++) {
            Eigen::Matrix3d contactJacobian_i = contactJacobian.block<3, 3>(i*3, 6 + i*3).transpose();
            Eigen::Matrix3d legJacobian_i = rootRotMatT * contactJacobian_i;
            joint_torques_swing.col(i) = legJacobian_i * foot_forces_swing.col(i);
        }
    }

    // Combine stance and swing leg torques
    for (int i = 0; i < NUM_LEG; i++) {
        joint_torques_stacked.block<3, 1>(i*3, 0) += joint_torques_swing.col(i);
    }

    // Apply torque limits (safety measure)
    for (int i = 0; i < 3*NUM_LEG; i++) {
        if (joint_torques_stacked(i, 0) > joint_torques_limits(i, 0)) {
            joint_torques_stacked(i, 0) = joint_torques_limits(i, 0);
        } else if (joint_torques_stacked(i, 0) < -joint_torques_limits(i, 0)) {
            joint_torques_stacked(i, 0) = -joint_torques_limits(i, 0);
        }
    }

    joint_torques << joint_torques_stacked.block<3, 1>(0, 0), // FR
                    joint_torques_stacked.block<3, 1>(3, 0), // FL
                    joint_torques_stacked.block<3, 1>(6, 0), // RR
                    joint_torques_stacked.block<3, 1>(9, 0); // RL

    // Zero out swing leg torques for next iteration
    joint_torques_swing.setZero();
}

Eigen::Vector3d go1State::bezierPos(int leg_idx) {
/*
    Computes the reference position along a Bezier curve between zero pos
    and a foot position ahead by foot_deltaX and foot_deltaY. The z height
    of the foot is purely based on swing_phase.
*/
    double phi = swing_phase % (SWING_PHASE_MAX/2);
    double phi_max = SWING_PHASE_MAX/2;

    Eigen::Vector3d p0 = foot_pos_liftoff.col(leg_idx);
    Eigen::Vector3d p_c1 = p0;
    Eigen::Vector3d p_c2 = default_foot_pos.col(leg_idx) + Eigen::Vector3d(foot_deltaX, foot_deltaY, 0);
    Eigen::Vector3d p2 = p_c2;
    Eigen::Vector3d p1 = (p0 + p2)/2.0 + Eigen::Vector3d(0, 0, STEP_HEIGHT*2);

    double t = phi/phi_max;
    double oneMinusT = 1.0 - t;

    Eigen::Vector3d fut_posDes = std::pow(oneMinusT, 4) * p0
                                + 4 * std::pow(oneMinusT, 3) * t * p_c1
                                + 6 * std::pow(oneMinusT, 2) * std::pow(t, 2) * p1
                                + 4 * oneMinusT * std::pow(t, 3) * p_c2
                                + std::pow(t, 4) * p2;

    return fut_posDes;
}

Eigen::Vector3d go1State::bezierVel(int leg_idx) {
/*
    Computes the reference velocity along a Bezier curve between zero pos
    and a foot position ahead by foot_deltaX and foot_deltaY. Essentially
    the derivative of the above function w.r.t. swing_phase.
*/
    double phi = swing_phase % (SWING_PHASE_MAX/2);
    double phi_max = SWING_PHASE_MAX/2;

    Eigen::Vector3d p0 = foot_pos_liftoff.col(leg_idx);
    Eigen::Vector3d p_c1 = p0;
    Eigen::Vector3d p_c2 = default_foot_pos.col(leg_idx) + Eigen::Vector3d(foot_deltaX, foot_deltaY, 0);
    Eigen::Vector3d p2 = p_c2;
    Eigen::Vector3d p1 = (p0 + p2)/2.0 + Eigen::Vector3d(0, 0, STEP_HEIGHT*2);

    double t = phi/phi_max;
    double oneMinusT = 1.0 - t;

    Eigen::Vector3d fut_velDes = 4 * std::pow(oneMinusT, 3) / phi_max * (p_c1 - p0)
                                + 12 * t * std::pow(oneMinusT, 2) / phi_max * (p1 - p_c1)
                                + 12 * std::pow(t, 2) * oneMinusT / phi_max * (p_c2 - p1)
                                + 4 * std::pow(t, 3) / phi_max * (p2 - p_c2);

    return fut_velDes;
}

Eigen::Vector3d go1State::sinusoidalPos(int leg_idx) {
/*
    Computes the reference position along a sinusoidal curve between the
    initial and final positions of the foot. Meant to mimic Muqun's hardware
    code performance to bridge sim2real gap.
*/
    double phi = swing_phase % (SWING_PHASE_MAX/2);
    double phi_max = SWING_PHASE_MAX/2;
    double phi_pi = 2 * M_PI * phi / phi_max;

    Eigen::Vector3d p0 = foot_pos_liftoff.col(leg_idx);
    Eigen::Vector3d p1 = default_foot_pos.col(leg_idx) + Eigen::Vector3d(foot_deltaX, foot_deltaY, 0);

    double alphaXY = (phi_pi - sin(phi_pi)) / (2 * M_PI);
    double alphaZ = (1 - cos(phi_pi)) / 2;

    Eigen::Vector3d fut_posDes = p0 + alphaXY * (p1 - p0);
    fut_posDes.z() = alphaZ * STEP_HEIGHT;

    return fut_posDes;
}

Eigen::Vector3d go1State::sinusoidalVel(int leg_idx) {
/*
    Computes the reference velocity along a sinusoidal curve between the
    initial and final positions of the foot. Meant to mimic Muqun's hardware
    code performance to bridge sim2real gap.
*/
    double phi = swing_phase % (SWING_PHASE_MAX/2);
    double phi_max = SWING_PHASE_MAX/2;
    double phi_pi = 2 * M_PI * phi / phi_max;
    double t_swing = (phi_max + 1) * DT_CTRL;

    Eigen::Vector3d p0 = foot_pos_liftoff.col(leg_idx);
    Eigen::Vector3d p1 = default_foot_pos.col(leg_idx) + Eigen::Vector3d(foot_deltaX, foot_deltaY, 0);

    double alphaXY = (phi_pi - cos(phi_pi)) / (2 * M_PI * t_swing);
    double alphaZ = M_PI * sin(phi_pi) / t_swing;

    Eigen::Vector3d fut_velDes = alphaXY * (p1 - p0);
    fut_velDes.z() = alphaZ * STEP_HEIGHT;

    return fut_velDes;
}

void go1State::raibertHeuristic(bool withCapturePoint) {
/*
    Calculates foot_deltaX and foot_deltaY by using a simple Raibert
    Heuristic footstep planner. If you also want capture point, use "true"
    as input; if not, leave input blank or use "false".
*/
    Eigen::Matrix3d rootRotMatZ; // assumes small roll + pitch only (should I keep this assumption?)
    Eigen::Vector3d lin_vel_rel;

    if (USE_EST_FOR_CONTROL) {
        rootRotMatZ = rotZ(root_rpy_est(2));
        lin_vel_rel = rootRotMatZ.transpose() * root_lin_vel_est;

    } else {
        rootRotMatZ = rotZ(root_rpy(2));
        lin_vel_rel = rootRotMatZ.transpose() * root_lin_vel;
    }
    
    double delta_x;
    double delta_y;
    double swingTime = (SWING_PHASE_MAX + 1)/2.0;

    if (withCapturePoint) {
        // Step lengths delta_* = sqrt(base_height/gravity)*(base_vel_* - base_vel_*_d) + (swing_duration/2)*base_vel_*_d
        delta_x = std::sqrt(WALK_HEIGHT/9.8) * (lin_vel_rel(0) - root_lin_vel_d(0)) +
                            (swingTime * DT_CTRL) / 2.0 * lin_vel_rel(0);

        delta_y = std::sqrt(WALK_HEIGHT/9.8) * (lin_vel_rel(1) - root_lin_vel_d(1)) +
                            (swingTime * DT_CTRL) / 2.0 * lin_vel_rel(1);

    } else {
        // Step lengths delta_* = (swing_duration/2)*base_vel_*_d
        delta_x = (swingTime * DT_CTRL) / 2.0 * lin_vel_rel(0);
        delta_y = (swingTime * DT_CTRL) / 2.0 * lin_vel_rel(1);
    }
    
    // Step length clipping/saturation
    if (delta_x < -FOOT_DELTA_X_LIMIT) {
        delta_x = -FOOT_DELTA_X_LIMIT;
    }
    if (delta_x > FOOT_DELTA_X_LIMIT) {
        delta_x = FOOT_DELTA_X_LIMIT;
    }
    if (delta_y < -FOOT_DELTA_Y_LIMIT) {
        delta_y = -FOOT_DELTA_Y_LIMIT;
    }
    if (delta_y > FOOT_DELTA_Y_LIMIT) {
        delta_y = FOOT_DELTA_Y_LIMIT;
    }

    // Updating the desired foot position
    foot_deltaX = delta_x;
    foot_deltaY = delta_y;
}

void go1State::amirHLIP() {
/*
    Calculates foot_deltaX and foot_deltaY based on Amir Iqbal's
    HT-LIP footstep planning algorithm, which uses the SRBM's CoM's
    positional offset from the weighted sum of the xy foot positions
    to determine the optimal gains to adjust foot_deltaX and foot_deltaY
    based on a QP formulation of HT-LIP dynamics. Need to ask I-Chia if
    correct or incorrect, may take a while to debug if incorrect.
*/
    // rootRotMatZ and lin_vel_rel assumes small roll + pitch only (should I keep this assumption?)
    Eigen::Matrix3d rootRotMatZ = rotZ(USE_EST_FOR_CONTROL ? root_rpy_est(2) : root_rpy(2));
    Eigen::Vector3d lin_vel_rel = rootRotMatZ.transpose() * (USE_EST_FOR_CONTROL ? root_lin_vel_est : root_lin_vel);

    double swingProg = 0.0;
    stanceFeetSumXY.setZero(); // sum of stance feet xy positions
    int twoFootContact = 0;

    // Swing progress for each swing leg in the gait cycle, calculates xy sum of stance feet
    for (int i = 0; i < NUM_LEG; i++) {
        if (contacts[i]) {
            stanceFeetSumXY += foot_pos_world_rot.block<2, 1>(0, i);
        } else {
            swingProg += double(swing_phase % (SWING_PHASE_MAX/2)) / (SWING_PHASE_MAX/2); // ranges from 0 to 2
            twoFootContact += 1;
        }
    }

    // Computes the centroid of the feet and the offset using a weighted average of the xy foot positions
    x_footprint_default = default_foot_pos.row(0).segment(0, default_foot_pos.cols());
    y_footprint_default = default_foot_pos.row(1).segment(0, default_foot_pos.cols());
    x_footprint_offset = 0.25*x_footprint_default.sum();
    y_footprint_offset = 0.25*y_footprint_default.sum();

    if (twoFootContact == 2) {
        StanceFeetMP = StanceFeetSumXY/2.0; // stance feet midpoint
        StanceFeetMP_prev = StanceFeetMP;
    } else {
        StanceFeetMP = StanceFeetMP_prev;
    }

    rel_error_pos = root_lin_vel_d.segment<2>(0) * DT_CTRL * (1.0 - swingProg)/2.0 - StanceFeetSumXY; // double check the (1 - swingProg) expression
    rel_error_vel = lin_vel_rel.segment<2>(0) - root_lin_vel_d.segment<2>(0);
    u_ref = root_lin_vel_d.segment<2>(0) * DT_CTRL * (SWING_PHASE_MAX + 1) / 2;

    // Finds the effective LIP frequency and adjusts it based on the vertical acceleration
    double a_ver_plus_g = root_lin_acc_meas(2) - 9.81; // isn't used anywhere else, what's the point of this? only comparison?
    double f_M; // effective LIP frequency

    if (root_lin_acc_meas(2) > 0){ // Amir uses world frame acc instead of body frame acc, maybe change?
        // f_M = root_lin_acc(2) / root_pos_d(2);
        f_M = root_lin_acc_meas(2) / WALK_HEIGHT; // either use hard-coded walk height or estimated height
    } 
    if(root_lin_acc_meas(2) > 15.0){
        // f_M = 15.0 / root_pos_d(2);
        f_M = 15.0 / WALK_HEIGHT;
    }
    if(root_lin_acc_meas(2) < 5.0){
        f_M = 25.0;
    }

    // State transition matrix for entire swing duration
    double a1 = cosh(DT_CTRL * (SWING_PHASE_MAX/2) * sqrt(f_M));
    double a2 = sinh(DT_CTRL * (SWING_PHASE_MAX/2) * sqrt(f_M)) / sqrt(f_M);
    double a3 = sinh(DT_CTRL * (SWING_PHASE_MAX/2) * sqrt(f_M)) * sqrt(f_M);
    double a4 = cosh(DT_CTRL * (SWING_PHASE_MAX/2) * sqrt(f_M));
    
    // State transition matrix for remaining swing duration
    double a1r = cosh((1.0 - 0.5*swingProg) * DT_CTRL * (SWING_PHASE_MAX/2) * sqrt(f_M)); // 0.5 because swing_progress_pram is averaged for the nominally 2 swinging legs
    double a2r = sinh((1.0 - 0.5*swingProg) * DT_CTRL * (SWING_PHASE_MAX/2) * sqrt(f_M)) / sqrt(f_M);
    double a3r = sinh((1.0 - 0.5*swingProg) * DT_CTRL * (SWING_PHASE_MAX/2) * sqrt(f_M)) * sqrt(f_M);
    double a4r = cosh((1.0 - 0.5*swingProg) * DT_CTRL * (SWING_PHASE_MAX/2) * sqrt(f_M));

    Phi_rem << a1r, a2r, 
                a3r, a4r; // state transition matrix for remainingTime in swing

    // HLIP state
    X_hlip << -StanceFeetMP(0) - x_footprint_offset, lin_vel_rel(0);
    Y_hlip << -StanceFeetMP(1) - y_footprint_offset, lin_vel_rel(1);

    // Desired HLIP state at end of swing phase
    X_hlip_ref_minus << root_lin_vel_d(0) * DT_CTRL * (SWING_PHASE_MAX/4), root_lin_vel_d(0);
    Y_hlip_ref_minus << root_lin_vel_d(1) * DT_CTRL * (SWING_PHASE_MAX/4), root_lin_vel_d(1);

    // Predicted HLIP state at end of swing phase
    X_hlip_minus = Phi_rem*X_hlip;
    Y_hlip_minus = Phi_rem*Y_hlip;

    // HLIP state errors between predicted and desired states at the end of the swing phase
    X_hlip_error_minus = X_hlip_minus - X_hlip_ref_minus;
    Y_hlip_error_minus = Y_hlip_minus - Y_hlip_ref_minus;

    // HLIP Hessian, gradient, linear constraints, and bounds
    // e << root (<-- incomplete line, double check Amir's HLIP formulation)
    for (int i = 0; i < 4; i++){
        hlip_dense_hessian(i,i) = 2.0 * (a1*a1 + a3*a3);
        hlip_dense_linearConst(i+2,i) = 1.0;
    }

    hlip_gradient.resize(4);
    hlip_gradient << -2*(a1*a1 + a3*a3), -2*(a1*a2 + a3*a4),-2*(a1*a1 + a3*a3), -2*(a1*a2 + a3*a4);

    hlip_dense_linearConst(0,0) = X_hlip_error_minus(0);
    hlip_dense_linearConst(0,1) = X_hlip_error_minus(1);
    hlip_dense_linearConst(1,2) = Y_hlip_error_minus(0);
    hlip_dense_linearConst(1,3) = Y_hlip_error_minus(1);

    hlip_lb.resize(6);
    hlip_ub.resize(6);
    hlip_lb << -FOOT_DELTA_X_LIMIT - u_ref(0), -FOOT_DELTA_Y_LIMIT - u_ref(1), 0.5, 0.0, 0.5, 0.0;
    hlip_ub << FOOT_DELTA_X_LIMIT + u_ref(0), FOOT_DELTA_Y_LIMIT + u_ref(1), 1.5, 0.5, 1.5, 0.5;

    hlip_hessian = hlip_dense_hessian.sparseView();
    hlip_linearConst = hlip_dense_linearConst.sparseView();

    // instantiate HLIP QP solver
    OsqpEigen::Solver solver_hlip;
    solver_hlip.settings()->setVerbosity(false);
    solver_hlip.settings()->setWarmStart(false);
    solver_hlip.data()->setNumberOfVariables(4);
    solver_hlip.data()->setNumberOfConstraints(6);
    solver_hlip.data()->setLinearConstraintsMatrix(hlip_linearConst);
    solver_hlip.data()->setHessianMatrix(hlip_hessian);
    solver_hlip.data()->setGradient(hlip_gradient);
    solver_hlip.data()->setLowerBound(hlip_lb);
    solver_hlip.data()->setUpperBound(hlip_ub);
    solver_hlip.initSolver();
    solver_hlip.solveProblem();

    // Solve the HLIP QP to get HLIP gains
    GainsHLIP = solver_hlip.getSolution(); //4x1
    
    // HLIP based step lengths in xy plane
    double stepLengthX = root_lin_vel_d(0) * DT_CTRL * (SWING_PHASE_MAX/2) + (GainsHLIP.segment<2>(0).dot(X_hlip_error_minus));
    double stepLengthY = root_lin_vel_d(1) * DT_CTRL * (SWING_PHASE_MAX/2) + (GainsHLIP.segment<2>(2).dot(Y_hlip_error_minus));
    
    // Saturation/clipping
    if (stepLengthX < -FOOT_DELTA_X_LIMIT) {
        stepLengthX = -FOOT_DELTA_X_LIMIT;
    }
    if (stepLengthX > FOOT_DELTA_X_LIMIT) {
        stepLengthX = FOOT_DELTA_X_LIMIT;
    }
    if (stepLengthY < -FOOT_DELTA_Y_LIMIT) {
        stepLengthY = -FOOT_DELTA_Y_LIMIT;
    }
    if (stepLengthY > FOOT_DELTA_Y_LIMIT) {
        stepLengthY = FOOT_DELTA_Y_LIMIT;
    }

    foot_deltaX = stepLengthX;
    foot_deltaY = stepLengthY;
}

void go1State::swingPD(int leg_idx, Eigen::Vector3d footPosRef, Eigen::Vector3d footVelRef) {
/*
    Calculates the forces for swing legs using PD control with
    relative foot positions, velocities, and their respective Bezier 
    curve references for smooth and precise swing leg control.
*/
    if (contacts[leg_idx] == false && walking_mode) {
        Eigen::Vector3d foot_posN = foot_pos.col(leg_idx);
        Eigen::Vector3d foot_velN = (foot_pos.col(leg_idx) - foot_pos_old.col(leg_idx))/DT_CTRL;

        if (SWING_PD_SELECT == 0) {
            Eigen::Matrix3d kp = SWING_KP_CART * Eigen::Matrix3d::Identity();
            Eigen::Matrix3d kd = SWING_KD_CART * Eigen::Matrix3d::Identity();

            foot_forces_swing.col(leg_idx) = kp * (footPosRef - foot_posN) + kd * (footVelRef - foot_velN);
        
        } else {
            Eigen::Vector3d rpy_ctrl = USE_EST_FOR_CONTROL ? root_rpy_est : root_rpy;
            Eigen::Matrix3d rootRotMat = rotZ(rpy_ctrl(2)) * rotY(rpy_ctrl(1)) * rotX(rpy_ctrl(0));
            Eigen::Matrix3d rootRotMatT = rootRotMat.transpose();

            ////////////////////////////////////////////////////////////////////////////////////////////////////
            // Double Muqun IK (for reference and current foot positions; not correct method, but works well) //
            ////////////////////////////////////////////////////////////////////////////////////////////////////

            // Eigen::Vector3d jointPosRef = computeFutIK(leg_idx, footPosRef);
            // Eigen::Vector3d jointPosN = computeFutIK(leg_idx, foot_posN);

            // Eigen::Vector3d jointVelRef = rootRotMatT * contactJacobian.block<3, 3>(leg_idx*3, 6 + leg_idx*3) * footVelRef;
            // Eigen::Vector3d jointVelN = rootRotMatT * contactJacobian.block<3, 3>(leg_idx*3, 6 + leg_idx*3) * foot_velN;

            ////////////////////////////////////////////////////////////////////////////////////////////////////
            // Reading current joint pos & Muqun IK for reference joint pos (IK in hip frame, not body frame) //
            ////////////////////////////////////////////////////////////////////////////////////////////////////

            // Eigen::Vector3d jointPosRef = computeFutIK(leg_idx, (footPosRef - Eigen::Vector3d(default_foot_pos(0, leg_idx), 
            //                                                                                     default_foot_pos(1, leg_idx),
            //                                                                                     0)));
            // joint_pos_d.block<3, 1>(0 + 3*leg_idx, 0) = jointPosRef;
            // Eigen::Vector3d jointPosN = joint_pos.block<3, 1>(0 + 3*leg_idx, 0);

            // // Eigen::Vector3d jointVelRef = rootRotMatT * contactJacobian.block<3, 3>(leg_idx*3, 6 + leg_idx*3) * footVelRef;
            // Eigen::Vector3d jointVelRef = go1HipFrameLegJacobian(leg_idx, joint_pos) * footVelRef;
            // joint_vel_d.block<3, 1>(0 + 3*leg_idx, 0) = jointVelRef;
            // Eigen::Vector3d jointVelN = joint_vel.block<3, 1>(0 + 3*leg_idx, 0);

            /////////////////////////////////////////////////////////////////////
            // Reading current joint pos & one-shot IK for reference joint pos //
            /////////////////////////////////////////////////////////////////////

            Eigen::Vector3d dx = footPosRef - foot_pos.col(leg_idx);
            Eigen::Matrix3d legJacobian = rootRotMatT * contactJacobian.block<3, 3>(leg_idx*3, 6 + leg_idx*3);
            Eigen::Vector3d dq = computeNewtonIK(legJacobian, dx);

            Eigen::Vector3d jointPosRef = joint_pos.block<3, 1>(0 + 3*leg_idx, 0) + dq;
            joint_pos_d.block<3, 1>(0 + 3*leg_idx, 0) = jointPosRef;
            Eigen::Vector3d jointPosN = joint_pos.block<3, 1>(0 + 3*leg_idx, 0);

            Eigen::Vector3d jointVelRef = legJacobian * footVelRef; // causes backwards drift?
            // Eigen::Vector3d jointVelRef = dq/DT_CTRL; // better but more jitter?
            joint_vel_d.block<3, 1>(0 + 3*leg_idx, 0) = jointVelRef;
            Eigen::Vector3d jointVelN = joint_vel.block<3, 1>(0 + 3*leg_idx, 0);

            //////////////////////////////////////////////////////////////////////

            Eigen::Matrix3d kp = SWING_KP_JOINT * Eigen::Matrix3d::Identity();
            Eigen::Matrix3d kd = SWING_KD_JOINT * Eigen::Matrix3d::Identity();

            joint_torques_swing.col(leg_idx) = kp * (jointPosRef - jointPosN) + kd * (jointVelRef - jointVelN);

            Eigen::Vector3d gravity_torque_comp(-0.66, -0.37, 0.16);
            if (leg_idx == 1 || leg_idx == 3) {
                gravity_torque_comp(0) *= -1;
            }

            joint_torques_swing.col(leg_idx) += gravity_torque_comp;
        }

    } else if (contacts[leg_idx] == true && walking_mode) { // applying swing PD to stance legs: FORBIDDEN
        std::cerr << "Error: Invalid foot index for swing leg PD! control" << std::endl;
        return;
    }
}

void go1State::jointPD(int joint_idx, double jointPos, double jointVel, bool startup) {
/*
    Calculates joint-level PD control for standing & shutdown.
*/
    double jointInterp;
    double jointTorque;

    joint_vel_d(joint_idx, 0) = 0.0;

    switch(joint_idx % 3) {
        case 0:
            if (startup) {
                jointInterp = (1.0 - squat_prog) * joint_pos_init(joint_idx);
                joint_pos_d(joint_idx, 0) = jointInterp;
                joint_torques(joint_idx % 3, joint_idx / 3) = std::clamp(SQUAT_JOINT_KP * (jointInterp - jointPos) + SQUAT_JOINT_KD * (0 - jointVel), -TORQUE_MAX_HIP, TORQUE_MAX_HIP);
                return;

            } else {
                joint_pos_d(joint_idx, 0) = 0.0;
                joint_torques(joint_idx % 3, joint_idx / 3) = std::clamp(SQUAT_JOINT_KP * (0.0 - jointPos) + SQUAT_JOINT_KD * (0.0 - jointVel), -TORQUE_MAX_HIP, TORQUE_MAX_HIP);
                return;
            }

        case 1:
            if (startup) {
                jointInterp = (1.0 - squat_prog) * joint_pos_init(joint_idx) + squat_prog * THIGH_RAD_STAND;
                joint_pos_d(joint_idx, 0) = jointInterp;
                joint_torques(joint_idx % 3, joint_idx / 3) = std::clamp(SQUAT_JOINT_KP * (jointInterp - jointPos) + SQUAT_JOINT_KD * (0 - jointVel), -TORQUE_MAX_THIGH, TORQUE_MAX_THIGH);
                return;

            } else {
                jointInterp = (1.0 - squat_prog) * joint_pos_init(joint_idx) + squat_prog * THIGH_RAD_STAND;
                joint_pos_d(joint_idx, 0) = jointInterp;
                joint_torques(joint_idx % 3, joint_idx / 3) = std::clamp(SQUAT_JOINT_KP * (jointInterp - jointPos) + SQUAT_JOINT_KD * (0.0 - jointVel), -TORQUE_MAX_THIGH, TORQUE_MAX_THIGH);
                return;
            }
            
        case 2:
            if (startup) {
                jointInterp = (1.0 - squat_prog) * joint_pos_init(joint_idx) + squat_prog * CALF_RAD_STAND;
                joint_pos_d(joint_idx, 0) = jointInterp;
                joint_torques(joint_idx % 3, joint_idx / 3) = std::clamp(SQUAT_JOINT_KP * (jointInterp - jointPos) + SQUAT_JOINT_KD * (0 - jointVel), -TORQUE_MAX_CALF, TORQUE_MAX_CALF);
                return;

            } else {
                jointInterp = (1.0 - squat_prog) * joint_pos_init(joint_idx) + squat_prog * CALF_RAD_STAND;
                joint_pos_d(joint_idx, 0) = jointInterp;
                joint_torques(joint_idx % 3, joint_idx / 3) = std::clamp(SQUAT_JOINT_KP * (jointInterp - jointPos) + SQUAT_JOINT_KD * (0.0 - jointVel), -TORQUE_MAX_CALF, TORQUE_MAX_CALF);
                return;
            }
            
    }
}

bool go1State::isStartupComplete() const {
    return squat_prog >= 1.0;
}

bool go1State::isShutdownComplete() const {
    return squat_prog <= 0.0;
}

void go1State::computeStartupPD() {
/*
    Activate simple joint PD for startup mode in MuJoCo.
*/
    if (squat_prog == 0) {
        joint_pos_init = joint_pos;
    }

    for (int i = 0; i < 3*NUM_LEG; i++) {
        go1State::jointPD(i, joint_pos(i, 0), joint_vel(i, 0));
    }

    if (!isStartupComplete()) squat_prog += 0.002;
}

void go1State::computeShutdownPD() {
/*
    Activate simple joint PD for shutdown mode in MuJoCo.
*/
    for (int i = 0; i < 3*NUM_LEG; i++) {
        go1State::jointPD(i, joint_pos(i, 0), joint_vel(i, 0), false);
    }

    if (!isShutdownComplete()) {
        squat_prog -= 0.002;
    } else {
        joint_pos_init = joint_pos;
    }
}

go1StateSnapshot go1State::getSnapshot() const {
    go1StateSnapshot stateSnap;

    stateSnap.foot_pos_world_rot = foot_pos_world_rot; // overwrite with estimate or new variable?
    stateSnap.go1_lumped_inertia = go1_lumped_inertia;
    stateSnap.root_pos = USE_EST_FOR_CONTROL ? root_pos_est : root_pos;
    stateSnap.root_pos_d = root_pos_d;
    stateSnap.root_rpy = USE_EST_FOR_CONTROL ? root_rpy_est : root_rpy;
    stateSnap.root_rpy_d = root_rpy_d;
    stateSnap.root_lin_vel = USE_EST_FOR_CONTROL ? root_lin_vel_est : root_lin_vel;
    stateSnap.root_lin_vel_d = root_lin_vel_d;
    stateSnap.root_ang_vel = USE_EST_FOR_CONTROL ? root_ang_vel_est : root_ang_vel;
    stateSnap.root_ang_vel_d = root_ang_vel_d;
    stateSnap.walking_mode = walking_mode;
    stateSnap.swing_phase = swing_phase;
    stateSnap.grf_forces.setZero();
    stateSnap.contacts = contacts;

    return stateSnap;
}

void go1State::retrieveGRF(const Eigen::Matrix<double, 3, NUM_LEG> &grf) {
    foot_forces_grf = grf;
}