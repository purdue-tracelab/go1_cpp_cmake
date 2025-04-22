#include "go1_cpp_cmake/go1State.h"

/*
    go1State.cpp handles the information storage between the different
    scripts for storing current information about the Unitree Go1 robot
    (position, orientation, target and current foot positions, foot forces, 
    velocities, accelerations, contacts, etc.). This is the main focus of
    translation into hardware from MuJoCo simulation state update and
    management.
*/

go1State::go1State() {
/*
    Sets the average lumped inertia to macro values defined in go1Params.h,
    and then resets the state.
*/
    go1_lumped_inertia << GO1_LUMPED_INERTIA_XX, GO1_LUMPED_INERTIA_XY, GO1_LUMPED_INERTIA_XZ,
                            GO1_LUMPED_INERTIA_XY, GO1_LUMPED_INERTIA_YY, GO1_LUMPED_INERTIA_YZ,
                            GO1_LUMPED_INERTIA_XZ, GO1_LUMPED_INERTIA_YZ, GO1_LUMPED_INERTIA_ZZ;

    default_foot_pos << 0.1881, 0.1881, -0.1881, -0.1881,
                        -0.12675, 0.12675, -0.12675, 0.12675,
                        -WALK_HEIGHT, -WALK_HEIGHT, -WALK_HEIGHT, -WALK_HEIGHT;

    root_hip_pos_abs << 0.1881, 0.1881, -0.1881, -0.1881,
                    -0.12675, 0.12675, -0.12675, 0.12675,
                    0, 0, 0, 0;

    root_hip_pos = root_hip_pos_abs;

    resetState();
}

void go1State::resetState() {
/*
    Sets all the positions/velocities to zero, all the rotations to identity, 
    all the forces to zero, and marks all foot contacts as true (Go1 is likely
    laying on the ground, so highly probable all the feet are touching the floor).
*/
    root_pos.setZero();
    root_pos_old.setZero();
    root_pos_d.setZero();
    root_pos_est.setZero();
    root_lin_vel.setZero();
    root_lin_vel_d.setZero();
    root_lin_vel_est.setZero();
    root_lin_acc.setZero();
    root_lin_acc_est.setZero();
    root_lin_acc_meas.setZero();

    root_quat = Eigen::Quaterniond::Identity();
    root_quat_old = Eigen::Quaterniond::Identity();
    root_quat_est = Eigen::Quaterniond::Identity();
    root_rpy.setZero();
    root_rpy_old.setZero();
    root_rpy_d.setZero();
    root_ang_vel.setZero();
    root_ang_vel_d.setZero();
    root_ang_vel_est.setZero();
    root_ang_vel_meas.setZero();

    // VERY IMPORTANT FOOT ORDER: FR, FL, RR, RL
    foot_pos.setZero();
    foot_pos_old.setZero();
    foot_pos_abs.setZero();
    foot_pos_abs_est.setZero();
    foot_pos_liftoff.setZero();
    foot_pos_d.setZero();
    foot_vel_d.setZero();

    foot_forces_grf.setZero();
    foot_forces_swing.setZero();
    joint_torques.setZero();
    joint_pos_init.setZero();

    walking_mode = false;
    squat_prog = 0.0;
    swing_phase = 0; // ranges from 0 to SWING_PHASE_MAX
    foot_deltaX = 0;
    foot_deltaY = 0;

    std::fill(std::begin(contacts), std::end(contacts), true);
    init = true;
    
}

void go1State::updateHardwareState(UNITREE_LEGGED_SDK::LowState& state) {
    // Get updated joint position
    // order is FR_0, FR_1, FR_2, FL_0, FL_1, FL_2, RR_0, RR_1, RR_2, RL_0, RL_1, RL_2
    for (int i = 0; i < jointPos.size(); i++) {
        jointPos[i] = state.motorState[i].q;
    }
    
    // Update current and old states of the root and feet
    root_pos_old = root_pos;
    root_lin_vel_old = root_lin_vel;
    root_lin_acc << state.imu.accelerometer[0], state.imu.accelerometer[1], state.imu.accelerometer[2];

    root_rpy_old = root_rpy;
    root_quat_old = root_quat;

    root_ang_vel_old = root_ang_vel;

    foot_pos_old = foot_pos;
    foot_pos = go1FKHardware(jointPos, root_rpy); // implemented go1FK using Muqun and Leo's work

    if (init) {
        root_pos_old = root_pos;
        root_rpy_old = root_rpy;
        foot_pos_old = foot_pos;
    }
    
    for (int i = 0; i < NUM_LEG; i++) {
        foot_pos_abs.block<3, 1>(0, i) = foot_pos.block<3, 1>(0, i) + root_pos;
    }

    Eigen::Matrix3d rootRotMat = rotZ(root_rpy(2))*rotY(root_rpy(1))*rotX(root_rpy(0));
    Eigen::Matrix3d hipRotMat = rotZ(root_rpy(2))*rotY(root_rpy(1));

    root_hip_pos.block<3, 1>(0, 0) = rootRotMat*Eigen::Vector3d(DELTA_X_HIP, -DELTA_Y_HIP, 0) + hipRotMat*Eigen::Vector3d(0, -DELTA_Y_HIP_JOINT, 0);
    root_hip_pos.block<3, 1>(0, 1) = rootRotMat*Eigen::Vector3d(DELTA_X_HIP, DELTA_Y_HIP, 0) + hipRotMat*Eigen::Vector3d(0, DELTA_Y_HIP_JOINT, 0);
    root_hip_pos.block<3, 1>(0, 2) = rootRotMat*Eigen::Vector3d(-DELTA_X_HIP, -DELTA_Y_HIP, 0) + hipRotMat*Eigen::Vector3d(0, -DELTA_Y_HIP_JOINT, 0);
    root_hip_pos.block<3, 1>(0, 3) = rootRotMat*Eigen::Vector3d(-DELTA_X_HIP, DELTA_Y_HIP, 0) + hipRotMat*Eigen::Vector3d(0, DELTA_Y_HIP_JOINT, 0);

    root_hip_pos_abs.block<3, 1>(0, 0) = root_pos + rootRotMat*Eigen::Vector3d(DELTA_X_HIP, -DELTA_Y_HIP, 0) + hipRotMat*Eigen::Vector3d(0, -DELTA_Y_HIP_JOINT, 0);
    root_hip_pos_abs.block<3, 1>(0, 1) = root_pos + rootRotMat*Eigen::Vector3d(DELTA_X_HIP, DELTA_Y_HIP, 0) + hipRotMat*Eigen::Vector3d(0, DELTA_Y_HIP_JOINT, 0);
    root_hip_pos_abs.block<3, 1>(0, 2) = root_pos + rootRotMat*Eigen::Vector3d(-DELTA_X_HIP, -DELTA_Y_HIP, 0) + hipRotMat*Eigen::Vector3d(0, -DELTA_Y_HIP_JOINT, 0);
    root_hip_pos_abs.block<3, 1>(0, 3) = root_pos + rootRotMat*Eigen::Vector3d(-DELTA_X_HIP, DELTA_Y_HIP, 0) + hipRotMat*Eigen::Vector3d(0, DELTA_Y_HIP_JOINT, 0);

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
            joint_torques.setZero();
            foot_pos_liftoff.setZero();
        
            for (int i = 0; i < NUM_LEG; i++) {
                if (!contacts[i]) {
                    foot_pos_liftoff.block<3, 1>(0, i) = foot_pos_abs.block<3, 1>(0, i);
                }
            }

        } else if (swing_phase == SWING_PHASE_MAX/2 + 1) { // swap swing legs and contact legs
            contacts[0] = false;  // FR swing
            contacts[1] = true;  // FL stance
            contacts[2] = true;  // RR stance
            contacts[3] = false;  // RL swing

            foot_forces_swing.setZero();
            joint_torques.setZero();
            foot_pos_liftoff.setZero();
        
            for (int i = 0; i < NUM_LEG; i++) {
                if (!contacts[i]) {
                    foot_pos_liftoff.block<3, 1>(0, i) = foot_pos_abs.block<3, 1>(0, i);
                }
            }

        }

        // Pick planner based on PLANNER_SELECT
        if (PLANNER_SELECT == 0) {
            go1State::raibertHeuristic(false);
        } else if (PLANNER_SELECT == 1) {
            go1State::raibertHeuristic(true); // w/ Capture Point
        } else {
            go1State::amirHLIP();
        }

        // Find desired foot positions and velocities based on swing_phase, then compute swing leg PD forces
        Eigen::Vector3d bezierPosVec = bezierPos();
        Eigen::Vector3d bezierVelVec = bezierVel();
        bool absolute = false; // toggle for foot positions in absolute or relative frame

        for (int i = 0; i < NUM_LEG; i++) {
            if (!contacts[i] && absolute) {
                foot_pos_d.block<3, 1>(0, i) << root_hip_pos_abs(0, i) + bezierPosVec.x(), root_hip_pos_abs(1, i) + bezierPosVec.y(), bezierPosVec.z(); 
                foot_vel_d.block<3, 1>(0, i) = bezierVelVec;

                go1State::swingPD(i, foot_pos_d.block<3, 1>(0, i), foot_vel_d.block<3, 1>(0, i), true);

            } else if (!contacts[i] && !absolute) {
                foot_pos_d.block<3, 1>(0, i) << root_hip_pos(0, i) + bezierPosVec.x(), root_hip_pos(1, i) + bezierPosVec.y(), bezierPosVec.z() - WALK_HEIGHT; 
                foot_vel_d.block<3, 1>(0, i) = bezierVelVec;

                go1State::swingPD(i, foot_pos_d.block<3, 1>(0, i), foot_vel_d.block<3, 1>(0, i), false);
            }
        }

        if (!init) {
            swing_phase++;
        }

    }

    // Flip init to false for actual planning
    if (init) {
        init = false;
    }
}

void go1State::updateStateFromMujoco(const mjtNum* q_vec, const mjtNum* q_vel, const Eigen::Vector3d &lin_acc) {
/*
    Reads the current state from the MuJoCo simulation and updates the
    go1State object with position, rotation, velocities, and foot 
    location updates. Also ticks swing_phase forward for tracking with
    swing leg PD functionality.
*/
    if (!q_vec || !q_vel) {
        std::cerr << "Error: q_vec or q_vel is NULL!" << std::endl;
        return;
    }
    
    // Update current and old states of the root and feet
    root_pos_old = root_pos;
    root_pos << q_vec[0], q_vec[1], q_vec[2];
    root_lin_vel << q_vel[0], q_vel[1], q_vel[2];
    root_lin_acc = lin_acc;

    root_rpy_old = root_rpy;
    root_quat_old = root_quat;

    root_quat.w() = q_vec[3];
    root_quat.x() = q_vec[4];
    root_quat.y() = q_vec[5];
    root_quat.z() = q_vec[6];

    // root_rpy = rotM2Euler(root_quat.toRotationMatrix());
    root_rpy = quat2Euler(root_quat);
    root_ang_vel << q_vel[3], q_vel[4], q_vel[5];

    foot_pos_old = foot_pos;
    foot_pos = go1FKMujoco(q_vec, root_rpy); // implemented go1FK using Muqun and Leo's work

    if (init) {
        root_pos_old = root_pos;
        root_rpy_old = root_rpy;
        foot_pos_old = foot_pos;
        joint_pos_init << q_vec[7], q_vec[8], q_vec[9], q_vec[10], q_vec[11], q_vec[12],
                            q_vec[13], q_vec[14], q_vec[15], q_vec[16], q_vec[17], q_vec[18];
    }
    
    for (int i = 0; i < NUM_LEG; i++) {
        foot_pos_abs.block<3, 1>(0, i) = foot_pos.block<3, 1>(0, i) + root_pos;
    }

    Eigen::Matrix3d rootRotMat = rotZ(root_rpy(2))*rotY(root_rpy(1))*rotX(root_rpy(0));
    Eigen::Matrix3d hipRotMat = rotZ(root_rpy(2))*rotY(root_rpy(1));

    root_hip_pos.block<3, 1>(0, 0) = rootRotMat*Eigen::Vector3d(DELTA_X_HIP, -DELTA_Y_HIP, 0) + hipRotMat*Eigen::Vector3d(0, -DELTA_Y_HIP_JOINT, 0);
    root_hip_pos.block<3, 1>(0, 1) = rootRotMat*Eigen::Vector3d(DELTA_X_HIP, DELTA_Y_HIP, 0) + hipRotMat*Eigen::Vector3d(0, DELTA_Y_HIP_JOINT, 0);
    root_hip_pos.block<3, 1>(0, 2) = rootRotMat*Eigen::Vector3d(-DELTA_X_HIP, -DELTA_Y_HIP, 0) + hipRotMat*Eigen::Vector3d(0, -DELTA_Y_HIP_JOINT, 0);
    root_hip_pos.block<3, 1>(0, 3) = rootRotMat*Eigen::Vector3d(-DELTA_X_HIP, DELTA_Y_HIP, 0) + hipRotMat*Eigen::Vector3d(0, DELTA_Y_HIP_JOINT, 0);

    root_hip_pos_abs.block<3, 1>(0, 0) = root_pos + rootRotMat*Eigen::Vector3d(DELTA_X_HIP, -DELTA_Y_HIP, 0) + hipRotMat*Eigen::Vector3d(0, -DELTA_Y_HIP_JOINT, 0);
    root_hip_pos_abs.block<3, 1>(0, 1) = root_pos + rootRotMat*Eigen::Vector3d(DELTA_X_HIP, DELTA_Y_HIP, 0) + hipRotMat*Eigen::Vector3d(0, DELTA_Y_HIP_JOINT, 0);
    root_hip_pos_abs.block<3, 1>(0, 2) = root_pos + rootRotMat*Eigen::Vector3d(-DELTA_X_HIP, -DELTA_Y_HIP, 0) + hipRotMat*Eigen::Vector3d(0, -DELTA_Y_HIP_JOINT, 0);
    root_hip_pos_abs.block<3, 1>(0, 3) = root_pos + rootRotMat*Eigen::Vector3d(-DELTA_X_HIP, DELTA_Y_HIP, 0) + hipRotMat*Eigen::Vector3d(0, DELTA_Y_HIP_JOINT, 0);

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
            joint_torques.setZero();
            foot_pos_liftoff.setZero();
        
            for (int i = 0; i < NUM_LEG; i++) {
                if (!contacts[i]) {
                    foot_pos_liftoff.block<3, 1>(0, i) = foot_pos_abs.block<3, 1>(0, i);
                }
            }

        } else if (swing_phase == SWING_PHASE_MAX/2 + 1) { // swap swing legs and contact legs
            contacts[0] = false;  // FR swing
            contacts[1] = true;  // FL stance
            contacts[2] = true;  // RR stance
            contacts[3] = false;  // RL swing

            foot_forces_swing.setZero();
            joint_torques.setZero();
            foot_pos_liftoff.setZero();
        
            for (int i = 0; i < NUM_LEG; i++) {
                if (!contacts[i]) {
                    foot_pos_liftoff.block<3, 1>(0, i) = foot_pos_abs.block<3, 1>(0, i);
                }
            }

        }

        // Pick planner based on PLANNER_SELECT
        if (PLANNER_SELECT == 0) {
            go1State::raibertHeuristic(false);
        } else if (PLANNER_SELECT == 1) {
            go1State::raibertHeuristic(true); // w/ Capture Point
        } else {
            go1State::amirHLIP();
        }

        // Find desired foot positions and velocities based on swing_phase, then compute swing leg PD forces
        Eigen::Vector3d bezierPosVec = bezierPos();
        Eigen::Vector3d bezierVelVec = bezierVel();
        bool absolute = false; // toggle for foot positions in absolute or relative frame

        for (int i = 0; i < NUM_LEG; i++) {
            if (!contacts[i] && absolute) {
                foot_pos_d.block<3, 1>(0, i) << root_hip_pos_abs(0, i) + bezierPosVec.x(), root_hip_pos_abs(1, i) + bezierPosVec.y(), bezierPosVec.z(); 
                foot_vel_d.block<3, 1>(0, i) = bezierVelVec;

                go1State::swingPD(i, foot_pos_d.block<3, 1>(0, i), foot_vel_d.block<3, 1>(0, i), true);

            } else if (!contacts[i] && !absolute) {
                foot_pos_d.block<3, 1>(0, i) << root_hip_pos(0, i) + bezierPosVec.x(), root_hip_pos(1, i) + bezierPosVec.y(), bezierPosVec.z() - WALK_HEIGHT; 
                foot_vel_d.block<3, 1>(0, i) = bezierVelVec;

                go1State::swingPD(i, foot_pos_d.block<3, 1>(0, i), foot_vel_d.block<3, 1>(0, i), false);
            }
        }

        if (!init) {
            swing_phase++;
        }

    } 

    // Flip init to false since old states have been populated in first function call
    if (init) {
        init = false;
    }

}

void go1State::convertForcesToTorquesMujoco(const mjtNum* q_vec) {
/*
    Converts all the foot forces from MPC and swing leg PD to torques for
    the lowest level of control, our joint motor actuators. Based on the
    joint positions relayed to it by MuJoCo simulation.
*/
    if (!q_vec) {
        std::cerr << "Error: q_vec is NULL!" << std::endl;
        return;
    }

    Eigen::Matrix<double, 3, NUM_LEG> foot_forces = foot_forces_grf + foot_forces_swing;
    Eigen::Matrix<double, 12, 1> foot_forces_stacked;

    foot_forces_stacked << foot_forces.block<3, 1>(0, 0), // FR
                            foot_forces.block<3, 1>(0, 1), // FL
                            foot_forces.block<3, 1>(0, 2), // RR
                            foot_forces.block<3, 1>(0, 3); // RL

    Eigen::MatrixXd jac = go1FootJacMujoco(q_vec, root_rpy);
    Eigen::Matrix<double, 18, 1> joint_torques_stacked = jac.transpose() * foot_forces_stacked;

    // Apply torque limits (safety measure)
    Eigen::Matrix<double, 12, 1> joint_torques_limits;
    joint_torques_limits << TORQUE_MAX_HIP, TORQUE_MAX_THIGH, TORQUE_MAX_CALF,
                            TORQUE_MAX_HIP, TORQUE_MAX_THIGH, TORQUE_MAX_CALF,
                            TORQUE_MAX_HIP, TORQUE_MAX_THIGH, TORQUE_MAX_CALF,
                            TORQUE_MAX_HIP, TORQUE_MAX_THIGH, TORQUE_MAX_CALF;

    for (int i = 0; i < 3*NUM_LEG; i++) {
        if (joint_torques_stacked(i+6, 0) > joint_torques_limits(i, 0)) {
            joint_torques_stacked(i+6, 0) = joint_torques_limits(i, 0);
        } else if (joint_torques_stacked(i+6, 0) < -joint_torques_limits(i, 0)) {
            joint_torques_stacked(i+6, 0) = -joint_torques_limits(i, 0);
        }
    }

    joint_torques << joint_torques_stacked.block<3, 1>(6, 0), // FR
                    joint_torques_stacked.block<3, 1>(9, 0), // FL
                    joint_torques_stacked.block<3, 1>(12, 0), // RR
                    joint_torques_stacked.block<3, 1>(15, 0); // RL

}

void go1State::convertForcesToTorquesHardware(const Eigen::VectorXd &jointPos) {
/*
    Converts all the foot forces from MPC and swing leg PD to torques for
    the lowest level of control, our joint motor actuators. Based on the 
    joint positions relayed to it by hardware.
*/
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces = foot_forces_grf + foot_forces_swing;
    Eigen::Matrix<double, 12, 1> foot_forces_stacked;

    foot_forces_stacked << foot_forces.block<3, 1>(0, 0), // FR
                            foot_forces.block<3, 1>(0, 1), // FL
                            foot_forces.block<3, 1>(0, 2), // RR
                            foot_forces.block<3, 1>(0, 3); // RL

    Eigen::MatrixXd jac = go1FootJacHardware(jointPos, root_rpy);
    Eigen::Matrix<double, 18, 1> joint_torques_stacked = jac.transpose() * foot_forces_stacked;

    // Apply torque limits (safety measure)
    Eigen::Matrix<double, 12, 1> joint_torques_limits;
    joint_torques_limits << TORQUE_MAX_HIP, TORQUE_MAX_THIGH, TORQUE_MAX_CALF,
                            TORQUE_MAX_HIP, TORQUE_MAX_THIGH, TORQUE_MAX_CALF,
                            TORQUE_MAX_HIP, TORQUE_MAX_THIGH, TORQUE_MAX_CALF,
                            TORQUE_MAX_HIP, TORQUE_MAX_THIGH, TORQUE_MAX_CALF;

    for (int i = 0; i < 3*NUM_LEG; i++) {
        if (joint_torques_stacked(i+6, 0) > joint_torques_limits(i, 0)) {
            joint_torques_stacked(i+6, 0) = joint_torques_limits(i, 0);
        } else if (joint_torques_stacked(i+6, 0) < -joint_torques_limits(i, 0)) {
            joint_torques_stacked(i+6, 0) = -joint_torques_limits(i, 0);
        }
    }

    joint_torques << joint_torques_stacked.block<3, 1>(6, 0), // FR
                    joint_torques_stacked.block<3, 1>(9, 0), // FL
                    joint_torques_stacked.block<3, 1>(12, 0), // RR
                    joint_torques_stacked.block<3, 1>(15, 0); // RL

}

Eigen::Vector3d go1State::bezierPos() {
/*
    Computes the reference position along a Bezier curve between zero pos
    and a foot position ahead by foot_deltaX and foot_deltaY. The z height
    of the foot is purely based on swing_phase.
*/
    double phi = swing_phase % (SWING_PHASE_MAX/2);
    double phi_max = SWING_PHASE_MAX/2;

    Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_c1 = p0;
    Eigen::Vector3d p1(p0.x() + foot_deltaX/2, p0.y() + foot_deltaY/2, STEP_HEIGHT*2);
    Eigen::Vector3d p_c2(p0.x() + foot_deltaX, p0.y() + foot_deltaY, 0);
    Eigen::Vector3d p2 = p_c2;

    double t = phi/phi_max;
    double oneMinusT = 1.0 - t;

    Eigen::Vector3d fut_posDes = std::pow(oneMinusT, 4) * p0
                                + 4 * std::pow(oneMinusT, 3) * t * p_c1
                                + 6 * std::pow(oneMinusT, 2) * std::pow(t, 2) * p1
                                + 4 * oneMinusT * std::pow(t, 3) * p_c2
                                + std::pow(t, 4) * p2;

    return fut_posDes;
}

Eigen::Vector3d go1State::bezierVel() {
/*
    Computes the reference velocity along a Bezier curve between zero pos
    and a foot position ahead by foot_deltaX and foot_deltaY. Essentially
    the derivative of the above function w.r.t. swing_phase.
*/
    double phi = swing_phase % (SWING_PHASE_MAX/2);
    double phi_max = SWING_PHASE_MAX/2;

    Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_c1 = p0;
    Eigen::Vector3d p1(p0.x() + foot_deltaX/2, p0.y() + foot_deltaY/2, STEP_HEIGHT*2);
    Eigen::Vector3d p_c2(p0.x() + foot_deltaX, p0.y() + foot_deltaY, 0);
    Eigen::Vector3d p2 = p_c2;

    double t = phi/phi_max;
    double oneMinusT = 1.0 - t;

    Eigen::Vector3d fut_velDes = 4 * std::pow(oneMinusT, 3) / phi_max * (p_c1 - p0)
                                + 12 * t * std::pow(oneMinusT, 2) / phi_max * (p1 - p_c1)
                                + 12 * std::pow(t, 2) * oneMinusT / phi_max * (p_c2 - p1)
                                + 4 * std::pow(t, 3) / phi_max * (p2 - p_c2);

    return fut_velDes;
}

void go1State::raibertHeuristic(bool withCapturePoint) {
/*
    Calculates foot_deltaX and foot_deltaY by using a simple Raibert
    Heuristic footstep planner. If you also want capture point, use "true"
    as input; if not, leave input blank or use "false".
*/
    Eigen::Matrix3d rootRotMatZ = rotZ(root_rpy(2));
    Eigen::Vector3d lin_vel_rel = rootRotMatZ.transpose() * root_lin_vel;
    double delta_x;
    double delta_y;
    double swingTime = (SWING_PHASE_MAX + 1)/2.0;

    if (withCapturePoint) {
        // Step lengths delta_* = sqrt(base_height/gravity)*(base_vel_* - base_vel_*_d) + (swing_duration/2)*base_vel_*_d
        double delta_x = std::sqrt(WALK_HEIGHT/9.8) * (lin_vel_rel(0) - root_lin_vel_d(0)) +
                            (swingTime * DT_CTRL) / 2.0 * lin_vel_rel(0);

        double delta_y = std::sqrt(WALK_HEIGHT/9.8) * (lin_vel_rel(1) - root_lin_vel_d(1)) +
                            (swingTime * DT_CTRL) / 2.0 * lin_vel_rel(1);

    } else {
        // Step lengths delta_* = (swing_duration/2)*base_vel_*_d
        double delta_x = (swingTime* DT_CTRL) / 2.0 * lin_vel_rel(0);
        double delta_y = (swingTime * DT_CTRL) / 2.0 * lin_vel_rel(1);
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
    Eigen::Matrix3d rootRotMatZ = rotZ(root_rpy(2));
    Eigen::Vector3d lin_vel_rel = rootRotMatZ.transpose() * root_lin_vel;
    
    double swingProg = 0.0;
    stanceFeetSumXY.setZero(); // sum of stance feet xy positions
    int twoFootContact = 0;

    // Swing progress for each swing leg in the gait cycle, calculates xy sum of stance feet
    for (int i = 0; i < NUM_LEG; i++) {
        if (contacts[i]) {
            stanceFeetSumXY += foot_pos.block<2, 1>(0, i);
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

    // std::cout << "StanceFeetMP: " << StanceFeetMP.transpose() << std::endl;

    rel_error_pos = root_lin_vel_d.segment<2>(0) * DT_CTRL * (1.0 - swingProg)/2.0 - StanceFeetSumXY; // double check the (1 - swingProg) expression
    rel_error_vel = lin_vel_rel.segment<2>(0) - root_lin_vel_d.segment<2>(0);
    u_ref = root_lin_vel_d.segment<2>(0) * DT_CTRL * (SWING_PHASE_MAX/2);

    // Finds the effective LIP frequency and adjusts it based on the vertical acceleration
    double a_ver_plus_g = root_lin_acc(2) - 9.81;
    double f_M; // effective LIP frequency

    if (root_lin_acc(2) > 0){
        f_M = root_lin_acc(2) / root_pos_d (2);
    } 
    if(root_lin_acc(2) > 15.0){
        f_M = 15.0 / root_pos_d(2);
    }
    if(root_lin_acc(2) < 5.0){
        f_M = 25.0;
    }

    // State transition matrix for entire swing duration
    double a1 = cosh(DT_CTRL * (SWING_PHASE_MAX/2) * sqrt(f_M));
    double a2 = sinh(DT_CTRL * (SWING_PHASE_MAX/2) * sqrt(f_M))/sqrt(f_M);
    double a3 = sinh(DT_CTRL * (SWING_PHASE_MAX/2) * sqrt(f_M))*sqrt(f_M);
    double a4 = cosh(DT_CTRL * (SWING_PHASE_MAX/2) * sqrt(f_M));
    
    // State transition matrix for remaining swing duration
    double a1r = cosh((1.0 - 0.5*swingProg) * DT_CTRL * (SWING_PHASE_MAX/2) * sqrt(f_M)); // 0.5 because swing_progress_pram is averaged for the nominally 2 swinging legs
    double a2r = sinh((1.0 - 0.5*swingProg) * DT_CTRL * (SWING_PHASE_MAX/2) * sqrt(f_M))/sqrt(f_M);
    double a3r = sinh((1.0 - 0.5*swingProg) * DT_CTRL * (SWING_PHASE_MAX/2) * sqrt(f_M))*sqrt(f_M);
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
    for (int i=0; i<4; ++i){
        hlip_dense_hessian(i,i) = 2.0*(a1*a1+a3*a3);
        hlip_dense_linearConst(i+2,i) =1.0;
    }

    hlip_gradient.resize(4);
    hlip_gradient << -2*(a1*a1+a3*a3), -2*(a1*a2+a3*a4),-2*(a1*a1+a3*a3), -2*(a1*a2+a3*a4);

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
    // QP solver settings
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

void go1State::swingPD(int leg_idx, Eigen::Vector3d footPosRef, Eigen::Vector3d footVelRef, bool absolute) {
/*
    Calculates the forces for swing legs using PD control with
    the foot positions, velocities, and their respective Bezier 
    curve references for smooth and precise swing leg control.
*/
    if (contacts[leg_idx] == false && !absolute && walking_mode) { // relative frame swing PD
        Eigen::Vector3d foot_posN = foot_pos.block<3, 1>(0, leg_idx);
        Eigen::Vector3d foot_velN = (foot_pos.block<3, 1>(0, leg_idx) - foot_pos_old.block<3, 1>(0, leg_idx))/DT_CTRL;

        Eigen::Matrix3d kp = SWING_KP * Eigen::Matrix3d::Identity();
        Eigen::Matrix3d kd = SWING_KD * Eigen::Matrix3d::Identity();

        foot_forces_swing.block<3, 1>(0, leg_idx) = kp * (footPosRef - foot_posN) + kd * (footVelRef - foot_velN);

    } else if (contacts[leg_idx] == false && absolute && walking_mode) { // absolute frame swing PD
        Eigen::Vector3d foot_posN = foot_pos_abs.block<3, 1>(0, leg_idx);
        Eigen::Vector3d foot_velN = (foot_pos.block<3, 1>(0, leg_idx) - foot_pos_old.block<3, 1>(0, leg_idx))/DT_CTRL;

        Eigen::Matrix3d kp = SWING_KP * Eigen::Matrix3d::Identity();
        Eigen::Matrix3d kd = SWING_KD * Eigen::Matrix3d::Identity();

        foot_forces_swing.block<3, 1>(0, leg_idx) = kp * (footPosRef - foot_posN) + kd * (footVelRef - foot_velN);

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

    switch(joint_idx % 3) {
        case 0:
            if (startup) {
                jointInterp = (1.0 - squat_prog) * joint_pos_init(joint_idx);
                joint_torques(joint_idx % 3, joint_idx / 3) = std::clamp(JOINT_KP * (jointInterp - jointPos) + JOINT_KD * (0 - jointVel), -TORQUE_MAX_HIP, TORQUE_MAX_HIP);
                return;
            } else {
                joint_torques(joint_idx % 3, joint_idx / 3) = std::clamp(JOINT_KP * (0.0 - jointPos) + JOINT_KD * (0.0 - jointVel), -TORQUE_MAX_HIP, TORQUE_MAX_HIP);
                return;
            }
        case 1:
            if (startup) {
                jointInterp = (1.0 - squat_prog) * joint_pos_init(joint_idx) + squat_prog * THIGH_RAD_STAND;
                joint_torques(joint_idx % 3, joint_idx / 3) = std::clamp(JOINT_KP * (jointInterp - jointPos) + JOINT_KD * (0 - jointVel), -TORQUE_MAX_THIGH, TORQUE_MAX_THIGH);
                return;
            } else {
                jointInterp = (1.0 - squat_prog) * joint_pos_init(joint_idx) + squat_prog * THIGH_RAD_STAND;
                joint_torques(joint_idx % 3, joint_idx / 3) = std::clamp(JOINT_KP * (jointInterp - jointPos) + JOINT_KD * (0.0 - jointVel), -TORQUE_MAX_THIGH, TORQUE_MAX_THIGH);
                return;
            }
            
        case 2:
            if (startup) {
                jointInterp = (1.0 - squat_prog) * joint_pos_init(joint_idx) + squat_prog * CALF_RAD_STAND;
                joint_torques(joint_idx % 3, joint_idx / 3) = std::clamp(JOINT_KP * (jointInterp - jointPos) + JOINT_KD * (0 - jointVel), -TORQUE_MAX_CALF, TORQUE_MAX_CALF);
                return;
            } else {
                jointInterp = (1.0 - squat_prog) * joint_pos_init(joint_idx) + squat_prog * CALF_RAD_STAND;
                joint_torques(joint_idx % 3, joint_idx / 3) = std::clamp(JOINT_KP * (jointInterp - jointPos) + JOINT_KD * (0.0 - jointVel), -TORQUE_MAX_CALF, TORQUE_MAX_CALF);
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

void go1State::computeStartupPDMujoco(const mjtNum* q_vec, const mjtNum* q_vel) {
/*
    Activate simple joint PD for startup mode in MuJoCo.
*/
    for (int i = 0; i < 3*NUM_LEG; i++) {
        go1State::jointPD(i, q_vec[7 + i], q_vel[i + 6]);
    }

    if (!isStartupComplete()) squat_prog += 0.002;
}

void go1State::computeShutdownPDMujoco(const mjtNum* q_vec, const mjtNum* q_vel) {
/*
    Activate simple joint PD for shutdown mode in MuJoCo.
*/
    for (int i = 0; i < 3*NUM_LEG; i++) {
        go1State::jointPD(i, q_vec[7 + i], q_vel[i + 6], false);
    }

    if (!isShutdownComplete()) squat_prog -= 0.002;
}

void go1State::computeStartupPDhardware(UNITREE_LEGGED_SDK::LowState& state) {
/*
    Activate simple joint PD for startup mode in MuJoCo.
*/
    for (int i = 0; i < 3*NUM_LEG; i++) {
        go1State::jointPD(i, state.motorState[i].q, state.motorState[i].dq);
    }

    if (!isStartupComplete()) squat_prog += 0.002;
}


void go1State::computeShutdownPDHarware(UNITREE_LEGGED_SDK::LowState& state) {
/*
    Activate simple joint PD for shutdown mode in MuJoCo.
*/
    for (int i = 0; i < 3*NUM_LEG; i++) {
        go1State::jointPD(i, state.motorState[i].q, state.motorState[i].dq, false);
    }

    if (!isShutdownComplete()) squat_prog -= 0.002;
}

go1StateSnapshot go1State::getSnapshot() const {
    std::lock_guard<std::mutex> lock(mtx_);
    go1StateSnapshot stateSnap;
    stateSnap.foot_pos = foot_pos;
    stateSnap.go1_lumped_inertia = go1_lumped_inertia;
    stateSnap.root_pos = root_pos;
    stateSnap.root_pos_d = root_pos_d;
    stateSnap.root_rpy = root_rpy;
    stateSnap.root_rpy_d = root_rpy_d;
    stateSnap.root_lin_vel = root_lin_vel;
    stateSnap.root_lin_vel_d = root_lin_vel_d;
    stateSnap.root_ang_vel = root_ang_vel;
    stateSnap.root_ang_vel_d = root_ang_vel_d;
    stateSnap.walking_mode = walking_mode;
    stateSnap.swing_phase = swing_phase;

    return stateSnap;
}

void go1State::setGRFForces(const Eigen::Matrix<double, 3, NUM_LEG> &grf) {
    std::lock_guard<std::mutex> lock(mtx_);
    foot_forces_grf = grf;
}