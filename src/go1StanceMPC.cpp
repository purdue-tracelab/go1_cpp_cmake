#include "go1_cpp_cmake/go1StanceMPC.h"

go1MPC::go1MPC() {
/*
    Initializes the go1MPC class for calculating the Model Predictive Control (MPC)
    problem for the stance phase of the quadruped. This class is a more class-oriented
    version of the go1StanceMPC() function.
*/
    reset();

    // Set up QP weights
    q_weights << q_weight_1, q_weight_2, q_weight_3, 
                q_weight_4, q_weight_5, q_weight_6, 
                q_weight_7, q_weight_8, q_weight_9, 
                q_weight_10, q_weight_11, q_weight_12, 0;

    Q.resize(13*MPC_HORIZON, 13*MPC_HORIZON);

    for (int i = 0; i < 13*MPC_HORIZON; i++) {
        // double w = (i < 13*2 ? 10.0 : 2.0); // consult w/ Muqun
        // Q.insert(i, i) = w * q_weights(i % 13);
        Q.insert(i, i) = q_weights(i % 13);
    }

    R.resize(3*NUM_LEG*MPC_HORIZON, 3*NUM_LEG*MPC_HORIZON);
    R.setIdentity();
    R *= r_weight_val;

    // Set up friction pyramid inequality constraints matrix & constraint bounds
    C_ineq_blk << 1, 0, -MU, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        -1, 0, -MU, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, -MU, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, -1, -MU, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, -MU, 0, 0, 0, 0, 0, 0,
        0, 0, 0, -1, 0, -MU, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, -MU, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, -1, -MU, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, -MU, 0, 0, 0,
        0, 0, 0, 0, 0, 0, -1, 0, -MU, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, -MU, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, -1, -MU, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, -MU,
        0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, -MU,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -MU,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -MU,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1;

    qp_lb.resize(FRIC_PYR*NUM_LEG*MPC_HORIZON + 2*MPC_HORIZON, 1);
    qp_lb.setZero();
    qp_lb.tail(FRIC_PYR*NUM_LEG*MPC_HORIZON) = Eigen::VectorXd::Constant(FRIC_PYR * NUM_LEG * MPC_HORIZON, -std::numeric_limits<double>::infinity());    
    
    qp_ub.resize(FRIC_PYR*NUM_LEG*MPC_HORIZON + 2*MPC_HORIZON, 1);
    qp_ub.setZero();

    for (int i = 0; i < MPC_HORIZON; i++) {
        C_ineq.block<FRIC_PYR*NUM_LEG, 3*NUM_LEG>(i*FRIC_PYR*NUM_LEG, i*3*NUM_LEG) = C_ineq_blk;
        qp_ub.segment<NUM_LEG*FRIC_PYR>(2*MPC_HORIZON + i*NUM_LEG*FRIC_PYR) << 0, 0, 0, 0, FZ_MAX, FZ_MIN,
                                                                                0, 0, 0, 0, FZ_MAX, FZ_MIN,
                                                                                0, 0, 0, 0, FZ_MAX, FZ_MIN,
                                                                                0, 0, 0, 0, FZ_MAX, FZ_MIN;
    }

    // Place C_ineq into C_qp
    dense_C_qp.bottomRows(C_ineq.rows()) = C_ineq;
    C_qp = dense_C_qp.sparseView();

    // Place dummy values in C_qp for C_eq swing equality constraints
    for (int i = 0; i < MPC_HORIZON; i++) {
        C_qp.insert(i*2, 2 + i*12) = 5;
        C_qp.insert(i*2, 5 + i*12) = 5;
        C_qp.insert(i*2 + 1, 8 + i*12) = 5;
        C_qp.insert(i*2 + 1, 11 + i*12) = 5;
    }

}

void go1MPC::reset() {
/*
    Resets the go1MPC class object to a blank state (all matrices are zero).
    This is useful for re-initializing the object after a simulation or
    when changing the model parameters.
*/
    q_weights.setZero();
    Q.setZero();
    R.setZero();

    A_mat.setZero();
    A_mat_d.setIdentity();
    B_mat.setZero();
    B_mat_d.setZero();

    A_qp.setZero();
    B_qp.setZero();
    A_qp_sparse.resize(MPC_REF_DIM, 13);
    B_qp_sparse.resize(MPC_REF_DIM, MPC_INPUT_DIM);
    B_qp_T_Q_sparse.resize(MPC_INPUT_DIM, MPC_INPUT_DIM);

    mpc_state.setZero();
    mpc_state_d_block.setZero();
    mpc_state_d.setZero();

    P.resize(12*MPC_HORIZON, 12*MPC_HORIZON);
    q.setZero();
    qp_lb.setZero();
    qp_ub.setZero();

    dense_C_qp.setZero();
    C_qp.resize(FRIC_PYR*NUM_LEG*MPC_HORIZON + 2*MPC_HORIZON, MPC_INPUT_DIM); // initialize for walking mode size
    C_ineq.setZero();
    C_ineq_blk.setZero();

}

void go1MPC::solveMPCFromSnapshot(go1StateSnapshot &snap) {
    updateRigidBodyModelFromSnapshot(snap);
    updateMPCStatesFromSnapshot(snap);
    buildAndSolveQPForSnapshot(snap);
}

void go1MPC::solveMPCForState(go1State &state) {
    auto snap = state.getSnapshot();
    solveMPCFromSnapshot(snap);
    state.retrieveGRF(snap.grf_forces);
}

// go1State wrapper for updateRigidBodyModelFromSnapshot
void go1MPC::updateRigidBodyModel(const go1State& state) {
    auto snap = state.getSnapshot();
    updateRigidBodyModelFromSnapshot(snap);
}

void go1MPC::updateRigidBodyModelFromSnapshot(const go1StateSnapshot &snap) {
/*
    Updates the rigid body model of the quadruped robot using the current
    root orientation (roll, pitch, yaw), inertia matrix, and foot positions.
    This is used to calculate the dynamics of the robot in the MPC problem.
*/
    // Extract information from the go1StateSnapshot object
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos = snap.foot_pos_world_rot;
    Eigen::Matrix3d rootRotMatZ = rotZ(snap.root_rpy(2));
    Eigen::Matrix3d inertiaEst = rootRotMatZ * snap.go1_lumped_inertia * rootRotMatZ.transpose();

    // Continuous single rigid body dynamics
    A_mat.setZero();
    A_mat.block<3, 3>(0, 6) = rootRotMatZ.transpose();
    A_mat.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
    A_mat(11, 12) = 1.0;

    A_mat_d.setIdentity();
    A_mat_d.noalias() += A_mat*DT_MPC;

    B_mat.setZero();
    for (int i = 0; i < NUM_LEG; i++) {
        Eigen::Vector3d footPosN = foot_pos.col(i);
        Eigen::Vector3d CoM(-0.0012, 0.0009, -0.0174); //(0, 0, 0); //
        Eigen::Matrix3d skewFootN = skew(footPosN - CoM);

        B_mat.block<3, 3>(6, 3*i) = inertiaEst.ldlt().solve(skewFootN);
        B_mat.block<3, 3>(9, 3*i) = Eigen::Matrix3d::Identity() / ROBOT_MASS;
    }

    B_mat_d.noalias() = B_mat*DT_MPC;
}

// go1State wrapper for updateMPCStatesFromSnapshot
void go1MPC::updateMPCStates(const go1State& state) {
    auto snap = state.getSnapshot();
    updateMPCStatesFromSnapshot(snap);
}

void go1MPC::updateMPCStatesFromSnapshot(const go1StateSnapshot &snap) {
/*
    Sets up the MPC states based on the current and desired poses and velocities
    of Go1, and creates a linear interpolation between the current and desired states
    as the MPC trajectory. Uses ALEPH & BETTA in go1Params.h for interpolation.
*/
    mpc_state.setZero();
    mpc_state_d.setZero();
    mpc_state_d_block.setZero();
    mpc_state << snap.root_rpy, snap.root_pos, snap.root_ang_vel, snap.root_lin_vel, -9.81;

    Eigen::Vector3d pos_ref = snap.root_pos;
    Eigen::Vector3d rpy_ref = snap.root_rpy;
    Eigen::Vector3d lin_vel_ref = snap.root_lin_vel;
    Eigen::Vector3d ang_vel_ref = snap.root_ang_vel;

    // Consult w/ Muqun on below
    for (int i = 0; i < MPC_HORIZON; i++) {
        pos_ref = (1 - ALEPH) * pos_ref + ALEPH * (snap.root_pos_d + snap.root_lin_vel_d * i * DT_MPC);
        rpy_ref = (1 - ALEPH) * rpy_ref + ALEPH * (snap.root_rpy_d + snap.root_ang_vel_d * i * DT_MPC);
        lin_vel_ref = (1 - BETTA) * lin_vel_ref + BETTA * snap.root_lin_vel_d;
        ang_vel_ref = (1 - BETTA) * ang_vel_ref + BETTA * snap.root_ang_vel_d;

        if ((pos_ref - snap.root_pos_d).norm() <= 1e-2) {
            lin_vel_ref.setZero();
            pos_ref = snap.root_pos_d;
        }

        if ((rpy_ref - snap.root_rpy_d).norm() <= 1e-2) {
            ang_vel_ref.setZero();
            rpy_ref = snap.root_rpy_d;
        }

        mpc_state_d_block << rpy_ref, pos_ref, ang_vel_ref, lin_vel_ref, -9.81;
        mpc_state_d.block<13, 1>(13*i, 0) = mpc_state_d_block;

    }
}

// go1State wrapper for solveMPCForcesForSnapshot
void go1MPC::buildAndSolveQP(go1State& state) {
    auto snap = state.getSnapshot();
    buildAndSolveQPForSnapshot(snap);
    state.retrieveGRF(snap.grf_forces);
}

void go1MPC::buildAndSolveQPForSnapshot(go1StateSnapshot &snap) {
/*
    Arranges the QP-MPC problem and solves it using the OSQP solver.
    Returns the calculated forces for the stance legs of Go1.
*/
    // Calculate A_qp and B_qp matrices
    A_qp.setZero();
    B_qp.setZero();
    for (int i = 0; i < MPC_HORIZON; i++) {
        if (i == 0) {
            A_qp.block<13, 13>(0, 0) = A_mat_d;
        } else {
            A_qp.block<13, 13>(13*i, 0).noalias() = A_mat_d * A_qp.block<13, 13>(13*(i-1), 0);
        }

        for (int j = 0; j < i+1; j++) {
            if (i-j == 0) {
                B_qp.block<13, 12>(13*i, 12*j) = B_mat_d;
            } else {
                B_qp.block<13, 12>(13*i, 12*j).noalias() = A_qp.block<13, 13>(13*(i-j-1), 0) * B_mat_d;
            }
        }
    }

    A_qp_sparse = A_qp.sparseView();
    B_qp_sparse = B_qp.sparseView();
    B_qp_T_Q_sparse = B_qp_sparse.transpose() * Q;

    // Calculate Hessian P = 2.0 * (B_qp^T * Q * B_qp + R)
    P = 2.0 * (B_qp_T_Q_sparse * B_qp_sparse + R);

    // Calculate gradient q = 2.0 * B_qp^T * Q * (A_qp * x_curr - x_ref)
    q = 2.0 * B_qp_T_Q_sparse * (A_qp_sparse * mpc_state - mpc_state_d);

    // Set up swing equality constraint section of C_qp
    if (snap.walking_mode) {
        int tempSwingPhase = snap.swing_phase;
        int swingPhaseShift = DT_MPC/DT_CTRL;
        int tempCurrSwingPhase = tempSwingPhase;

        // Fill in contacts according to nominal gait schedule
        for (int i = 0; i < MPC_HORIZON; i++) {
            tempCurrSwingPhase = (tempSwingPhase + i*swingPhaseShift) % SWING_PHASE_MAX;

            if (tempCurrSwingPhase <= SWING_PHASE_MAX/2) {
                // FL + RR swinging
                C_qp.coeffRef(i*2, 2 + i*12) = 0;
                C_qp.coeffRef(i*2, 5 + i*12) = 1;
                C_qp.coeffRef(i*2 + 1, 8 + i*12) = 1;
                C_qp.coeffRef(i*2 + 1, 11 + i*12) = 0;

            } else {
                // FR + RL swinging
                C_qp.coeffRef(i*2, 2 + i*12) = 1;
                C_qp.coeffRef(i*2, 5 + i*12) = 0;
                C_qp.coeffRef(i*2 + 1, 8 + i*12) = 0;
                C_qp.coeffRef(i*2 + 1, 11 + i*12) = 1;
            }
        }

    } else {
        for (int i = 0; i < MPC_HORIZON; i++) {
            C_qp.coeffRef(i*2, 2 + i*12) = 0;
            C_qp.coeffRef(i*2, 5 + i*12) = 0;
            C_qp.coeffRef(i*2 + 1, 8 + i*12) = 0;
            C_qp.coeffRef(i*2 + 1, 11 + i*12) = 0;
        }

    }

    // Setup the OsqpEigen solver
    if (!mpc_solver.isInitialized()) {
        mpc_solver.clearSolver();
        mpc_solver.settings()->setVerbosity(false);
        mpc_solver.settings()->setWarmStart(true);
        mpc_solver.settings()->setTimeLimit(0.001);
    
        mpc_solver.data()->setNumberOfVariables(MPC_INPUT_DIM);
        mpc_solver.data()->setNumberOfConstraints(qp_lb.size());
        mpc_solver.data()->setLowerBound(qp_lb);
        mpc_solver.data()->setUpperBound(qp_ub);
        mpc_solver.data()->setHessianMatrix(P);
        mpc_solver.data()->setGradient(q);
        mpc_solver.data()->setLinearConstraintsMatrix(C_qp);
    
        mpc_solver.initSolver();

    } else {
        mpc_solver.updateHessianMatrix(P);
        mpc_solver.updateGradient(q);
        mpc_solver.updateLinearConstraintsMatrix(C_qp);

    }

    mpc_solver.solveProblem();
    Eigen::VectorXd solution = mpc_solver.getSolution();

    // Store the result into go1StateSnapshot's grf_forces
    for (int i = 0; i < NUM_LEG; ++i) {
        if (!solution.segment<3>(i * 3).hasNaN())
            snap.grf_forces.col(i) = -solution.segment<3>(i * 3);   
    }

}