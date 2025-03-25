#include "go1_cpp_cmake/go1StanceMPC.h"

void go1StanceMPC(go1State &state) {
/*
    Standalone script for calculating the Model Predictive Control (MPC)
    problem for the stance phase of the quadruped. This script is a
    one-and-done version of the go1GRFMPC.cpp file, which is a more class-
    oriented version of this script.
*/
    // Extract information from the state
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos = state.foot_pos;
    Eigen::Matrix3d rootRotMatZ = rotZ(state.root_rpy(2));
    Eigen::Matrix3d inertiaEst = rootRotMatZ * state.go1_lumped_inertia * rootRotMatZ.transpose();

    // Continuous single rigid body dynamics
    Eigen::Matrix<double, 13, 13> A_mat = Eigen::Matrix<double, 13, 13>::Zero();
    A_mat.block<3, 3>(0, 6) = rootRotMatZ.transpose();
    A_mat.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
    A_mat(11, 12) = 1.0;

    Eigen::Matrix<double, 13, 13> A_mat_d = Eigen::Matrix<double, 13, 13>::Identity() + A_mat*DT_MPC;

    Eigen::Matrix<double, 13, 12> B_mat = Eigen::Matrix<double, 13, 12>::Zero();
    for (int i = 0; i < NUM_LEG; i++) {
        Eigen::Vector3d footPosN = foot_pos.block<3, 1>(0, i);
        Eigen::Vector3d CoM (-0.0012, 0.0009, -0.0174); //(0, 0, 0); //
        Eigen::Matrix3d skewFootN = skew(footPosN - CoM);

        B_mat.block<3, 3>(6, 3*i) = inertiaEst.ldlt().solve(skewFootN);
        B_mat.block<3, 3>(9, 3*i) = Eigen::Matrix3d::Identity() / ROBOT_MASS;
    }

    Eigen::Matrix<double, 13, 12> B_mat_d = B_mat*DT_MPC;

    // Calculate MPC states using linear interpolation between current states, desired states, and velocity commands
    Eigen::Matrix<double, 13, 1> mpc_state;
    Eigen::Matrix<double, 13*MPC_HORIZON, 1> mpc_state_d;
    mpc_state << state.root_rpy, state.root_pos, state.root_ang_vel, state.root_lin_vel, -9.81;
    
    Eigen::Matrix<double, 13, 1> mpc_state_d_block;
    
    Eigen::Vector3d pos_ref = state.root_pos;
    Eigen::Vector3d rpy_ref = state.root_rpy;
    Eigen::Vector3d lin_vel_ref = state.root_lin_vel;
    Eigen::Vector3d ang_vel_ref = state.root_ang_vel;

    for (int i = 0; i < MPC_HORIZON; i++) {
        pos_ref = (1 - ALEPH) * pos_ref + ALEPH * (state.root_pos_d + state.root_lin_vel_d * i * DT_MPC);
        rpy_ref = (1 - ALEPH) * rpy_ref + ALEPH * (state.root_rpy_d + state.root_ang_vel_d * i * DT_MPC);
        lin_vel_ref = (1 - BETTA) * lin_vel_ref + BETTA * state.root_lin_vel_d;
        ang_vel_ref = (1 - BETTA) * ang_vel_ref + BETTA * state.root_ang_vel_d;

        if ((pos_ref - state.root_pos_d).norm() <= 1e-2) {
            lin_vel_ref.setZero();
            pos_ref = state.root_pos_d;
        }

        if ((rpy_ref - state.root_rpy_d).norm() <= 1e-2) {
            ang_vel_ref.setZero();
            rpy_ref = state.root_rpy_d;
        }

        mpc_state_d_block << rpy_ref, pos_ref, ang_vel_ref, lin_vel_ref, -9.81;
        mpc_state_d.block<13, 1>(13*i, 0) = mpc_state_d_block;

    }

    // Calculate weight matrices
    Eigen::Matrix<double, 13, 1> q_weights;
    q_weights << q_weight_1, q_weight_2, q_weight_3, 
                q_weight_4, q_weight_5, q_weight_6, 
                q_weight_7, q_weight_8, q_weight_9, 
                q_weight_10, q_weight_11, q_weight_12, 0;

    Eigen::MatrixXd Q_diag_blk = q_weights.asDiagonal();

    Eigen::Matrix<double, 13*MPC_HORIZON, 13*MPC_HORIZON> Q = Eigen::Matrix<double, 13*MPC_HORIZON, 13*MPC_HORIZON>::Zero();

    for (int i = 0; i < MPC_HORIZON; i++) {
        Q.block<13, 13>(i*13, i*13) = Q_diag_blk;
    }

    double r_weight = r_weight_val;
    Eigen::Matrix<double, 3*NUM_LEG*MPC_HORIZON, 3*NUM_LEG*MPC_HORIZON> R = Eigen::Matrix<double, 3*NUM_LEG*MPC_HORIZON, 3*NUM_LEG*MPC_HORIZON>::Identity();
    R = r_weight*R;

    // Calculate A_qp and B_qp matrices
    Eigen::Matrix<double, MPC_REF_DIM, 13> A_qp = Eigen::Matrix<double, MPC_REF_DIM, 13>::Zero();
    Eigen::Matrix<double, MPC_REF_DIM, MPC_INPUT_DIM> B_qp = Eigen::Matrix<double, MPC_REF_DIM, MPC_INPUT_DIM>::Zero();

    for (int i = 0; i < MPC_HORIZON; i++) {
        if (i == 0) {
            A_qp.block<13, 13> (0, 0) = A_mat_d;
        } else {
            A_qp.block<13, 13> (13*i, 0) = A_mat_d * A_qp.block<13, 13>(13*(i-1), 0);
        }

        for (int j = 0; j < i+1; j++) {
            if (i-j == 0) {
                B_qp.block<13, 12> (13*i, 12*j) = B_mat_d;
            } else {
                B_qp.block<13, 12> (13*i, 12*j) = A_qp.block<13, 13>(13*(i-j-1), 0) * B_mat_d;
            }
        }
    }

    // Calculate Hessian P = 2.0 * (B_qp^T * Q * B_qp + R)
    Eigen::Matrix<double, 12*MPC_HORIZON, 12*MPC_HORIZON> denseP = 2.0 * (B_qp.transpose() * Q * B_qp + R);
    Eigen::SparseMatrix<double> P = denseP.sparseView();

    // Calculate gradient q = 2.0 * B_qp^T * Q * (A_qp * x_curr - x_ref)
    Eigen::Matrix<double, MPC_INPUT_DIM, 1> q = 2.0 * B_qp.transpose() * Q * (A_qp * mpc_state - mpc_state_d);

    // Set up friction pyramid inequality constraints
    Eigen::Matrix<double, FRIC_PYR*NUM_LEG, 3*NUM_LEG> C_ineq_blk;
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

    Eigen::Matrix<double, FRIC_PYR*NUM_LEG*MPC_HORIZON, MPC_INPUT_DIM> C_ineq = Eigen::Matrix<double, FRIC_PYR*NUM_LEG*MPC_HORIZON, MPC_INPUT_DIM>::Zero();

    Eigen::VectorXd qp_lb = Eigen::VectorXd::Constant(FRIC_PYR * NUM_LEG * MPC_HORIZON, -std::numeric_limits<double>::infinity());;
    Eigen::VectorXd qp_ub (FRIC_PYR * NUM_LEG * MPC_HORIZON, 1);

    for (int i = 0; i < MPC_HORIZON; i++) {
        C_ineq.block<FRIC_PYR*NUM_LEG, 3*NUM_LEG>(i*FRIC_PYR*NUM_LEG, i*3*NUM_LEG) = C_ineq_blk;
        qp_ub.segment<NUM_LEG*FRIC_PYR>(i*NUM_LEG*FRIC_PYR) << 0, 0, 0, 0, FZ_MAX, FZ_MIN,
                                                                0, 0, 0, 0, FZ_MAX, FZ_MIN,
                                                                0, 0, 0, 0, FZ_MAX, FZ_MIN,
                                                                0, 0, 0, 0, FZ_MAX, FZ_MIN;
    }
    
    // Set up swing equality constraints
    Eigen::Matrix<double, 2*MPC_HORIZON, MPC_INPUT_DIM> C_eq = Eigen::Matrix<double, 2*MPC_HORIZON, MPC_INPUT_DIM>::Zero();
    Eigen::SparseMatrix<double> C_qp;
    Eigen::Matrix<double, 2, 12> C_eq_blk14;
    Eigen::Matrix<double, 2, 12> C_eq_blk23;

    C_eq_blk14 << 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1; // FR + RL swinging
    C_eq_blk23 << 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0; // FL + RR swinging 

    if (state.walking_mode) {
        int tempSwingPhase = state.swing_phase;
        int swingPhaseShift = DT_MPC/0.002;
        int tempCurrSwingPhase = tempSwingPhase;

        for (int i = 0; i < MPC_HORIZON; i++) {
            tempCurrSwingPhase = (tempSwingPhase + i*swingPhaseShift) % SWING_PHASE_MAX;

            if (tempCurrSwingPhase <= SWING_PHASE_MAX/2) {
                C_eq.block<2, 12>(i*2, i*12) = C_eq_blk23;
            } else {
                C_eq.block<2, 12>(i*2, i*12) = C_eq_blk14;
            }
        }

        Eigen::MatrixXd dense_C_qp = Eigen::MatrixXd::Zero(FRIC_PYR*NUM_LEG*MPC_HORIZON + 2*MPC_HORIZON, MPC_INPUT_DIM);

        // C_eq, then C_ineq
        dense_C_qp.topRows(C_eq.rows()) = C_eq;
        dense_C_qp.bottomRows(C_ineq.rows()) = C_ineq;
        C_qp = dense_C_qp.sparseView();

        Eigen::VectorXd temp_qp_ub (qp_ub.size() + 2*MPC_HORIZON);
        Eigen::VectorXd temp_qp_lb (qp_lb.size() + 2*MPC_HORIZON);

        temp_qp_ub.head(2*MPC_HORIZON).setZero();
        temp_qp_ub.tail(qp_ub.size()) = qp_ub;

        temp_qp_lb.head(2*MPC_HORIZON).setZero();
        temp_qp_lb.tail(qp_lb.size()) = qp_lb;

        qp_ub = temp_qp_ub;
        qp_lb = temp_qp_lb;

    } else {
        C_qp = C_ineq.sparseView();

    }

    // Solve the MPC problem
    OsqpEigen::Solver mpc_solver;
    mpc_solver.clearSolver();

    mpc_solver.settings()->setVerbosity(false);
    mpc_solver.settings()->setWarmStart(false);
    mpc_solver.data()->setNumberOfVariables(MPC_INPUT_DIM);
    mpc_solver.data()->setNumberOfConstraints(qp_lb.size());
    mpc_solver.data()->setLowerBound(qp_lb);
    mpc_solver.data()->setUpperBound(qp_ub);
    mpc_solver.data()->setHessianMatrix(P);
    mpc_solver.data()->setGradient(q);
    mpc_solver.data()->setLinearConstraintsMatrix(C_qp);

    mpc_solver.initSolver();
    mpc_solver.solveProblem();

    Eigen::VectorXd solution = mpc_solver.getSolution();
    state.foot_forces_grf.setZero();

    // Store the result into state.foot_forces_grf
    for (int i = 0; i < NUM_LEG; ++i) {
        if (!solution.segment<3>(i * 3).hasNaN()) {
            state.foot_forces_grf.block<3, 1>(0, i) = -solution.segment<3>(i * 3);
        }
    }
    
}