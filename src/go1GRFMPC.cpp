#include "go1_cpp_cmake/go1GRFMPC.h"

/*
    go1GRFMPC.cpp sets up the grfMPC class, which handles the setup and
    calculation of the Model Predictive Control problem in a quadratic
    programming (QP) format for convex optimization (minimization). The
    details of this formulation are covered in the Overleaf document, and
    the MPC weights are handled by q_weights and r_weight. This class is
    pretty computationally expensive and dense, so debugging this may be
    challenging if you're not familiar with it already.
*/

grfMPC::grfMPC() {
/*
    Initializes the grfMPC class by setting the MPC weights (q_weights,
    r_weight) as diagonal matrices, then establishes the frictional
    pyramid constraint selection matrix. I'll add the details for this
    section later, as they change between the MATLAB and C++ implementations.
*/
    reset();

}

void grfMPC::reset() {
/*
    Resets all the matrices involved in constructing the QP-MPC problem.
*/
    A_mat.setZero();
    B_mat.setZero();
    
    A_qp.setZero();
    B_qp.setZero();
    C_ineq.setZero();
    C_eq.setZero();
    C_qp.setZero();

    mpc_state.setZero();
    mpc_state_d_block.setZero();
    mpc_state_d.setZero();

    P.setZero();
    q.setZero();

}

void grfMPC::calculateA(const Eigen::Vector3d &root_rpy) {
/*
    Calculates the state propagation matrix A in continuous-time domain,
    then discretizes it for digital control.
*/
    Eigen::Matrix3d rootRotMatZ = rotZ(root_rpy(2));

    A_mat.block<3, 3>(0, 6) = rootRotMatZ.transpose();
    A_mat.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
    A_mat(11, 12) = 1.0;

    A_mat = Eigen::Matrix<double, 13, 13>::Identity() + A_mat*DT_MPC;

}

void grfMPC::calculateB(const Eigen::Matrix3d &inertia, const Eigen::Vector3d &root_rpy, const Eigen::Matrix<double, 3, NUM_LEG> foot_pos) {
/*
    Calculate the control input matrix B in continuous-time domain,
    then discretizes it for digital control.
*/
    Eigen::Matrix3d rootRotMatZ = rotZ(root_rpy(2));
    Eigen::Matrix3d inertiaEst = rootRotMatZ * inertia * rootRotMatZ.transpose();

    // std::cout << "estimated inertia:\n" << inertiaEst << std::endl;

    for (int i = 0; i < NUM_LEG; i++) {
        Eigen::Vector3d footPosN = foot_pos.block<3, 1>(0, i);
        Eigen::Vector3d CoM (0, 0, 0); //(-0.0012, 0.0009, -0.0174);
        Eigen::Matrix3d skewFootN = skew(footPosN - CoM);

        // std::cout << "skew foot calculation for foot " << i << ":\n" << skewFootN << std::endl;

        B_mat.block<3, 3>(6, 3*i) = inertiaEst.ldlt().solve(skewFootN);
        B_mat.block<3, 3>(9, 3*i) = Eigen::Matrix3d::Identity() / ROBOT_MASS;
    }

    B_mat = B_mat*DT_MPC;

}

void grfMPC::calculateMPCStates(const go1State &state) {
/*
    Calculates the MPC initial state and future predicted states for
    MPC optimization.
*/
    mpc_state << state.root_rpy, state.root_pos, state.root_ang_vel, state.root_lin_vel, -9.81;
    mpc_state_d_block << state.root_rpy, state.root_pos, state.root_ang_vel_d, state.root_lin_vel_d, -9.81;
    // bool reachedPosD = false;
    // bool reachedRpyD = false;

    // for (int i = 0; i < MPC_HORIZON; i++) {
    //     if (!reachedRpyD) {
    //         mpc_state_d_block.block<3, 1>(0, 0) += mpc_state_d_block.block<3, 1>(6, 0) * DT_MPC * i;
    //     }

    //     if (!reachedPosD) {
    //         mpc_state_d_block.block<3, 1>(3, 0) += mpc_state_d_block.block<3, 1>(9, 0) * DT_MPC * i;
    //     }

    //     if ((mpc_state_d_block.block<3, 1>(0, 0) - state.root_rpy_d).norm() <= 1e-2) {
    //         mpc_state_d_block.block<3, 1>(6, 0).setZero();
    //         reachedRpyD = true;
    //     }

    //     if ((mpc_state_d_block.block<3, 1>(3, 0) - state.root_pos_d).norm() <= 1e-2) {
    //         mpc_state_d_block.block<3, 1>(9, 0).setZero();
    //         reachedPosD = true;
    //     }

    //     mpc_state_d.block<13, 1>(13*i, 0) = mpc_state_d_block;
    // }

    for (int i = 0; i < MPC_HORIZON; i++) {
        mpc_state_d_block.block<3, 1>(0, 0) += mpc_state_d_block.block<3, 1>(6, 0) * DT_MPC * i;
        mpc_state_d_block.block<3, 1>(3, 0) += mpc_state_d_block.block<3, 1>(9, 0) * DT_MPC * i;

        mpc_state_d.block<13, 1>(13*i, 0) = mpc_state_d_block;
    }

}

void grfMPC::calculateQPMats(const go1State &state) {
/*
    Calculates the larger A_qp and B_qp matrices, which then get reformatted
    with the larger Q and R weight matrices to formulate the MPC problem into
    QP form, as well as populates the lower and upper bounds according to the
    desired foot contacts (default is all four feet on floor) and the linear
    constraint matrix for any additional swing leg equality constraints.
*/  
    // calculate Q and R matrices
    q_weights << q_weight_1, q_weight_2, q_weight_3, 
                q_weight_4, q_weight_5, q_weight_6, 
                q_weight_7, q_weight_8, q_weight_9, 
                q_weight_10, q_weight_11, q_weight_12, 0;

    Eigen::MatrixXd Q_diag = q_weights.asDiagonal();

    for (int i = 0; i < MPC_HORIZON; i++) {
        Q.block<13, 13>(i*13, i*13) = Q_diag;
    }

    r_weight = r_weight_val;
    R.setIdentity();
    R = r_weight*R;

    // calculate A_qp and B_qp matrices
    for (int i = 0; i < MPC_HORIZON; i++) {
        if (i == 0) {
            A_qp.block<13, 13> (0, 0) = A_mat;
        } else {
            A_qp.block<13, 13> (13*i, 0) = A_mat * A_qp.block<13, 13>(13*(i-1), 0);
        }

        for (int j = 0; j < i+1; j++) {
            if (i-j == 0) {
                B_qp.block<13, 12> (13*i, 12*j) = B_mat;
            } else {
                B_qp.block<13, 12> (13*i, 12*j) = A_qp.block<13, 13>(13*(i-j-1), 0) * B_mat;
            }
        }
    }

    // calculate Hessian P = 2.0 * (B_qp^T * Q * B_qp + R)
    Eigen::Matrix<double, 12*MPC_HORIZON, 12*MPC_HORIZON> denseP;
    denseP = 2.0 * (B_qp.transpose() * Q * B_qp + R);
    P = denseP.sparseView();

    // calculate gradient q = 2.0 * B_qp^T * Q * (A_qp * x_curr - x_ref)
    q = 2.0 * B_qp.transpose() * Q * (A_qp * mpc_state - mpc_state_d); // A_qp * x_curr

    // set up friction pyramid across entire horizon
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

    fric_ub = Eigen::VectorXd::Zero(FRIC_PYR * NUM_LEG * MPC_HORIZON);
    
    for (int i = 0; i < MPC_HORIZON; i++) {
        C_ineq.block<FRIC_PYR*NUM_LEG, 3*NUM_LEG>(i*FRIC_PYR*NUM_LEG, i*3*NUM_LEG) = C_ineq_blk;

        Eigen::VectorXd fric_ub_horiz(NUM_LEG * FRIC_PYR);
        Eigen::VectorXd fric_ub_horiz_foot(FRIC_PYR);

        for (int j = 0; j < NUM_LEG; j++) {
            if (state.contacts[j] == false && state.walking_mode == false) { // if i'm standing AND this leg is off the ground
                fric_ub_horiz_foot << 0, 0, 0, 0, 0, 0;
            } else {
                fric_ub_horiz_foot << 0, 0, 0, 0, FZ_MAX, FZ_MIN;
            }

            fric_ub_horiz.segment<FRIC_PYR>(j*FRIC_PYR) = fric_ub_horiz_foot;
        }

        fric_ub.segment<NUM_LEG * FRIC_PYR>(i * NUM_LEG * FRIC_PYR) = fric_ub_horiz;
    }

    fric_lb = Eigen::VectorXd::Constant(FRIC_PYR * NUM_LEG * MPC_HORIZON, -std::numeric_limits<double>::infinity());

    // set up swing equality constraints (only if walking)
    C_eq_blk14 << 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1; // FR + RL swinging
    C_eq_blk23 << 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0; // FL + RR swinging

    if (state.walking_mode) {
        int tempSwingPhase = state.swing_phase;
        int swingPhaseShift = DT_MPC/0.002; // will change
        int tempCurrSwingPhase = tempSwingPhase;

        for (int i = 0; i < MPC_HORIZON; i++) {
            tempCurrSwingPhase = (tempSwingPhase + i*swingPhaseShift) % SWING_PHASE_MAX;

            if (tempCurrSwingPhase <= SWING_PHASE_MAX/2) {
                C_eq.block<2, 12>(i*2, i*12) = C_eq_blk23; // FL + RR are in swing
            } else {
                C_eq.block<2, 12>(i*2, i*12) = C_eq_blk14; // FR + RL are in swing
            }
        }

        dense_C_qp.resize(FRIC_PYR*NUM_LEG*MPC_HORIZON + 2*MPC_HORIZON, MPC_INPUT_DIM);
        dense_C_qp.block<FRIC_PYR*NUM_LEG*MPC_HORIZON, MPC_INPUT_DIM>(0, 0) = C_ineq;
        // 24*7 x 12*7 - C_ineq
        // 2*7 x 12*7 - C_eq
        // 26*7 x 12*7 - C_qp
        dense_C_qp.block<2*MPC_HORIZON, MPC_INPUT_DIM>(FRIC_PYR*NUM_LEG*MPC_HORIZON, 0) = C_eq;
        C_qp = dense_C_qp.sparseView();

        swing_lb = Eigen::VectorXd::Zero(2*MPC_HORIZON);
        swing_ub = Eigen::VectorXd::Zero(2*MPC_HORIZON);

        qp_lb.resize(FRIC_PYR*NUM_LEG*MPC_HORIZON + 2*MPC_HORIZON);
        qp_ub.resize(FRIC_PYR*NUM_LEG*MPC_HORIZON + 2*MPC_HORIZON);

        qp_lb.segment<FRIC_PYR*NUM_LEG*MPC_HORIZON>(0) = fric_lb;
        qp_lb.segment<2*MPC_HORIZON>(FRIC_PYR*NUM_LEG*MPC_HORIZON) = swing_lb;
        qp_ub.segment<FRIC_PYR*NUM_LEG*MPC_HORIZON>(0) = fric_ub;
        qp_ub.segment<2*MPC_HORIZON>(FRIC_PYR*NUM_LEG*MPC_HORIZON) = swing_ub;

    } else {
        C_qp.resize(FRIC_PYR*NUM_LEG*MPC_HORIZON, MPC_INPUT_DIM);
        C_qp.setZero();
        C_qp = C_ineq.sparseView();

        qp_lb.resize(FRIC_PYR*NUM_LEG*MPC_HORIZON);
        qp_ub.resize(FRIC_PYR*NUM_LEG*MPC_HORIZON);

        qp_lb = fric_lb;
        qp_ub = fric_ub;
        
    }

}

void grfMPC::solveMPC(go1State &state) {
/*
    Solves the QP-MPC problem using a QP solver through OsqpEigen,
    then sets the ground reaction forces in the state vector to the
    first set of solved inputs.
*/
    if (!mpc_solver.isInitialized()) {
        mpc_solver.clearSolver(); // Reset solver for new constraints
        mpc_solver.settings()->setVerbosity(false);
        mpc_solver.settings()->setWarmStart(false);
        mpc_solver.data()->setNumberOfVariables(MPC_INPUT_DIM);
        mpc_solver.data()->setNumberOfConstraints(qp_lb.size());
        mpc_solver.data()->setLowerBound(qp_lb);
        mpc_solver.data()->setUpperBound(qp_ub);
        mpc_solver.data()->setHessianMatrix(P);
        mpc_solver.data()->setGradient(q);
        mpc_solver.data()->clearLinearConstraintsMatrix();
        mpc_solver.data()->setLinearConstraintsMatrix(C_qp);
        mpc_solver.initSolver();

    } else {
        // If solver is already initialized and mode hasn't changed, only update problem data
        mpc_solver.updateHessianMatrix(P);
        mpc_solver.updateGradient(q);
        mpc_solver.data()->clearLinearConstraintsMatrix();
        mpc_solver.data()->setLinearConstraintsMatrix(C_qp);
        mpc_solver.data()->setLowerBound(qp_lb);
        mpc_solver.data()->setUpperBound(qp_ub);

    }

    // Solve the MPC problem
    mpc_solver.solveProblem();
    Eigen::VectorXd solution = mpc_solver.getSolution();

    // Store the result into state.foot_forces_grf
    for (int i = 0; i < NUM_LEG; ++i) {
        if (!solution.segment<3>(i * 3).hasNaN()) {
            state.foot_forces_grf.block<3, 1>(0, i) = -solution.segment<3>(i * 3);
        }
    }

}