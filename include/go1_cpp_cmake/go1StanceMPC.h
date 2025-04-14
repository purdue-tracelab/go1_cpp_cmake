#ifndef GO1_STANCE_MPC_H
#define GO1_STANCE_MPC_H

// Standard libraries
#include <vector>
#include <chrono>

// Open-source libraries
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>

// Package-specific header files
#include "go1State.h"
#include "go1Params.h"
#include "go1Utils.h"

void go1StanceMPC(go1State &state); // standalone version of go1MPC class below

class go1MPC { // class-based version of go1StanceMPC function above
    public:
        go1MPC();
        void reset();
        void updateRigidBodyModel(const go1State& state);
        void updateMPCStates(const go1State& state);
        void solveMPCForces(go1State& state);

        // weights
        Eigen::Matrix<double, 13, 1> q_weights;
        Eigen::SparseMatrix<double> Q;
        Eigen::SparseMatrix<double> R;

        // Continuous- and discrete-time dynamics matrices
        Eigen::Matrix<double, 13, 13> A_mat; // 13x13
        Eigen::Matrix<double, 13, 13> A_mat_d;
        Eigen::Matrix<double, 13, 12> B_mat; // 13x12
        Eigen::Matrix<double, 13, 12> B_mat_d;

        // MPC matrices
        Eigen::Matrix<double, MPC_REF_DIM, 13> A_qp; // (13*MPC_HORIZON)x13
        Eigen::Matrix<double, MPC_REF_DIM, MPC_INPUT_DIM> B_qp; // (13*MPC_HORIZON)x(12*MPC_HORIZON)
        Eigen::Matrix<double, 13, 1> mpc_state; // state reformulated in RBM form [13x1]
        Eigen::Matrix<double, 13, 1> mpc_state_d_block; // desired state reformulated in RBM form [13x1]
        Eigen::Matrix<double, MPC_REF_DIM, 1> mpc_state_d; // desired state reformulated in QP-MPC form [(13*MPC_HORIZON)x1]

        // MPC matrices (sparse versions)
        Eigen::SparseMatrix<double> A_qp_sparse; // (13*MPC_HORIZON)x13
        Eigen::SparseMatrix<double> B_qp_sparse; // (13*MPC_HORIZON)x(12*MPC_HORIZON)
        Eigen::SparseMatrix<double> B_qp_T_Q_sparse; // (12*MPC_HORIZON)x(12*MPC_HORIZON)

        // standard QP formulation
        // minimize 1/2 * U^T * P * U + U^T * q
        // subject to lb <= C_qp * U <= ub
        Eigen::SparseMatrix<double> P; // P = B_qp^T * Q * B_qp + R, the Hessian of the QP problem [(12*MPC_HORIZON)x(12*MPC_HORIZON)]
        Eigen::Matrix<double, MPC_INPUT_DIM, 1> q; // q = 2.0 * B_qp^T * Q * (A_qp * x_curr - x_ref), the gradient of the QP problem [(12*MPC_HORIZON)x1]
        Eigen::VectorXd qp_lb;
        Eigen::VectorXd qp_ub;
        Eigen::Matrix<double, FRIC_PYR*NUM_LEG*MPC_HORIZON + 2*MPC_HORIZON, MPC_INPUT_DIM> dense_C_qp;
        Eigen::SparseMatrix<double> C_qp; // stacked C_ineq and C_eq for all constraints

        // friction pyramid inequality constraints
        Eigen::Matrix<double, FRIC_PYR*NUM_LEG*MPC_HORIZON, MPC_INPUT_DIM> C_ineq;
        Eigen::Matrix<double, FRIC_PYR*NUM_LEG, 3*NUM_LEG> C_ineq_blk;

        OsqpEigen::Solver mpc_solver;
        Eigen::Matrix<double, 3, NUM_LEG> grf_forces;
};

#endif // GO1_STANCE_MPC_H