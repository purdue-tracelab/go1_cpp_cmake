#ifndef GO1_GRFMPC_H
#define GO1_GRFMPC_H

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

class grfMPC {
    public:
        grfMPC(); // constructs the object, sets up the friction pyramid constraints
        void reset();
        void calculateA(const Eigen::Vector3d &root_rpy);
        void calculateB(const Eigen::Matrix3d &inertia, const Eigen::Vector3d &root_rpy, const Eigen::Matrix<double, 3, NUM_LEG> foot_pos);
        void calculateMPCStates(const go1State &state);
        void calculateQPMats(const go1State &state);
        void solveMPC(go1State &state);

        // weights
        Eigen::Matrix<double, 13, 1> q_weights;
        double r_weight;
        Eigen::Matrix<double, MPC_REF_DIM, MPC_REF_DIM> Q;
        Eigen::Matrix<double, 3*NUM_LEG*MPC_HORIZON, 3*NUM_LEG*MPC_HORIZON> R;

        // QP ZOH matrices
        Eigen::Matrix<double, 13, 13> A_mat; // 13x13
        Eigen::Matrix<double, 13, 12> B_mat; // 13x12

        // QP-MPC ZOH matrices
        Eigen::Matrix<double, MPC_REF_DIM, 13> A_qp; // (13*MPC_HORIZON)x13
        Eigen::Matrix<double, MPC_REF_DIM, MPC_INPUT_DIM> B_qp; // (13*MPC_HORIZON)x(12*MPC_HORIZON)
        Eigen::Matrix<double, 13, 1> mpc_state; // state reformulated in RBM form [13x1]
        Eigen::Matrix<double, 13, 1> mpc_state_d_block; // desired state reformulated in RBM form [13x1]
        Eigen::Matrix<double, MPC_REF_DIM, 1> mpc_state_d; // desired state reformulated in QP-MPC form [(13*MPC_HORIZON)x1]

        // standard QP formulation
        // minimize 1/2 * x' * P * x + q' * x
        // subject to lb <= Ac * x <= ub
        Eigen::SparseMatrix<double> P; // P = B_qp^T * Q * B_qp + R, the Hessian of the QP problem [(12MPC_HORIZON)x(12MPC_HORIZON)]
        Eigen::Matrix<double, MPC_INPUT_DIM, 1> q; // q , the gradient of the QP problem [(12MPC_HORIZON)x1]
        Eigen::VectorXd qp_lb;
        Eigen::VectorXd qp_ub;

        Eigen::MatrixXd dense_C_qp;
        Eigen::SparseMatrix<double> C_qp; // stacked C_ineq and C_eq for all constraints
        Eigen::Matrix<double, FRIC_PYR*NUM_LEG*MPC_HORIZON, MPC_INPUT_DIM> C_ineq; // handles the frictional pyramid constraints
        Eigen::VectorXd fric_lb;
        Eigen::VectorXd fric_ub;

        Eigen::Matrix<double, 2*MPC_HORIZON, MPC_INPUT_DIM> C_eq; // handles the swing leg equality constraints (sets swing leg stance forces to 0 in MPC)
        Eigen::Matrix<double, 2, 12> C_eq_blk14;
        Eigen::Matrix<double, 2, 12> C_eq_blk23;
        Eigen::VectorXd swing_lb;
        Eigen::VectorXd swing_ub;

        OsqpEigen::Solver mpc_solver;
};

#endif //GO1_GRFMPC_H
