#ifndef GO1_STATE_EST_H
#define GO1_STATE_EST_H

// Open-source libraries
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>
#include <memory>
#include <Eigen/Cholesky>
#include <filesystem>

// Package-specific header files
#include "go1Params.h"
#include "go1FK.h"
#include "go1Utils.h"
#include "go1State.h"

class go1StateEstimator {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        virtual ~go1StateEstimator() = default;
        virtual void collectInitialState(const go1State& state) = 0;
        virtual void estimateState(go1State& state) = 0;
        virtual void setFootHeightResidual(double h) {}
        virtual Eigen::VectorXd getMeasurement() = 0;
        virtual Eigen::VectorXd getPrediction() = 0;
        virtual Eigen::VectorXd getPostFitResidual() = 0;
        virtual Eigen::VectorXd getPostFitPrediction() = 0;
};

class NaiveKF : public go1StateEstimator {
    public:
        NaiveKF();
        void collectInitialState(const go1State& state) override;
        void estimateState(go1State& state) override;
        Eigen::VectorXd getMeasurement() override { return z_k; }
        Eigen::VectorXd getPrediction() override { return H_k*x_k1_getter; }
        Eigen::VectorXd getPostFitResidual() override { return z_k - H_k*x_k1; }
        Eigen::VectorXd getPostFitPrediction() override { return H_k*x_k; }

    private:
        Eigen::Matrix<double, 9, 1> x_k, x_k1;
        Eigen::Matrix<double, 9, 1> x_k1_getter; // stores x_k1 from BEFORE the Kalman update
        Eigen::Vector3d z_k, y_res;
        Eigen::Matrix<double, 9, 9> F_k, P_k, P_k1, Q_k;
        Eigen::Matrix<double, 3, 9> H_k;
        Eigen::Matrix<double, 9, 3> K_k;
        Eigen::Matrix3d R_k, S_k;
        Eigen::Matrix3d eye3;
};

class KinematicKF : public go1StateEstimator {
    public:
        KinematicKF();
        void collectInitialState(const go1State& state) override;
        void estimateState(go1State& state) override;
        void setFootHeightResidual(double h) override;
        Eigen::VectorXd getMeasurement() override { return z_k; }
        Eigen::VectorXd getPrediction() override { return H_k*x_k1_getter; }
        Eigen::VectorXd getPostFitResidual() override { return z_k - H_k*x_k1; }
        Eigen::VectorXd getPostFitPrediction() override { return H_k*x_k; }

    private:
        Eigen::Matrix<double, 18, 1> x_k, x_k1;
        Eigen::Matrix<double, 18, 1> x_k1_getter; // stores x_k1 from BEFORE the Kalman update
        Eigen::Vector3d u_k;
        Eigen::Matrix<double, 28, 1> z_k, y_res;
        Eigen::Matrix<double, 18, 18> F_k, P_k, P_k1, Q_k;
        Eigen::Matrix<double, 18, 3> B_k;
        Eigen::Matrix<double, 28, 18> H_k;
        Eigen::Matrix<double, 28, 28> R_k, S_k;
        Eigen::Matrix<double, 18, 28> K_k;
        Eigen::Matrix3d eye3;
};

class TwoStageKF : public go1StateEstimator {
    public:
        TwoStageKF();
        void collectInitialState(const go1State& state) override;
        void estimateState(go1State& state) override;
        void setFootHeightResidual(double h) override;
        Eigen::VectorXd getMeasurement() override { return z_k; }
        Eigen::VectorXd getPrediction() override { return H_k*x_k1_getter; }
        Eigen::VectorXd getPostFitResidual() override { return z_k - H_k*x_k1; }
        Eigen::VectorXd getPostFitPrediction() override { return H_k*x_k; }

    private:
        // Estimator matrices
        Eigen::Matrix<double, 18, 1> x_k, x_k1;
        Eigen::Matrix<double, 18, 1> x_k1_getter; // stores x_k1 from BEFORE the Kalman update
        Eigen::Vector3d u_k;
        Eigen::Matrix<double, 28, 1> z_k, y_res;
        Eigen::Matrix<double, 18, 18> F_k, P_k, P_k1, Q_k, Q_0;
        Eigen::Matrix<double, 18, 3> B_k;
        Eigen::Matrix<double, 28, 18> H_k;
        Eigen::Matrix<double, 28, 28> R_k, R_0, S_k;
        Eigen::Matrix<double, 18, 28> K_k;
        Eigen::Matrix3d eye3;

        // Noise constants from Muqun's code
        double process_noise_pimu = 0.02;
        double process_noise_vimu = 0.02;
        double process_noise_pfoot = 0.002;
        double sensor_noise_pimu_rel_foot = 0.001;
        double sensor_noise_vimu_rel_foot = 0.1;
        double sensor_noise_zfoot = 0.001;
};

// Nonlinear state transition function for ExtendedKF
inline Eigen::Matrix<double, 22, 1> fState(const Eigen::Matrix<double, 22, 1>& x_k, const Eigen::Vector3d& f_meas, const Eigen::Vector3d& omg_meas) {
    Eigen::Matrix<double, 22, 1> x_k1 = x_k;
    Eigen::Vector3d grav (0, 0, -9.81);
    Eigen::Matrix3d C_kT = quat2RotM(x_k.segment<4>(6)).transpose();

    auto acc_world = C_kT * f_meas + grav;
    if (!acc_world.allFinite()) {
        std::ostringstream oss;
        oss << "NaN/Inf in fState acc_world: [" << acc_world.transpose() 
            << "]\nf_meas: [" << f_meas.transpose() 
            << "]\nroot_quat: [" << x_k.segment<4>(6).transpose() << "]";
        throw std::runtime_error(oss.str());
    }

    // r_{k+1}
    x_k1.segment<3>(0) = x_k.segment<3>(0) + DT_CTRL * x_k.segment<3>(3) + 0.5 * std::pow(DT_CTRL, 2) * (C_kT * f_meas + grav);

    // v_{k+1}
    x_k1.segment<3>(3) = x_k.segment<3>(3) + DT_CTRL * (C_kT * f_meas + grav);

    // q_{k+1}
    Eigen::Quaterniond q(x_k(6), x_k(7), x_k(8), x_k(9));
    if (!q.coeffs().allFinite()) {
        throw std::runtime_error("NaN in quaternion before propagation!");
    }
    q.normalize();
    if (std::abs(q.norm() - 1) > 1e-6) q.normalize();

    Eigen::Vector3d deltaRPY = DT_CTRL * omg_meas;
    double angle = deltaRPY.norm();
    Eigen::Quaterniond dq;
    
    if (angle < 1e-8) {
        dq = Eigen::Quaterniond(1.0, 0.5 * deltaRPY.x(), 0.5 * deltaRPY.y(), 0.5 * deltaRPY.z());
    } else {
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, deltaRPY / angle));
    }

    Eigen::Quaterniond q_k1 = q * dq;
    q_k1.normalize();

    x_k1.segment<4>(6) << q_k1.w(), q_k1.x(), q_k1.y(), q_k1.z();

    return x_k1;
}

// Nonlinear measurement model function for ExtendedKF
inline Eigen::Matrix<double, 12, 1> hState(const Eigen::Matrix<double, 22, 1>& x_k1) {
    Eigen::Matrix<double, 12, 1> z_k1;
    Eigen::Matrix3d C_k = quat2RotM(x_k1.segment<4>(6));

    z_k1.segment<3>(0) = C_k * (x_k1.segment<3>(10) - x_k1.segment<3>(0));
    z_k1.segment<3>(3) = C_k * (x_k1.segment<3>(13) - x_k1.segment<3>(0));
    z_k1.segment<3>(6) = C_k * (x_k1.segment<3>(16) - x_k1.segment<3>(0));
    z_k1.segment<3>(9) = C_k * (x_k1.segment<3>(19) - x_k1.segment<3>(0));

    return z_k1;
}

class ExtendedKF : public go1StateEstimator {
    public:
        ExtendedKF();
        void collectInitialState(const go1State& state) override;
        void estimateState(go1State& state) override;
        Eigen::VectorXd getMeasurement() override { return z_k; }
        Eigen::VectorXd getPrediction() override { return hState(x_k1_getter); }
        Eigen::VectorXd getPostFitResidual() override { return z_k - hState(x_k1); }
        Eigen::VectorXd getPostFitPrediction() override { return hState(x_k); }
    
    private:
        // Generic state variables and matrices
        Eigen::Matrix<double, 22, 1> x_k, x_k1;
        Eigen::Matrix<double, 22, 1> x_k1_getter; // stores x_k1 from BEFORE the Kalman update
        Eigen::Matrix<double, 12, 1> z_k, y_res;
        Eigen::Matrix<double, 12, 12> R_k, S_k;
        Eigen::Matrix3d eye3;

        // // Numerical method-related matrices
        // Eigen::Matrix<double, 22, 22> F_k, P_k, P_k1, F_P_k, K_H_P_k, Q_k;
        // Eigen::Matrix<double, 22, 12> K_k;
        // Eigen::Matrix<double, 12, 22> H_k, H_P, S_k_invT;
        // Eigen::LDLT<decltype(S_k)> S_k_ldlt;
        
        // Analytical (manual) method-related matrices
        Eigen::Matrix<double, 21, 1> delta_x_k1_man;
        Eigen::Matrix<double, 21, 21> F_k_man, Q_k_man, P_k_man, P_k1_man;
        Eigen::Matrix<double, 21, 12> K_k_man;
        Eigen::Matrix<double, 12, 21> H_k_man;

        // Empirical noise covariance calculations
};

// virtual interface pointer function to create designated estimator object
inline std::unique_ptr<go1StateEstimator> makeEstimator() {
    switch (STATE_EST_SELECT) {
        case 0: return std::make_unique<NaiveKF>();
        case 1: return std::make_unique<KinematicKF>();
        case 2: return std::make_unique<TwoStageKF>();
        case 3: return std::make_unique<ExtendedKF>();
        default: throw std::runtime_error("Invalid STATE_EST_SELECT");
    }
}

#endif // GO1_STATE_EST_H