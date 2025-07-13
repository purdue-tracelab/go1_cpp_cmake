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
        virtual double getKalmanGainNorm() = 0;
};

/////////////////////////////////////////////

class NaiveKF : public go1StateEstimator {
    public:
        NaiveKF();
        void collectInitialState(const go1State& state) override;
        void estimateState(go1State& state) override;
        Eigen::VectorXd getMeasurement() override { return z_k; }
        Eigen::VectorXd getPrediction() override { return H_k*x_k1_getter; }
        Eigen::VectorXd getPostFitResidual() override { return z_k - H_k*x_k1; }
        Eigen::VectorXd getPostFitPrediction() override { return H_k*x_k; }
        double getKalmanGainNorm() override { return K_k.norm(); }

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

/////////////////////////////////////////////

class MIT_TwoStageKF : public go1StateEstimator {
    public:
        MIT_TwoStageKF();
        void collectInitialState(const go1State& state) override;
        void estimateState(go1State& state) override;
        void setFootHeightResidual(double h) override;
        Eigen::VectorXd getMeasurement() override { return z_k; }
        Eigen::VectorXd getPrediction() override { return H_k*x_k1_getter; }
        Eigen::VectorXd getPostFitResidual() override { return z_k - H_k*x_k1; }
        Eigen::VectorXd getPostFitPrediction() override { return H_k*x_k; }
        double getKalmanGainNorm() override { return K_k.norm(); }

    private:
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

/////////////////////////////////////////////

// Nonlinear state transition function for ETHZ_EKF
inline Eigen::Matrix<double, 22, 1> fState_ETHZ(const Eigen::Matrix<double, 22, 1>& x_k, const Eigen::Vector3d& f_meas, const Eigen::Vector3d& omg_meas, const Eigen::Matrix3d& C_k) {
    Eigen::Matrix<double, 22, 1> x_k1 = x_k; // implicitly passes foot positions through
    Eigen::Vector3d grav (0, 0, -9.81);

    auto acc_world = C_k * f_meas + grav;
    if (!acc_world.allFinite()) {
        std::ostringstream oss;
        oss << "NaN/Inf in fState_ETHZ acc_world: [" << acc_world.transpose() 
            << "]\nf_meas: [" << f_meas.transpose() 
            << "]\nroot_quat: [" << x_k.segment<4>(6).transpose() << "]";
        throw std::runtime_error(oss.str());
    }

    // r_{k+1}
    x_k1.segment<3>(0) = x_k.segment<3>(0) + DT_CTRL * x_k.segment<3>(3) + 0.5 * std::pow(DT_CTRL, 2) * (acc_world);

    // v_{k+1}
    x_k1.segment<3>(3) = x_k.segment<3>(3) + DT_CTRL * (acc_world);

    // q_{k+1}
    Eigen::Quaterniond q(x_k(6), x_k(7), x_k(8), x_k(9));
    if (!q.coeffs().allFinite()) {
        throw std::runtime_error("NaN in quaternion before propagation!");
    }
    q.normalize();
    if (std::abs(q.norm() - 1) > 1e-6) q.normalize();

    Eigen::Vector3d deltaRPY = DT_CTRL * omg_meas;
    
    Eigen::AngleAxisd rollAngle(deltaRPY(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(deltaRPY(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(deltaRPY(2), Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond dq = yawAngle * pitchAngle * rollAngle;

    Eigen::Quaterniond q_k1 = q * dq;
    q_k1.normalize();

    x_k1.segment<4>(6) << q_k1.w(), q_k1.x(), q_k1.y(), q_k1.z();

    return x_k1;
}

// Nonlinear measurement model function for ETHZ_EKF
inline Eigen::Matrix<double, 12, 1> hState_ETHZ(const Eigen::Matrix<double, 22, 1>& x_k1, const Eigen::Matrix3d& C_k) {
    Eigen::Matrix<double, 12, 1> z_k1;
    Eigen::Matrix3d C_kT = C_k.transpose();

    z_k1.segment<3>(0) = C_kT * (x_k1.segment<3>(10) - x_k1.segment<3>(0));
    z_k1.segment<3>(3) = C_kT * (x_k1.segment<3>(13) - x_k1.segment<3>(0));
    z_k1.segment<3>(6) = C_kT * (x_k1.segment<3>(16) - x_k1.segment<3>(0));
    z_k1.segment<3>(9) = C_kT * (x_k1.segment<3>(19) - x_k1.segment<3>(0));

    return z_k1;
}

class ETHZ_EKF_man : public go1StateEstimator {
    public:
        ETHZ_EKF_man();
        void collectInitialState(const go1State& state) override;
        void estimateState(go1State& state) override;
        Eigen::VectorXd getMeasurement() override { return z_k; }
        Eigen::VectorXd getPrediction() override { return hState_ETHZ(x_k1_getter, C_k); }
        Eigen::VectorXd getPostFitResidual() override { return z_k - hState_ETHZ(x_k1, C_k); }
        Eigen::VectorXd getPostFitPrediction() override { return hState_ETHZ(x_k, C_k); }
        double getKalmanGainNorm() override { return K_k.norm(); }
    
    private:
        // Generic state variables and matrices
        Eigen::Matrix<double, 22, 1> x_k, x_k1;
        Eigen::Matrix<double, 22, 1> x_k1_getter; // stores x_k1 from BEFORE the Kalman update
        Eigen::Matrix<double, 12, 1> z_k, y_res;
        Eigen::Matrix<double, 12, 12> R_k, S_k;
        Eigen::Matrix3d eye3, C_k;
        
        // Analytical Jacobian-related matrices
        Eigen::Matrix<double, 21, 1> delta_x_k1;
        Eigen::Matrix<double, 21, 21> F_k, Q_k, P_k, P_k1;
        Eigen::Matrix<double, 21, 12> K_k;
        Eigen::Matrix<double, 12, 21> H_k;

        // Optimized gains for estimation from MATLAB script
        double q_f;
        double q_w;
        double q_p;
        double r_meas;
};

/////////////////////////////////////////////

class ETHZ_EKF_num : public go1StateEstimator {
    public:
        ETHZ_EKF_num();
        void collectInitialState(const go1State& state) override;
        void estimateState(go1State& state) override;
        Eigen::VectorXd getMeasurement() override { return z_k; }
        Eigen::VectorXd getPrediction() override { return hState_ETHZ(x_k1_getter, C_k); }
        Eigen::VectorXd getPostFitResidual() override { return z_k - hState_ETHZ(x_k1, C_k); }
        Eigen::VectorXd getPostFitPrediction() override { return hState_ETHZ(x_k, C_k); }
        double getKalmanGainNorm() override { return K_k.norm(); }
    
    private:
        // Generic state variables and matrices
        Eigen::Matrix<double, 22, 1> x_k, x_k1;
        Eigen::Matrix<double, 22, 1> x_k1_getter; // stores x_k1 from BEFORE the Kalman update
        Eigen::Matrix<double, 12, 1> z_k, y_res;
        Eigen::Matrix<double, 12, 12> R_k, S_k;
        Eigen::Matrix3d eye3, C_k;
        
        // Numerical Jacobian-related matrices
        Eigen::Matrix<double, 22, 22> F_k, Q_k, P_k, P_k1;
        Eigen::Matrix<double, 22, 12> K_k;
        Eigen::Matrix<double, 12, 22> H_k, H_P_trans, K_k_trans;
};

// virtual interface pointer function to create designated estimator object
inline std::unique_ptr<go1StateEstimator> makeEstimator() {
    switch (STATE_EST_SELECT) {
        case 0: return std::make_unique<NaiveKF>();
        case 1: return std::make_unique<MIT_TwoStageKF>();
        case 2: return std::make_unique<ETHZ_EKF_man>();
        case 3: return std::make_unique<ETHZ_EKF_num>();
        default: throw std::runtime_error("Invalid STATE_EST_SELECT");
    }
}

#endif // GO1_STATE_EST_H

/*
// Retired nonlinear state transition function for CMU_EKF
inline Eigen::Matrix<double, 22, 1> fState_CMU(const Eigen::Matrix<double, 22, 1>& x_k, const Eigen::Vector3d& f_meas, const Eigen::Vector3d& omg_meas) {
    Eigen::Matrix<double, 22, 1> x_k1 = x_k;
    Eigen::Vector3d grav (0, 0, -9.81);
    Eigen::Matrix3d C_kT = quat2RotM(x_k.segment<4>(6)).transpose();

    auto acc_world = C_kT * f_meas + grav;
    if (!acc_world.allFinite()) {
        std::ostringstream oss;
        oss << "NaN/Inf in fState_CMU acc_world: [" << acc_world.transpose() 
            << "]\nf_meas: [" << f_meas.transpose() 
            << "]\nroot_quat: [" << x_k.segment<4>(6).transpose() << "]";
        throw std::runtime_error(oss.str());
    }

    // r_{k+1}
    x_k1.segment<3>(0) = x_k.segment<3>(0) + DT_CTRL * x_k.segment<3>(3) + 0.5 * std::pow(DT_CTRL, 2) * (acc_world);

    // v_{k+1}
    x_k1.segment<3>(3) = x_k.segment<3>(3) + DT_CTRL * (acc_world);

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

// Retired nonlinear measurement model function for CMU_EKF
inline Eigen::Matrix<double, 24, 1> hState_CMU(const Eigen::Matrix<double, 22, 1>& x_k1) {
    Eigen::Matrix<double, 24, 1> z_k1;
    Eigen::Matrix3d C_k = quat2RotM(x_k1.segment<4>(6));
    
    for (int i = 0; i < NUM_LEG; i++) {
        z_k1.segment<3>(i*3) = C_k * (x_k1.segment<3>(10 + i*3) - x_k1.segment<3>(0));
        z_k1.segment<3>(12 + i*3) = C_k * x_k1.segment<3>(3);
    }

    return z_k1;
}
*/
