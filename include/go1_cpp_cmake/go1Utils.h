#ifndef GO1_UTILS_H
#define GO1_UTILS_H

// Open-source libraries
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <functional>
#include <utility>

Eigen::Matrix3d skew(const Eigen::Vector3d &vec);
Eigen::Matrix3d rotX(const double &angle);
Eigen::Matrix3d rotY(const double &angle);
Eigen::Matrix3d rotZ(const double &angle);
Eigen::Vector3d quat2Euler(const Eigen::Quaterniond &quat);
Eigen::Matrix3d quat2RotM(const Eigen::Vector4d &quat);
Eigen::Vector3d rotM2Euler(const Eigen::Matrix3d &rotMat);
// Eigen::Matrix3d euler2RotM(const Eigen::Vector3d &rpy);

// define numerical Jacobian function in header file explicitly or make a separate .tpp file
template<class F, class... Extra>
inline Eigen::MatrixXd numericalJacobian(F&& f, const Eigen::VectorXd& x, double eps = 1e-6, Extra&&... extra) {
    auto wrapped = [&](const Eigen::VectorXd& x_var) {
        return std::invoke(std::forward<F>(f), x_var, std::forward<Extra>(extra)...);
    };

    Eigen::VectorXd f0 = wrapped(x);
    const int xlen = x.size();
    const int ylen = f0.size();
    Eigen::MatrixXd numJac(ylen, xlen);

    for (int i = 0; i < xlen; i++) {
        Eigen::VectorXd x_eps = x;
        x_eps(i) += eps;
        numJac.col(i) = (wrapped(x_eps) - f0) / eps;
    }

    return numJac;
}
// check speed of above function in EKF in threaded sim

#endif //GO1_UTILS_H