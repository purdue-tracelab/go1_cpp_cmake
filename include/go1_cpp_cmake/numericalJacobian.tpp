#pragma once

#include <Eigen/Dense>
#include <functional>
#include <type_traits>

template<class F, class... Extra>
Eigen::MatrixXd numericalJacobian(F&& f, const Eigen::VectorXd& x, double eps, bool central, Extra&&... extra) {
    auto wrapped = [&](const Eigen::VectorXd& xx) {
        return std::invoke(std::forward<F>(f), xx, std::forward<Extra>(extra)...);
    };

    Eigen::VectorXd f0 = wrapped(x);
    const int n = x.size();
    const int m = f0.size();
    Eigen::MatrixXd numJac(m, n);

    Eigen::VectorXd x_eps = x;
    Eigen::VectorXd f1(m), f2(m);

    for (int i = 0; i < n; i++) {
        double delta = eps * std::max(1.0, std::abs(x[i]));

        if (central) {
            x_eps[i] = x[i] + delta;
            f1 = wrapped(x_eps);

            x_eps[i] = x[i] - delta;
            f2 = wrapped(x_eps);

            numJac.col(i) = (f1 - f2) / (2.0 * delta);
            
        } else {
            x_eps[i] = x[i] + delta;
            f1 = wrapped(x_eps);

            numJac.col(i) = (f1 - f0) / delta;
        }

        x_eps[i] = x[i];

    }

    return numJac;
}

template<int M, int N, class F, class... Extra>
Eigen::Matrix<double, M, N> numericalJacobianFixedSize(F&& f, const Eigen::Matrix<double, N, 1>& x, double eps, bool central, Extra&&... extra) {
    Eigen::Matrix<double, M, 1> f0 = f(x, std::forward<Extra>(extra)...);
    Eigen::Matrix<double, M, N> numJac;
    Eigen::Matrix<double, N, 1> x_eps = x;
    Eigen::Matrix<double, M, 1> f1, f2;

    for (int i = 0; i < N; i++) {
        double delta = eps * std::max(1.0, std::abs(x[i]));

        if (central) {
            x_eps[i] = x[i] + delta; f1 = f(x_eps, extra...);
            x_eps[i] = x[i] - delta; f2 = f(x_eps, extra...);
            numJac.col(i) = (f1 - f2) / (2.0 * delta);

        } else {
            x_eps[i] = x[i] + delta; f1 = f(x_eps, extra...);
            numJac.col(i) = (f1 - f0) / delta;
        }

        x_eps[i] = x[i];

    }

    return numJac;
}