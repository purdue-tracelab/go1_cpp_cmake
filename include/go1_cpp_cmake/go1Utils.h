#ifndef GO1_UTILS_H
#define GO1_UTILS_H

// Open-source libraries
#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <utility>
#include <deque>
#include <string>
#include <fstream>
#include <mutex>
#include <condition_variable>
#include <type_traits>
#include <thread>
#include <iostream>

#include "go1Params.h"

// Math helper functions
Eigen::Matrix3d skew(const Eigen::Vector3d &vec);

Eigen::Matrix3d rotX(const double &angle);
Eigen::Matrix3d rotY(const double &angle);
Eigen::Matrix3d rotZ(const double &angle);

Eigen::Vector3d quat2Euler(const Eigen::Quaterniond &quat);
Eigen::Matrix3d quat2RotM(const Eigen::Vector4d &quat);
Eigen::Vector3d rotM2Euler(const Eigen::Matrix3d &rotMat);
Eigen::Quaterniond quatMult(const Eigen::Quaterniond &quat1, const Eigen::Quaterniond &quat2);
Eigen::Quaterniond euler2Quat(const Eigen::Vector3d &vec);

Eigen::Matrix3d computeGamma0(const Eigen::Vector3d& gyro);

double hipJointIK(double pFutY, double pFutZ, double hipLength);
double calfJointIK(double thighLength, double calfLength, double futDistEff);
double thighJointIK(double hipTheta, double calfTheta, Eigen::Vector3d pFut, double thighLength, double calfLength);
Eigen::Vector3d computeHipFrameFutIK(int leg_idx, Eigen::Vector3d pFut);
Eigen::Vector3d computeNewtonIK(Eigen::Matrix3d legJacobian, Eigen::Vector3d dx);

double computeDihedralAngle(const Eigen::Vector3d &planeNormal);

// Asynchronous data storage class
class AsyncLogger {
    public:
        AsyncLogger(const std::string &path, const std::string &header);
        ~AsyncLogger();
        void logLine(std::string &&line);
      
      private:
        void loop();
      
        std::ofstream               out_;
        std::deque<std::string>     queue_;           // <-- non‑static member
        std::mutex                  mtx_;
        std::condition_variable     cv_;
        bool                        running_;
        std::thread                 thread_;
};

// Syncronous data storage class
class SyncLogger {
    public:
        SyncLogger(const std::string &path, const std::string &header);
        ~SyncLogger();
        void logLine(const std::string &line);
        void flush();

    private:
        std::ofstream out_;
};

// numerical Jacobian functions, implemented in separate .tpp file
template<class F, class... Extra>
Eigen::MatrixXd numericalJacobian(F&& f, const Eigen::VectorXd& x, double eps = 1e-6, bool central = false, Extra&&... extra);

template<int M, int N, class F, class... Extra>
Eigen::Matrix<double, M, N> numericalJacobianFixedSize(F&& f, const Eigen::Matrix<double, N, 1>& x, double eps, bool central, Extra&&... extra);

#include "numericalJacobian.tpp"

#endif //GO1_UTILS_H