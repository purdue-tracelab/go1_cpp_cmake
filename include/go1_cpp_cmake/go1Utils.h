#ifndef GO1_UTILS_H
#define GO1_UTILS_H

// Open-source libraries
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <iomanip>

Eigen::Matrix3d skew(const Eigen::Vector3d &vec);
Eigen::Matrix3d rotX(const double &angle);
Eigen::Matrix3d rotY(const double &angle);
Eigen::Matrix3d rotZ(const double &angle);
Eigen::Vector3d quat2Euler(const Eigen::Quaterniond &quat);
Eigen::Matrix3d quat2RotM(const Eigen::Vector4d &quat);
Eigen::Vector3d rotM2Euler(const Eigen::Matrix3d &rotMat);

#endif //GO1_UTILS_H