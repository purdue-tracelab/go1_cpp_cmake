#ifndef GO1_FK_H
#define GO1_FK_H

// Open-source libraries
#include <Eigen/Dense>
#include <mujoco/mujoco.h>

// Package-specific header files
#include "go1Params.h"
#include "go1Utils.h"

Eigen::Matrix4d transformMat(const Eigen::Matrix3d &rotMat, const double dispX, const double dispY, const double dispZ);
Eigen::Matrix<double, 3, NUM_LEG> go1FK(const mjtNum* q_vec, const Eigen::Vector3d &root_rpy);
Eigen::MatrixXd go1FootJac(const mjtNum* q_vec, const Eigen::Vector3d &root_rpy);

#endif //GO1_FK_H