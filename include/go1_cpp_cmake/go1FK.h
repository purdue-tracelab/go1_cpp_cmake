#ifndef GO1_FWD_KIN_H
#define GO1_FWD_KIN_H

// Open-source libraries
#include <Eigen/Dense>
#include <mujoco/mujoco.h>

// Package-specific header files
#include "go1Params.h"
#include "go1Utils.h"

Eigen::Matrix4d transformMat(const Eigen::Matrix3d &rotMat, const double dispX, const double dispY, const double dispZ);
Eigen::Matrix<double, 3, NUM_LEG> go1BaseFrameWorldRotFwdKin(const Eigen::VectorXd& jointPos, const Eigen::Vector3d& root_rpy);
Eigen::MatrixXd go1WorldFrameContactJacobian(const Eigen::VectorXd& jointPos, const Eigen::Vector3d& root_rpy);
Eigen::Matrix3d go1HipFrameLegJacobian(int leg_idx, const Eigen::VectorXd& jointPos);

#endif //GO1_FWD_KIN_H