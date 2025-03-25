#include "go1_cpp_cmake/go1Utils.h"

/*
    Just a script for some convenient functions. This will expand as
    we explore walking and footstep planning.
*/

Eigen::Matrix3d skew(const Eigen::Vector3d &vec) {
/*
    Turns a 3x1 column vector into a skew-symmetric matrix as a faster
    way to take the cross-product of two vectors (a x b = a_skew * b)
*/
    Eigen::Matrix3d skewMat;
    skewMat << 0, -vec.z(), vec.y(),
                vec.z(), 0, -vec.x(),
                -vec.y(), vec.x(), 0;

    return skewMat;
}

Eigen::Matrix3d rotX(const double &angle) {
/*
    Finds the rotation matrix around the x-axis by an angle (in radians).
*/
    Eigen::Matrix3d rotMat;
    rotMat << 1, 0, 0,
            0, cos(angle), -sin(angle),
            0, sin(angle), cos(angle);
    
    return rotMat;
}

Eigen::Matrix3d rotY(const double &angle) {
/*
    Finds the rotation matrix around the y-axis by an angle (in radians).
*/
    Eigen::Matrix3d rotMat;
    rotMat << cos(angle), 0, sin(angle),
            0, 1, 0,
            -sin(angle), 0, cos(angle);
    
    return rotMat;
}

Eigen::Matrix3d rotZ(const double &angle) {
/*
    Finds the rotation matrix around the z-axis by an angle (in radians).
*/
    Eigen::Matrix3d rotMat;
    rotMat << cos(angle), -sin(angle), 0,
            sin(angle), cos(angle), 0,
            0, 0, 1;
    
    return rotMat;
}

Eigen::Vector3d quat2Euler(const Eigen::Quaterniond &quat) {
/*
    Converts the quaternion to roll-pitch-yaw (Euler) angles.
*/
    Eigen::Vector3d euler;

    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (quat.w() * quat.x() + quat.y() * quat.z());
    double cosr_cosp = 1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y());
    euler(0) = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (quat.w() * quat.y() - quat.z() * quat.x());
    if (std::abs(sinp) >= 1)
        euler(1) = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    else
        euler(1) = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (quat.w() * quat.z() + quat.x() * quat.y());
    double cosy_cosp = 1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z());
    euler(2) = std::atan2(siny_cosp, cosy_cosp);

    return euler;
}

Eigen::Matrix3d quat2RotM(const Eigen::Vector4d &quat) {
/*
    Converts the quaternion in vector form to a rotation matrix.
*/
    Eigen::Quaterniond q(quat(0), quat(1), quat(2), quat(3));
    return q.toRotationMatrix();
}

Eigen::Vector3d rotM2Euler(const Eigen::Matrix3d &rotMat) {
/*
    Converts a rotation matrix to roll-pitch-yaw (Euler) angles.
*/
   if (std::abs(rotMat(2, 0)) < 1.0) {
        double yaw = std::atan2(rotMat(1, 0), rotMat(0, 0));    // Rotation around Z-axis
        double pitch = std::asin(-rotMat(2, 0));                // Rotation around Y-axis
        double roll = std::atan(rotMat(2, 1)/ rotMat(2, 2));    // Rotation around X-axis
        return Eigen::Vector3d(roll, pitch, yaw);

    } else {
        // Handle gimbal lock case
        double yaw = std::atan(-rotMat(0, 1)/ rotMat(1, 1));
        double pitch = (rotMat(2, 0) > 0) ? -M_PI_2 : M_PI_2; // +/- 90 degrees
        double roll = 0; // Undefined, set to zero
        return Eigen::Vector3d(roll, pitch, yaw);

    }
} 