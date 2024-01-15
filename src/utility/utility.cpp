#include "utility/utility.h"

Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};
    // R0*ng1 = ng2
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    Eigen::Vector3d ypr = Utility::R2ypr(R0);
    LOG(INFO) << "c0 to ENU ypr: " << ypr.transpose();
    double yaw = Utility::R2ypr(R0).x();

    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    ypr = Utility::R2ypr(R0);
    LOG(INFO) << "c0 to ENU after yaw_offset ypr: " << ypr.transpose();

    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}
