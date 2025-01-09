#pragma once

#include <Eigen/Eigen>


namespace utils 
{
    Eigen::Quaternionf frd2fluRotation(const Eigen::Quaternionf& quat_frd);
    Eigen::Vector3f frd2fluAngVel(const Eigen::Vector3f& ang_vel_frd);
    Eigen::Quaternionf ned2enuRotation(const Eigen::Quaternionf& quat_ned);
    Eigen::Vector3f ned2enuPosition(const Eigen::Vector3f& pos_ned);
    Eigen::Quaternionf nedfrd2enufluRotation(const Eigen::Quaternionf& quat_ned);
    float distance(const Eigen::Vector3f& pos1, const Eigen::Vector3f& pos2);
}