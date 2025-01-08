#include "nmpc_px4_ros2_utils/utils.hpp"

namespace utils 
{
    Eigen::Quaternionf frd2fluRotation(const Eigen::Quaternionf& quat_frd)
    {
        return quat_frd * Eigen::Quaternionf {0, 1, 0, 0};
    }

    Eigen::Vector3f frd2fluAngVel(const Eigen::Vector3f& ang_vel_frd)
    {
        return Eigen::Vector3f {ang_vel_frd(0), -ang_vel_frd(1), -ang_vel_frd(2)};
    }

    Eigen::Quaternionf ned2enuRotation(const Eigen::Quaternionf& quat_ned)
    {
        return Eigen::Quaternionf {0, sqrt(2)/2, sqrt(2)/2, 0} * quat_ned;
    }

    Eigen::Vector3f ned2enuPosition(const Eigen::Vector3f& pos_ned)
    {
        return Eigen::Vector3f {pos_ned(1), pos_ned(0), -pos_ned(2)};
    }

    Eigen::Quaternionf nedfrd2enufluRotation(const Eigen::Quaternionf& quat_ned)
    {
        Eigen::Quaternionf quat_ned_flu = frd2fluRotation(quat_ned);
        Eigen::Quaternionf quat_enu_flu = ned2enuRotation(quat_ned_flu);
        return quat_enu_flu;
    }
}