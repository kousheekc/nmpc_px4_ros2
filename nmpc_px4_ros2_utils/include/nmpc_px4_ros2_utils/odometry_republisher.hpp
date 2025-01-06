#include <memory>
#include <Eigen/Eigen>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "px4_msgs/msg/vehicle_odometry.hpp"

class OdometryRepublisher : public rclcpp::Node
{
  public:
    OdometryRepublisher();

  private:
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _px4_odom_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _ros2_odom_pub;

  private:
    void _odomCallback(const px4_msgs::msg::VehicleOdometry & msg);
    // TODO: Define this somewhere common because it is used in many places across packages
    Eigen::Quaternionf _nedfrd2enuflu(const Eigen::Quaternionf& quat_ned) const;
};