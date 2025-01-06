#include "nmpc_px4_ros2_utils/odometry_republisher.hpp"

OdometryRepublisher::OdometryRepublisher() 
: Node("odometry_republisher_node")
{
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    _px4_odom_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos_profile, std::bind(&OdometryRepublisher::_odomCallback, this, std::placeholders::_1));
    _ros2_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
}

void OdometryRepublisher::_odomCallback(const px4_msgs::msg::VehicleOdometry & msg)
{
  Eigen::Quaternionf quat_ned(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
  Eigen::Quaternionf quat_enu = _nedfrd2enuflu(quat_ned);

  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.frame_id = "world";
  odom_msg.header.stamp = this->get_clock()->now();
  odom_msg.pose.pose.position.x = msg.position[1];
  odom_msg.pose.pose.position.y = msg.position[0];
  odom_msg.pose.pose.position.z = -msg.position[2];
  odom_msg.pose.pose.orientation.w = quat_enu.w();
  odom_msg.pose.pose.orientation.x = quat_enu.x();
  odom_msg.pose.pose.orientation.y = quat_enu.y();
  odom_msg.pose.pose.orientation.z = quat_enu.z();
  _ros2_odom_pub->publish(odom_msg);
}

Eigen::Quaternionf OdometryRepublisher::_nedfrd2enuflu(const Eigen::Quaternionf& quat_ned) const 
{
  Eigen::Quaternionf rotation_flu(0, 1, 0, 0);
  Eigen::Quaternionf rotation_enu(0, sqrt(2)/2, sqrt(2)/2, 0);
  Eigen::Quaternionf quat_ned_flu = quat_ned * rotation_flu;
  Eigen::Quaternionf quat_enu_flu = rotation_enu * quat_ned_flu;
  return quat_enu_flu;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryRepublisher>());
  rclcpp::shutdown();
  return 0;
}