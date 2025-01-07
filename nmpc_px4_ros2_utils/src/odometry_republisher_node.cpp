#include <memory>
#include <Eigen/Eigen>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include "nmpc_px4_ros2_utils/utils.hpp"

class OdometryRepublisher : public rclcpp::Node
{
  public:
    OdometryRepublisher()
    : Node("odometry_republisher_node")
    {
        auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        _px4_odom_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos_profile, std::bind(&OdometryRepublisher::_odomCallback, this, std::placeholders::_1));
        _ros2_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    }

  private:
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _px4_odom_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _ros2_odom_pub;

  private:
    void _odomCallback(const px4_msgs::msg::VehicleOdometry & msg)
    {
      Eigen::Quaternionf quat_ned(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
      Eigen::Quaternionf quat_enu = utils::nedfrd2enufluRotation(quat_ned);

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
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryRepublisher>());
  rclcpp::shutdown();
  return 0;
}