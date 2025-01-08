#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>

using namespace std::chrono_literals; // NOLINT

class ExternalStateEstimationInterface : public px4_ros2::LocalPositionMeasurementInterface
{
public:
  explicit ExternalStateEstimationInterface(rclcpp::Node & node)
  : LocalPositionMeasurementInterface(node, px4_ros2::PoseFrame::LocalNED,
      px4_ros2::VelocityFrame::LocalNED)
  {
    _node.declare_parameter("state_estimation_topic", "/Robot_1/pose");
    _pose_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>(node.get_parameter("state_estimation_topic").as_string(), 10, std::bind(&ExternalStateEstimationInterface::_poseCallback, this, std::placeholders::_1));


    RCLCPP_INFO(_node.get_logger(), "External state estimation node running");
  }

  void _poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    px4_ros2::LocalPositionMeasurement local_position_measurement {};

    local_position_measurement.timestamp_sample = _node.get_clock()->now();

    local_position_measurement.position_xy = Eigen::Vector2f {msg->pose.position.x, msg->pose.position.y};
    local_position_measurement.position_xy_variance = Eigen::Vector2f {0.1f, 0.1f};

    local_position_measurement.position_z = msg->pose.position.z;
    local_position_measurement.position_z_variance = 0.1F;

    local_position_measurement.attitude_quaternion = Eigen::Quaternionf {msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z};
    local_position_measurement.attitude_variance = Eigen::Vector3f {0.1, 0.1, 0.1};

    try {
      update(local_position_measurement);
      RCLCPP_INFO_ONCE(_node.get_logger(), "Successfully sent position update to navigation interface.");
    } 
    catch (const px4_ros2::NavigationInterfaceInvalidArgument & e) {
      RCLCPP_ERROR_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000, "Exception caught: %s", e.what());
    }
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_sub;

private:

};

class ExternalStateEstimationNode : public rclcpp::Node
{
public:
  ExternalStateEstimationNode()
  : Node("ext_state_est_node")
  {
    // Enable debug output
    auto ret = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (ret != RCUTILS_RET_OK) 
    {
      RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
      rcutils_reset_error();
    }

    _interface = std::make_unique<ExternalStateEstimationInterface>(*this);

    if (!_interface->doRegister()) 
    {
      throw std::runtime_error("Registration failed");
    }
  }

private:
  std::unique_ptr<ExternalStateEstimationInterface> _interface;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExternalStateEstimationNode>());
  rclcpp::shutdown();
  return 0;
}

