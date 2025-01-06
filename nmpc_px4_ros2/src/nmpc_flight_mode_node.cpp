#include "rclcpp/rclcpp.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include "nmpc_px4_ros2/nmpc_flight_mode_node.hpp"

using MyNodeWithMode = px4_ros2::NodeWithMode<NMPCFlightMode>;

static const std::string kNodeName = "nmpc_flight_mode";
static const bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNodeWithMode>(kNodeName, kEnableDebugOutput));
  rclcpp::shutdown();
  return 0;
}