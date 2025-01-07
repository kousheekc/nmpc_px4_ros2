#include <chrono>
#include <fstream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include "nmpc_px4_ros2_interfaces/msg/state_trajectory.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

class ReferenceTrajectoryPublisher : public rclcpp::Node
{
  public:
    ReferenceTrajectoryPublisher()
    : Node("ref_traj_pub_node")
    {
      this->declare_parameter("ref_traj", "circle");

      _ref_traj_pub = this->create_publisher<nmpc_px4_ros2_interfaces::msg::StateTrajectory>("ref_traj", 10);
      _ref_traj_viz_pub = this->create_publisher<nav_msgs::msg::Path>("ref_traj_viz", 10);
      _timer = this->create_wall_timer(5000ms, std::bind(&ReferenceTrajectoryPublisher::timer_callback, this));

      std::string ref_traj_path = ament_index_cpp::get_package_share_directory("nmpc_px4_ros2_utils") + "/config/traj/" + this->get_parameter("ref_traj").as_string() + ".txt";
      _ref_traj_len = readDataFromFile(ref_traj_path.c_str(), _ref_traj);
      _initMsg();
      RCLCPP_INFO(this->get_logger(), "Reference path with %d points found", _ref_traj_len);
    }

  private:
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<nmpc_px4_ros2_interfaces::msg::StateTrajectory>::SharedPtr _ref_traj_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _ref_traj_viz_pub;

    int _ref_traj_len;
    std::vector<std::vector<double>> _ref_traj;
    nmpc_px4_ros2_interfaces::msg::StateTrajectory _ref_traj_msg;
    nav_msgs::msg::Path _ref_traj_viz_msg;

  private:
    void _initMsg()
    {
      _ref_traj_msg.len.data = _ref_traj_len;
      for (auto & state : _ref_traj)
      {
        auto pose = geometry_msgs::msg::Pose();
        pose.position.x = state[0];
        pose.position.y = state[1];
        pose.position.z = state[2];
        pose.orientation.w = state[3];
        pose.orientation.x = state[4];
        pose.orientation.y = state[5];
        pose.orientation.z = state[6];

        auto lin_vel = geometry_msgs::msg::Vector3();
        lin_vel.x = state[7];
        lin_vel.y = state[8];
        lin_vel.z = state[9];

        auto ang_vel = geometry_msgs::msg::Vector3();
        ang_vel.x = state[10];
        ang_vel.y = state[11];
        ang_vel.z = state[12];

        auto control_inputs = std_msgs::msg::Float32MultiArray();
        control_inputs.data.push_back(state[13]);
        control_inputs.data.push_back(state[14]);
        control_inputs.data.push_back(state[15]);
        control_inputs.data.push_back(state[16]);

        _ref_traj_msg.poses.push_back(pose);
        _ref_traj_msg.linear_velocities.push_back(lin_vel);
        _ref_traj_msg.angular_velocities.push_back(ang_vel);
        _ref_traj_msg.control_inputs.push_back(control_inputs);
      }
    }
    void timer_callback()
    {
      // TODO: having to publish two msgs representing the same thing because rviz can only visualise nav_msgs::msg::Path
      _ref_traj_viz_msg.header.frame_id = "world";
      _ref_traj_viz_msg.header.stamp = this->get_clock()->now();
      for (auto & state : _ref_traj)
      {
        auto pose = geometry_msgs::msg::PoseStamped();
        pose.header.frame_id = "world";
        pose.header.stamp = this->get_clock()->now();
        pose.pose.position.x = state[0];
        pose.pose.position.y = state[1];
        pose.pose.position.z = state[2];
        _ref_traj_viz_msg.poses.push_back(pose);
      }
      _ref_traj_viz_pub->publish(_ref_traj_viz_msg);
      _ref_traj_pub->publish(_ref_traj_msg);
    }

    int readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data)
    {
      std::ifstream file(fileName);
      std::string line;
      int num_of_steps = 0;

      if (file.is_open())
      {
        while(getline(file, line))
        {
          ++num_of_steps;
          std::istringstream linestream( line );
          std::vector<double> linedata;
          double number;

          while( linestream >> number )
          {
            linedata.push_back( number );
          }
          data.push_back( linedata );
        }

        file.close();
      }
      else
      {
        return 0;
      }
      return num_of_steps;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReferenceTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}