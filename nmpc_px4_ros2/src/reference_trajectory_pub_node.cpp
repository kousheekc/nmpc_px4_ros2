#include <chrono>
#include <fstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("ref_traj_pub")
    {
        _ref_traj_pub = this->create_publisher<nav_msgs::msg::Path>("ref_traj", 10);
        _timer = this->create_wall_timer(5000ms, std::bind(&MinimalPublisher::timer_callback, this));

        std::string ref_traj_path = ament_index_cpp::get_package_share_directory("nmpc_px4_ros2") + "/config/traj/" + this->get_parameter("ref_traj").as_string() + ".txt";
        _ref_traj_len = readDataFromFile(ref_traj_path.c_str(), _ref_traj);
        RCLCPP_INFO(this->get_logger(), "Reference path with '%d' points", _ref_traj_len);
    }

  private:
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _ref_traj_pub;

    int _ref_traj_len;
    std::vector<std::vector<double>> _ref_traj;
    nav_msgs::msg::Path _ref_traj_msg;

  private:
    void timer_callback()
    {
      _ref_traj_msg.header.frame_id = "world";
      _ref_traj_msg.header.stamp = this->get_clock()->now();
      for (auto & state : _ref_traj)
      {
        auto pose = geometry_msgs::msg::PoseStamped();
        pose.header.frame_id = "world";
        pose.header.stamp = this->get_clock()->now();
        pose.pose.position.x = state[0];
        pose.pose.position.y = state[1];
        pose.pose.position.z = state[2];
        _ref_traj_msg.poses.push_back(pose);
      }
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
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}