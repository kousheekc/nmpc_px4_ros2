#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/direct_actuators.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/angular_velocity.hpp>
#include <px4_ros2/odometry/attitude.hpp>

#include <Eigen/Eigen>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "nmpc_px4_ros2_interfaces/msg/state_trajectory.hpp"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_nmpc_flight_mode.h"

#include "nmpc_px4_ros2_utils/utils.hpp"

#define N      NMPC_FLIGHT_MODE_N
#define NX     NMPC_FLIGHT_MODE_NX
#define NU     NMPC_FLIGHT_MODE_NU
#define NY     NMPC_FLIGHT_MODE_NY

using namespace std::chrono_literals; // NOLINT
static const std::string kName = "NMPC Flight Mode";

class NMPCFlightMode : public px4_ros2::ModeBase
{
public:
  explicit NMPCFlightMode(rclcpp::Node & node)
  : ModeBase(node, kName), _node(node)
  {
    _node.declare_parameter("ref_traj", "circle");

    _ref_traj_sub = node.create_subscription<nmpc_px4_ros2_interfaces::msg::StateTrajectory>("/ref_traj", 10, std::bind(&NMPCFlightMode::_refTrajCallback, this, std::placeholders::_1));

    _thrust_setpoint = std::make_shared<px4_ros2::DirectActuatorsSetpointType>(*this);
    _vehicle_local_position_velocity = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _vehicle_angular_velocity = std::make_shared<px4_ros2::OdometryAngularVelocity>(*this);
    _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

    acados_ocp_capsule = nmpc_flight_mode_acados_create_capsule();
    status = nmpc_flight_mode_acados_create_with_discretization(acados_ocp_capsule, N, NULL);
    if (status)
    {
        RCLCPP_ERROR(_node.get_logger(), "nmpc_flight_mode_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }
    nlp_config = nmpc_flight_mode_acados_get_nlp_config(acados_ocp_capsule);
    nlp_dims = nmpc_flight_mode_acados_get_nlp_dims(acados_ocp_capsule);
    nlp_in = nmpc_flight_mode_acados_get_nlp_in(acados_ocp_capsule);
    nlp_out = nmpc_flight_mode_acados_get_nlp_out(acados_ocp_capsule);
    nlp_solver = nmpc_flight_mode_acados_get_nlp_solver(acados_ocp_capsule);
    nlp_opts = nmpc_flight_mode_acados_get_nlp_opts(acados_ocp_capsule);
  }

  void onActivate() override 
  {
    iter = 0;
    std::fill(std::begin(prev_u), std::end(prev_u), 4.9033);
  }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    // TODO: Implement state machine for various options: 1. Approach trajectory if too far but near enough, track trajectory, hold last position
    // TODO: Throw error if starting point of trajectory too far away from current position
    if(iter < ref_traj_len-N)
    {
      Eigen::Vector3f pos_ned = _vehicle_local_position_velocity->positionNed();
      Eigen::Quaternionf quat_ned = _vehicle_attitude->attitude();
      Eigen::Vector3f lin_vel_ned = _vehicle_local_position_velocity->velocityNed();
      Eigen::Vector3f ang_vel_frd = _vehicle_angular_velocity->angularVelocityFrd();

      Eigen::Vector3f pos_enu = utils::ned2enuPosition(pos_ned);
      Eigen::Quaternionf quat_enu = utils::nedfrd2enufluRotation(quat_ned);
      Eigen::Vector3f lin_vel_enu = utils::ned2enuPosition(lin_vel_ned);
      Eigen::Vector3f ang_vel_flu = utils::frd2fluAngVel(ang_vel_frd);

      double x_init[NX];
      x_init[0] = pos_enu(0);
      x_init[1] = pos_enu(1);
      x_init[2] = pos_enu(2);
      x_init[3] = quat_enu.w();
      x_init[4] = quat_enu.x();
      x_init[5] = quat_enu.y();
      x_init[6] = quat_enu.z();
      x_init[7] = lin_vel_enu(0);
      x_init[8] = lin_vel_enu(1);
      x_init[9] = lin_vel_enu(2);
      x_init[10] = ang_vel_flu(0);
      x_init[11] = ang_vel_flu(1);
      x_init[12] = ang_vel_flu(2);

      ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x_init);
      ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x_init);
      ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbu", prev_u);
      ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubu", prev_u);

      for (int j = 0; j < N; j++)
      {
        double ref_state[17];
        std::copy_n(ref_traj[iter+j].begin(), 17, ref_state);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, j, "yref", ref_state);
      }
      double ref_state_e[17];
      std::copy_n(ref_traj[iter+N].begin(), 13, ref_state_e);
			ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", ref_state_e);

      double xtraj[NX * (N+1)];
      double utraj[NU * N];
      status = nmpc_flight_mode_acados_solve(acados_ocp_capsule);
      for (int ii = 0; ii <= N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
      for (int ii = 0; ii < N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);
      
      auto optimal_path_msg = nav_msgs::msg::Path();
      optimal_path_msg.header.frame_id = "world";
      optimal_path_msg.header.stamp = _node.get_clock()->now();
      for (int ii = 0; ii <= N; ii++)
      {
        auto pose = geometry_msgs::msg::PoseStamped();
        pose.header.frame_id = "map";
        pose.header.stamp = _node.get_clock()->now();
        pose.pose.position.x = xtraj[ii*NX+0];
        pose.pose.position.y = xtraj[ii*NX+1];
        pose.pose.position.z = xtraj[ii*NX+2];
        optimal_path_msg.poses.push_back(pose);
      }
      _optimal_traj_pub->publish(optimal_path_msg);


      Eigen::Matrix<float, 12, 1> setpoint;
      int set = 1;
      // RCLCPP_INFO(_node.get_logger(), "Iter %d, thrust setpoint: %f, %f, %f, %f", iter, sqrt(utraj[4*set+1]/8.580775e-06)/1000, sqrt(utraj[4*set+3]/8.580775e-06)/1000, sqrt(utraj[4*set+2]/8.580775e-06)/1000, sqrt(utraj[4*set+0]/8.580775e-06)/1000);
      // RCLCPP_INFO(_node.get_logger(), "Iter %d, prev u: %f, %f, %f, %f", iter, prev_u[0], prev_u[2], prev_u[3], prev_u[1]);
      
      std::copy(utraj + set * 4, utraj + set * 4 + 4, prev_u);
      setpoint << sqrt(utraj[4*set+1]/8.580775e-06)/1000, sqrt(utraj[4*set+3]/8.580775e-06)/1000, sqrt(utraj[4*set+2]/8.580775e-06)/1000, sqrt(utraj[4*set+0]/8.580775e-06)/1000, std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1");
      // setpoint << 0.0, 0.0, 1.0, 0.0, std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1");
      _thrust_setpoint->updateMotors(setpoint);
      iter++;
    }
  }

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _optimal_traj_pub;
  rclcpp::Subscription<nmpc_px4_ros2_interfaces::msg::StateTrajectory>::SharedPtr _ref_traj_sub;

  std::shared_ptr<px4_ros2::DirectActuatorsSetpointType> _thrust_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position_velocity;
  std::shared_ptr<px4_ros2::OdometryAngularVelocity> _vehicle_angular_velocity;
  std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;

  void *nlp_opts;
  ocp_nlp_solver *nlp_solver;
  ocp_nlp_out *nlp_out;
  ocp_nlp_in *nlp_in;
  ocp_nlp_dims *nlp_dims;
  ocp_nlp_config *nlp_config;
  nmpc_flight_mode_solver_capsule *acados_ocp_capsule;
  int status;
  int iter;
  double prev_u[NU];
  int ref_traj_len;
  std::vector<std::vector<double>> ref_traj;

private:
  void _refTrajCallback(const nmpc_px4_ros2_interfaces::msg::StateTrajectory & msg)
  {
    ref_traj_len = msg.len.data;
    for (int i = 0; i < ref_traj_len; i++)
    {
      std::vector<double> state;
      state.push_back(msg.poses[i].position.x);
      state.push_back(msg.poses[i].position.y);
      state.push_back(msg.poses[i].position.z);
      state.push_back(msg.poses[i].orientation.w);
      state.push_back(msg.poses[i].orientation.x);
      state.push_back(msg.poses[i].orientation.y);
      state.push_back(msg.poses[i].orientation.z);
      state.push_back(msg.linear_velocities[i].x);
      state.push_back(msg.linear_velocities[i].y);
      state.push_back(msg.linear_velocities[i].z);
      state.push_back(msg.angular_velocities[i].x);
      state.push_back(msg.angular_velocities[i].y);
      state.push_back(msg.angular_velocities[i].z);
      state.push_back(msg.control_inputs[i].data[0]);
      state.push_back(msg.control_inputs[i].data[1]);
      state.push_back(msg.control_inputs[i].data[2]);
      state.push_back(msg.control_inputs[i].data[3]);
      ref_traj.push_back(state);
    }
  }

  float _thrust2rpm(float thrust) const
  {
    return sqrt(thrust/8.580775e-06)/1000;
  }
};
