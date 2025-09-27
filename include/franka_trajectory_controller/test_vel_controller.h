// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/subscriber.h>
#include <geometry_msgs/PoseArray.h>

#include <realtime_tools/realtime_buffer.h>

namespace franka_trajectory_controller {

class TestVelController : public controller_interface::MultiInterfaceController<
                                               franka_hw::FrankaVelocityCartesianInterface,
                                               franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;
  int traj_idx;
  bool done_with_traj;
  double epsilon;
  double dt;
  double print_rate;
  double time_since_print;
  double vel_;
  double v_max_;
  double a_max_;
  std::array<double, 6> last_command;
  void traj_callback(const geometry_msgs::PoseArrayConstPtr& msg);
  
 private:
  franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;
  ros::Duration elapsed_time_;
  std::string arm_id_;
  geometry_msgs::PoseArray traj;
  ros::Subscriber sub_;
  ros::Publisher pose_pub_;

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  realtime_tools::RealtimeBuffer<geometry_msgs::PoseArray> command_buffer_;

};

}  // namespace franka_example_controllers

// X from .3 to .15, Z from .48 to .62