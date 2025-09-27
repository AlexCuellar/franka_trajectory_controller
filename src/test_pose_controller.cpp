// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_trajectory_controller/test_pose_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_trajectory_controller {

bool TestPoseController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "TestPoseController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("TestPoseController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "TestPoseController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("TestPoseController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "TestPoseController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first. ALSO BTW THIS IS ALEX'S CONTROLLER!!!!!");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "TestPoseController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void TestPoseController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);
  traj_counter = -1;
  traj.header.stamp = ros::Time::now();
  double radius = 0.15;
  ros::Duration elapsed_time_fake = ros::Duration(0.0);
  ros::Duration period_fake = ros::Duration(0.001);
  std::array<double, 16> initial_pose_fake = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  for(int i = 0; i < 2000; i++){
    double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_fake.toSec()));
    double delta_x = radius * std::sin(angle);
    double delta_z = radius * (std::cos(angle) - 1);   
    geometry_msgs::Pose p;
    p.position.x = initial_pose_fake[12] - delta_x;
    p.position.z = initial_pose_fake[14] - delta_z;
    traj.poses.push_back(p);
    // std::cout << "Angle: " << angle << " Pred x: " << traj.poses[i].position.x << " -- Pred z: " << traj.poses[i].position.z << std::endl;
    elapsed_time_fake += period_fake;
  }
  std::cout << "Num pts: " << traj.poses.size() << std::endl;
}

void TestPoseController::update(const ros::Time& /* time */, const ros::Duration& period) {
  //////////////////// ORIGINAL //////////////////////////////////
  double radius = 0.15;
  double angle_last = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
  double delta_x_last = radius * std::sin(angle_last);
  double delta_z_last = radius * (std::cos(angle_last) - 1);
  std::array<double, 16> new_pose_og_last = initial_pose_;
  new_pose_og_last[12] -= delta_x_last;
  new_pose_og_last[14] -= delta_z_last;
  elapsed_time_ += period;
  double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
  double delta_x = radius * std::sin(angle);
  double delta_z = radius * (std::cos(angle) - 1);
  std::array<double, 16> new_pose_og = initial_pose_;
  new_pose_og[12] -= delta_x;
  new_pose_og[14] -= delta_z;
  /////////////////////////////////////////////////////////////////
  traj_counter ++;
  if(traj_counter >= traj.poses.size()){
    traj_counter = traj.poses.size();
  }
  if(traj_counter < 0){
    traj_counter = 0;
  }
  std::array<double, 16> new_pose = initial_pose_;
  new_pose[12] = traj.poses[traj_counter].position.x;
  new_pose[14] = traj.poses[traj_counter].position.z;
  if(traj_counter > 0){
    double dx = new_pose[12] - traj.poses[traj_counter - 1].position.x;
    double dz = new_pose[14] - traj.poses[traj_counter - 1].position.z;
    std::cout << "t: " << traj_counter << " P: " << period << "X's: (" << new_pose_og[12] << ", " << new_pose[12] << ") Z's: (" << new_pose_og[14] << ", " << new_pose[14] << ") --- " <<  " OG dx: " << new_pose_og[12] - new_pose_og_last[12] << " NEW dx: " << dx << " --- OG dz: " << new_pose_og[14] - new_pose_og_last[14] << " NEW dz: " << dz << std::endl;
  }
  // std::cout << "t: " << traj_counter << " x: " << new_pose[12] << " z: " << new_pose[14] << std::endl;
  cartesian_pose_handle_->setCommand(new_pose_og);
}

}  

PLUGINLIB_EXPORT_CLASS(franka_trajectory_controller::TestPoseController,
                       controller_interface::ControllerBase)