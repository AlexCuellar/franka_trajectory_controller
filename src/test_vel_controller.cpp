// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <traj_control/test_vel_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <algorithm>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace traj_control {

bool TestVelController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  std::string arm_id_;
  if (!node_handle.getParam("arm_id", arm_id_)) {
    ROS_ERROR("TestVelController: Could not get parameter arm_id_");
    return false;
  }
  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "TestVelController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id_ + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "TestVelController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("TestVelController: Could not get state interface from hardware");
    return false;
  }

  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
            state_interface->getHandle(arm_id_ + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "TestVelController: Exception getting state handle: " << e.what());
    return false;
  }

  /////////////// ALEX STUFF //////////////////////
  std::array<double, 16> initial_pose = state_handle_->getRobotState().O_T_EE_d;
  traj.poses.clear();
  std::array<double, 6> last_command = {{0, 0.0, 0, 0.0, 0.0, 0.0}};
  pose_pub_ = node_handle.advertise<geometry_msgs::Pose>("current_pose", 10);
  ROS_INFO("CREATING CONTROLLER SUBSCRIBER");
  sub_ = node_handle.subscribe<geometry_msgs::PoseArray>("trajectories", 1, &TestVelController::traj_callback, this);
  ROS_INFO("CONTROLLER SUBSCRIBER CREATED");
  done_with_traj = true;
  epsilon = 0.002;
  return true;
}

void TestVelController::traj_callback(const geometry_msgs::PoseArrayConstPtr& msg) {
  if(!done_with_traj){
    ROS_INFO("Received new trajectory but still executing previous trajectory, ignoring new trajectory.");
    return;
  }
  traj = *msg;
  command_buffer_.writeFromNonRT(traj);
  std::cout << "Received new trajectory with " << traj.poses.size() << " points." << std::endl;
  std::cout << "First point: (" << traj.poses[0].position.x << ", " << traj.poses[0].position.y << ", " << traj.poses[0].position.z << ")" << std::endl;
  done_with_traj = false;
}

void TestVelController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  // Convert rotation matrix to quaternion
}

void TestVelController::update(const ros::Time& /* time */, const ros::Duration& period) {
  std::array<double, 16> current_pose = state_handle_->getRobotState().O_T_EE_d;
  if(done_with_traj){
    // std::array<double, 6> command = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    // velocity_cartesian_handle_->setCommand(command);
    const geometry_msgs::PoseArray& traj = *command_buffer_.readFromRT();
    geometry_msgs::Pose current_pose_msg;
    current_pose_msg.position.x = current_pose[12];
    current_pose_msg.position.y = current_pose[13];
    current_pose_msg.position.z = current_pose[14];
    pose_pub_.publish(current_pose_msg);
    return;
  }
  double v_max = 0.2;
  double a_max = 0.0005;
  double x_curr = current_pose[12];
  double y_curr = current_pose[13];
  double z_curr = current_pose[14];
  double x_des = traj.poses[traj_idx].position.x;
  double y_des = traj.poses[traj_idx].position.y;
  double z_des = traj.poses[traj_idx].position.z;
  double dx = x_des - x_curr;
  double dy = y_des - y_curr;
  double dz = z_des - z_curr;
  double dist_to_goal_squared = (x_curr - x_des)*(x_curr - x_des) + (y_curr - y_des)*(y_curr - y_des) + (z_curr - z_des)*(z_curr - z_des);
  if(dist_to_goal_squared < epsilon*epsilon){
    traj_idx++;
    x_des = traj.poses[traj_idx].position.x;
    y_des = traj.poses[traj_idx].position.y;
    z_des = traj.poses[traj_idx].position.z;
    dx = x_des - x_curr;
    dy = y_des - y_curr;
    dz = z_des - z_curr;
    if(traj_idx >= traj.poses.size()){
      ROS_INFO("Done with trajectory, waiting for new trajectory...");
      traj.poses.clear();
      traj_idx = 0;
      done_with_traj = true;
      return;
    }
  }
  double norm = std::max({std::sqrt(dx*dx + dy*dy + dz*dz), .000001});
  dx = v_max*dx/norm;
  dy = v_max*dy/norm;
  dz = v_max*dz/norm;
  std::array<double, 6> command = {{dx, dy, dz, 0.0, 0.0, 0.0}};
  if(dist_to_goal_squared < epsilon*epsilon){
    std::cout << "t: " << traj_idx << " v: " << v_max << " x_curr: " << x_curr << " y_curr: " << y_curr << " z_curr: " << z_curr << " -- x_des: " << x_des << " y_des: " << y_des << " z_des: " << z_des << " -- dx: " << dx << " dy: " << dy << " dz: " << dz << " dist^2: " << dist_to_goal_squared << std::endl;
  }
  if(command[0] - last_command[0] >  a_max){command[0] = last_command[0] + a_max;}
  if(command[0] - last_command[0] < -a_max){command[0] = last_command[0] - a_max;}
  if(command[1] - last_command[1] >  a_max){command[1] = last_command[1] + a_max;}
  if(command[1] - last_command[1] < -a_max){command[1] = last_command[1] - a_max;}
  if(command[2] - last_command[2] >  a_max){command[2] = last_command[2] + a_max;}
  if(command[2] - last_command[2] < -a_max){command[2] = last_command[2] - a_max;}
  velocity_cartesian_handle_->setCommand(command);
  last_command = command;
}

void TestVelController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(traj_control::TestVelController,
                       controller_interface::ControllerBase)
