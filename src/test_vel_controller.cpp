// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_trajectory_controller/test_vel_controller.h>

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

namespace franka_trajectory_controller {

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
  pose_pub_ = node_handle.advertise<geometry_msgs::PoseStamped>("current_pose", 1);
  ROS_INFO("CREATING CONTROLLER SUBSCRIBER");
  sub_ = node_handle.subscribe<geometry_msgs::PoseArray>("trajectories", 1, &TestVelController::traj_callback, this);
  ROS_INFO("CONTROLLER SUBSCRIBER CREATED");
  done_with_traj = true;
  epsilon = 0.002;
  dt = 0.3;
  print_rate = .05;
  time_since_print = 0.0;
  v_max_ = 0.15;
  vel_ = v_max_*.5;
  a_max_ = 0.0004; 
  return true;
}

void TestVelController::traj_callback(const geometry_msgs::PoseArrayConstPtr& msg) {
  if(!done_with_traj){
    ROS_INFO("Received new trajectory but still executing previous trajectory, ignoring new trajectory.");
    return;
  }
  traj = *msg;
  std::cout << "Received new trajectory with " << traj.poses.size() << " points." << std::endl;
  std::cout << "First pointime: (" << traj.poses[0].position.x << ", " << traj.poses[0].position.y << ", " << traj.poses[0].position.z << ")" << std::endl;
  done_with_traj = false;
  traj_idx = 0;
  time_since_print = 0.0;
  elapsed_time_ = ros::Duration(0.0);
  command_buffer_.writeFromNonRT(traj);
}

void TestVelController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  // Convert rotation matrix to quaternion
}

void TestVelController::update(const ros::Time& /* time */, const ros::Duration& period) {
  // ESTABLISH CURRENT POSE AND TIME
  std::array<double, 16> current_pose = state_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ += period;
  double time = elapsed_time_.toSec();
  geometry_msgs::PoseStamped current_pose_msg;
  double x_curr;
  double y_curr;
  double z_curr;
  double x_des;
  double y_des;
  double z_des;
  double dx;
  double dy;
  double dz;
  double dist_to_goal_squared;
  std::array<double, 6> command = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  // IF DONE WITH TRAJ, SEND ZERO VELOCITIES AND SEND CURRENT POSE
  if(done_with_traj){
    const geometry_msgs::PoseArray& traj = *command_buffer_.readFromRT();
    current_pose_msg.pose.position.x = current_pose[12];
    current_pose_msg.pose.position.y = current_pose[13];
    current_pose_msg.pose.position.z = current_pose[14];
    current_pose_msg.header.stamp = ros::Time::now();
    current_pose_msg.header.frame_id = "world";
    pose_pub_.publish(current_pose_msg);
  }
  else{
    // GET CURRENT AND DESIRED POSE, COMPUTE DISTANCE TO GOAL
    x_curr = current_pose[12];
    y_curr = current_pose[13];
    z_curr = current_pose[14];
    x_des = traj.poses[traj_idx].position.x;
    y_des = traj.poses[traj_idx].position.y;
    z_des = traj.poses[traj_idx].position.z;
    dx = x_des - x_curr;
    dy = y_des - y_curr;
    dz = z_des - z_curr;
    dist_to_goal_squared = (x_curr - x_des)*(x_curr - x_des) + (y_curr - y_des)*(y_curr - y_des) + (z_curr - z_des)*(z_curr - z_des);
    // IF NOT REACHED FIRST POINT IN THE TRAJECTORY, DO NOT COUNT UP TIME
    if(traj_idx == 0 and dist_to_goal_squared > epsilon*epsilon){
      ROS_INFO("Not at first point yet, waiting to start trajectory...");
      elapsed_time_ = ros::Duration(0.0);
      time = 0.0;
    }
    // IF REACHED GOAL, SWITCH TO NEXT POINT
    if(dist_to_goal_squared < epsilon*epsilon){
      ROS_INFO("Triggered By Time: trigger_time: %f, time: %f, idx: %d", dt*(traj_idx), time, traj_idx);
      traj_idx++;
      // RECALCULATE GOAL AND DISTANCE TO GOAL
      x_des = traj.poses[traj_idx].position.x;
      y_des = traj.poses[traj_idx].position.y;
      z_des = traj.poses[traj_idx].position.z;
      dx = x_des - x_curr;
      dy = y_des - y_curr;
      dz = z_des - z_curr;
      dist_to_goal_squared = (x_curr - x_des)*(x_curr - x_des) + (y_curr - y_des)*(y_curr - y_des) + (z_curr - z_des)*(z_curr - z_des);
      // SET VELOCITY BASED ON DISTANCE TO GOAL AND TIME TO NEXT POINT
      vel_ = std::min({v_max_, std::sqrt(dist_to_goal_squared) / std::max({dt*(traj_idx) - time, .00001})});
      // IF AT END OF TRAJ, SET DONE FLAG
      if(traj_idx >= traj.poses.size()){
        ROS_INFO("Done with trajectory, waiting for new trajectory...");
        done_with_traj = true;
        return;
      }
    }
    // COMPUTE AND SEND COMMAND
    // Normalize direction and multiply by vel_
    double norm = std::max({std::sqrt(dx*dx + dy*dy + dz*dz), .000001});
    dx = vel_*dx/norm;
    dy = vel_*dy/norm;
    dz = vel_*dz/norm;
    command[0] = dx;
    command[1] = dy;
    command[2] = dz; 
  }
  // Enforce acceleration limits
  if(command[0] - last_command[0] >  a_max_){command[0] = last_command[0] + a_max_;}
  if(command[0] - last_command[0] < -a_max_){command[0] = last_command[0] - a_max_;}
  if(command[1] - last_command[1] >  a_max_){command[1] = last_command[1] + a_max_;}
  if(command[1] - last_command[1] < -a_max_){command[1] = last_command[1] - a_max_;}
  if(command[2] - last_command[2] >  a_max_){command[2] = last_command[2] + a_max_;}
  if(command[2] - last_command[2] < -a_max_){command[2] = last_command[2] - a_max_;}
  // PRINT INFO
  if(time - time_since_print > print_rate){
    time_since_print = time;
    if(done_with_traj){
      ROS_INFO("Done with trajectory, publishing current pose: (%f, %f, %f) stamp: %f", current_pose_msg.pose.position.x, current_pose_msg.pose.position.y, current_pose_msg.pose.position.z, current_pose_msg.header.stamp.toSec());
    }
    else{
      double v_err = std::sqrt((command[0] - dx)*(command[0] - dx) + (command[1] - dy)*(command[1] - dy) + (command[2] - dz)*(command[2] - dz));
      std::cout << "time: " << time << " done_with_traj: " << done_with_traj << " traj_idx: " << traj_idx << " vel: " << vel_ << " v_err: " << v_err << " dist: " << std::sqrt(dist_to_goal_squared) << " -- dx: " << dx << " dy: " << dy << " dz: " << dz << " -- x_des: " << x_des << " y_des: " << y_des << " z_des: " << z_des << std::endl;
    }
  }
  // SEND COMMAND
  velocity_cartesian_handle_->setCommand(command);
  last_command = command;
}

void TestVelController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_trajectory_controller::TestVelController,
                       controller_interface::ControllerBase)
