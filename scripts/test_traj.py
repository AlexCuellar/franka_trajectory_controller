#!/usr/bin/env python3

import rospy
import tf.transformations
import numpy as np

from geometry_msgs.msg import PoseArray, Pose

class TestTrajNode:
    def __init__(self):
        print("STARTING TEST TRAJ NODE")
        rospy.init_node('test_traj', anonymous=True)
        self.pub = rospy.Publisher('test_vel_controller/trajectories', PoseArray, queue_size=10)
        self.sub = rospy.Subscriber('test_vel_controller/current_pose', Pose, self.get_pose_callback)
        self.rate = rospy.Rate(1) # 1 Hz

        self.robot_init_pose = None
        self.traj = PoseArray()
        self.traj.header.frame_id = "world"


    def get_pose_callback(self, pose):
        if self.robot_init_pose is None:
            print("Received pose: position=({}, {}, {}), orientation=({}, {}, {}, {})".format(
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            ))
            self.robot_init_pose = pose

    def main(self):
        # Define a trajectory that maps out a circle of radius 0.1
        while True:
            print("Waiting for initial pose...")
            while self.robot_init_pose is None:
                rospy.sleep(1)
            radius = 0.1
            num_points = 20
            points = []
            center_x = self.robot_init_pose.position.x + radius
            center_y = self.robot_init_pose.position.y
            center_z = self.robot_init_pose.position.z + .1
            for i in range(num_points):
                angle = 2 * np.pi * i / num_points
                x = center_x - radius * np.cos(angle)
                y = center_y + radius * np.sin(angle)
                z = center_z + radius * np.sin(angle) 
                points.append((x, y, z))

            for point in points:
                pose = Pose()
                pose.position.x = point[0]
                pose.position.y = point[1]
                pose.position.z = point[2]
                # Orientation as a quaternion (no rotation)
                quat = tf.transformations.quaternion_from_euler(0, 0, 0)
                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]
                self.traj.poses.append(pose)
            self.traj.poses *= 5
            self.traj.header.stamp = rospy.Time.now()
            self.pub.publish(self.traj)
            self.robot_init_pose = None
            self.traj.poses = []

        #### Uncomment below to continuously publish the trajectory
        # while not rospy.is_shutdown():
        #     print("Publishing trajectory with {} points".format(len(self.traj.poses)))
        #     self.traj.header.stamp = rospy.Time.now()
        #     pub.publish(self.traj)
        #     rate.sleep()

if __name__ == '__main__':
    try:
        n = TestTrajNode()
        n.main()
    except rospy.ROSInterruptException:
        pass