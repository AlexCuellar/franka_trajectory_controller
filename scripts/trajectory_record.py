#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from std_msgs.msg import Header, Float32MultiArray
from franka_msgs.msg import FrankaState
import csv
import sys
import select
import termios
import tty
from copy import deepcopy
import os

class PoseRecorder:
    def __init__(self):
        rospy.init_node('pose_recorder', anonymous=True)

        # Parameters
        self.pose_topic = rospy.get_param('~pose_topic', '/franka_state_controller/franka_states')
        self.output_topic = rospy.get_param('~output_topic', '/recorded_trajectories')
        self.label_topic = rospy.get_param('~label_topic', '/recorded_labels')
        self.output_csv = rospy.get_param('~output_csv', 'recorded_poses.csv')
        self.base_frame = rospy.get_param('~base_frame', 'panda_link0')

        # ROS pub/sub
        self.pose_sub = rospy.Subscriber(self.pose_topic, FrankaState, self.pose_callback)
        self.pose_pub = rospy.Publisher(self.output_topic, PoseArray, queue_size=10)
        self.label_pub = rospy.Publisher(self.label_topic, Float32MultiArray, queue_size=10)

        # Internal buffers
        self.latest_pose = None
        self.poses = []
        self.labels = []          # list of [l1, l2, l3]
        self.active_labels = [0,0,0]  # current one-hot state
        self.recording = False

        rospy.loginfo("PoseRecorder initialized. "
                      "Press 'r' to start/stop, '1'/'2'/'3' to toggle labels, 'q' to quit.")

    def pose_callback(self, msg):
        """Callback for pose updates."""
        self.latest_pose = msg.O_T_EE
        pose = Pose()
        pose.position.x = self.latest_pose[12]
        pose.position.y = self.latest_pose[13]
        pose.position.z = self.latest_pose[14]
        if self.recording:
            self.poses.append(pose)
            print("Recorded pose: position=({}, {}, {}), labels={}".format(
                pose.position.x, pose.position.y, pose.position.z, self.active_labels))
            self.labels.append(list(self.active_labels))

    def toggle_recording(self):
        """Start or stop trajectory recording."""
        self.recording = not self.recording
        if self.recording:
            rospy.loginfo("Recording started...")
            self.poses = []
            self.labels = []
            self.active_labels = [0,0,0]
        else:
            print("Would you like to save the recorded trajectory? (y/n)")
            key = ''
            while key not in ['y','n']:
                key = sys.stdin.read(1)
                print(key)
                if key == 'n':
                    rospy.loginfo("Recording discarded.")
                    self.poses = []
                    self.labels = []
                    self.active_labels = [0,0,0]
                if key == 'y':
                    rospy.loginfo("Saving and publishing...")
                    self.save_to_csv()
                    self.publish_pose_array()
                    self.publish_labels()

    def toggle_label(self, idx):
        """Toggle a label index (1,2,3)."""
        if all([self.active_labels[i] == 0 for i in range(3) if i != idx]):
            self.active_labels[idx] = 1 - self.active_labels[idx]
            rospy.loginfo("Label {} toggled -> {}".format(idx+1, self.active_labels))
        else:
            rospy.loginfo("Only one label can be active at a time. Clear current labels first.")

    def save_to_csv(self):
        """Save poses and labels to CSV."""
        if not self.poses:
            rospy.logwarn("No poses recorded; skipping save.")
            return
        directory = os.path.dirname(self.output_csv)
        if directory and not os.path.exists(directory):
            os.makedirs(directory)

        with open(self.output_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x','y','z','qx','qy','qz','qw','label1','label2','label3'])
            for pose, lbl in zip(self.poses, self.labels):
                writer.writerow([
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w,
                    lbl[0], lbl[1], lbl[2]
                ])
        rospy.loginfo("Saved {} poses to {}".format(len(self.poses), self.output_csv))

    def publish_pose_array(self):
        """Publish PoseArray of recorded trajectory."""
        if not self.poses:
            return
        msg = PoseArray()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.base_frame
        msg.poses = self.poses
        self.pose_pub.publish(msg)
        rospy.loginfo("Published PoseArray with {} poses on {}".format(len(self.poses), self.output_topic))

    def publish_labels(self):
        """Publish all label entries as a Float32MultiArray."""
        if not self.labels:
            return
        msg_array = []
        for pose, lbl in zip(self.poses, self.labels):
            msg_array += [pose.position.x, pose.position.y, pose.position.z] + lbl
        msg = Float32MultiArray()
        # Flatten list: [[1,0,0],[0,1,0]] → [1,0,0,0,1,0]
        msg.data = msg_array
        self.label_pub.publish(msg)
        rospy.loginfo("Published {} label triplets on {}".format(len(self.labels), self.label_topic))

    def keyboard_listener(self):
        """Listen for keyboard commands."""
        old_attrs = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == 'r':
                        self.toggle_recording()
                    elif key in ['1','2','3']:
                        self.toggle_label(int(key)-1)
                    elif key == 'q':
                        if self.recording:
                            self.recording = False
                            self.save_to_csv()
                            self.publish_pose_array()
                            self.publish_labels()
                        rospy.signal_shutdown("User quit.")
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attrs)


if __name__ == '__main__':
    node = PoseRecorder()
    node.keyboard_listener()
