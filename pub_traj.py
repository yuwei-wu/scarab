#!/usr/bin/env python

import rospy
import pickle
import time
from datetime import datetime
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf_trans
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA



def load_trajectory(filepath):
    with open(filepath, 'rb') as f:

        rospy.loginfo(f"Loading trajectory from {filepath}...")

        file = pickle.load(f)
        #print(f"Loaded trajectory file: {file}")

        # it is a list of points, and convert into a single list

        num_robot = len(file)
        traj = file[0]


        #down sample the trajectory to reduce the number of points
        traj = traj[::40]  # Adjust the step size as needed

        return traj

        

def wait_until(start_time_str):
    """
    Wait until the specified real-world time.
    Format: "YYYY-MM-DD HH:MM:SS"
    """
    target_time = datetime.strptime(start_time_str, "%Y-%m-%d %H:%M:%S")
    rospy.loginfo(f"Waiting to start until {target_time}...")
    while datetime.now() < target_time and not rospy.is_shutdown():
        time.sleep(0.1)
    rospy.loginfo("Start time reached. Publishing trajectory...")


def publish_trajectory_markers(pub_marker_array, points, frame_id="map"):
    """
    Publishes a MarkerArray showing the trajectory points as blue spheres.
    
    Args:
        pub_marker_array: rospy.Publisher for MarkerArray
        points: list of [x, y, theta]
        frame_id: coordinate frame for visualization
    """
    marker_array = MarkerArray()

    for i, (x, y) in enumerate(points):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "trajectory"
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # Blue, fully opaque
        marker.lifetime = rospy.Duration(0)  # Permanent until manually removed
        marker_array.markers.append(marker)

    pub_marker_array.publish(marker_array)


def publish_trajectory(points, robot, interval=2.0):

    pub_name = f"/{robot}/move_base_simple/goal"
    frame_id = f"{robot}/map"
    pub = rospy.Publisher(pub_name, PoseStamped, queue_size=10)
    marker_pub = rospy.Publisher(f"/{robot}/trajectory_markers", MarkerArray, queue_size=10)

    rospy.init_node('trajectory_publisher', anonymous=True)
    rate = rospy.Rate(1.0 / interval)

    publish_trajectory_markers(marker_pub, points, frame_id=frame_id)

    for point in points:
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_id

        print(f"Publishing point: {point}")

        # Assuming [x, y, theta]
        x, y= point
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        rospy.loginfo(f"Publishing to {pub_name}: ({x:.2f}, {y:.2f}")
        pub.publish(pose)
        rate.sleep()


import argparse
import rospy

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Replay robot trajectory from a specified start time.")
    parser.add_argument("--start-time", required=True, help="Start time in format YYYY-MM-DD HH:MM:SS")
    parser.add_argument("--robot", default="scarab40", help="Robot name (default: scarab40)")
    parser.add_argument("--file", default="Real_World/RM1/robot_trajectories.pkl", help="Trajectory file path")

    args = parser.parse_args()

    try:
        traj = load_trajectory(args.file)
        print(f"Loaded trajectory with {len(traj)} points.")
        wait_until(args.start_time)
        publish_trajectory(traj, robot=args.robot, interval=2.0)
    except rospy.ROSInterruptException:
        pass
