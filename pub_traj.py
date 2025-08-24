#!/usr/bin/env python

import rospy
import pickle
import time
from datetime import datetime
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf_trans
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from scarab_msgs.msg import MoveActionGoal


def load_trajectory(filepath, robot):
    with open(filepath, 'rb') as f:

        rospy.loginfo(f"Loading trajectory from {filepath}...")

        file = pickle.load(f)
        #print(f"Loaded trajectory file: {file}")

        # it is a list of points, and convert into a single list

        num_robot = len(file)

        print(f"Number of robots in file: {num_robot}")


        #print file type
        print(f"File type: {type(file)}")
        #it is a dict
        print(f"File keys: {file.keys()}")

        keys = list(file.keys())

        if robot == "scarab41":
            traj = file[keys[0]]
            #traj[-1][0] = -0.2  # Set last x to -0.2
        elif robot == "scarab42":
            traj = file[keys[1]]
        elif robot == "scarab46":
            traj = file[keys[2]]
        elif robot == "scarab45":
            traj = file[keys[3]]



        #down sample the trajectory to reduce the number of points, and keep the first and last points
        # start = traj[0]
        # end = traj[-1]
        traj = traj[::10]


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



def publish_trajectory(points, robot, interval=2.0, simulation=True):
    if simulation:
        pub_name = f"/{robot}/move_base_simple/goal"
        pub = rospy.Publisher(pub_name, PoseStamped, queue_size=10)
    else:
        pub_name = f"/{robot}/move_base/goal"
        pub = rospy.Publisher(pub_name, MoveActionGoal, queue_size=10)

    frame_id = f"{robot}/map"
    marker_pub = rospy.Publisher(f"/{robot}/trajectory_markers", MarkerArray, queue_size=10)

    rospy.init_node('trajectory_publisher', anonymous=True)
    rate = rospy.Rate(1.0 / interval)

    publish_trajectory_markers(marker_pub, points, frame_id=frame_id)

    for i, point in enumerate(points):
        x, y = point

        if simulation:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            qx, qy, qz, qw = tf_trans.quaternion_from_euler(0, 0, 1.5708)  # 90 degree rotation
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            


            rospy.loginfo(f"[SIM] Publishing to {pub_name}: ({x:.2f}, {y:.2f})")
            pub.publish(pose)
        else:
            goal = MoveActionGoal()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = frame_id
            goal.goal_id.stamp = goal.header.stamp
            goal.goal_id.id = f"goal_{i}"
            goal.goal.target_pose.header = goal.header
            goal.goal.target_pose.pose.position.x = x
            goal.goal.target_pose.pose.position.y = y
            goal.goal.target_pose.pose.position.z = 0.0
            # set 90 degree rotation
            qx, qy, qz, qw = tf_trans.quaternion_from_euler(0, 0, -1.5708)
            goal.goal.target_pose.pose.orientation.x = qx
            goal.goal.target_pose.pose.orientation.y = qy
            goal.goal.target_pose.pose.orientation.z = qz
            goal.goal.target_pose.pose.orientation.w = qw

            rospy.loginfo(f"[REAL] Publishing to {pub_name}: ({x:.2f}, {y:.2f})")
            pub.publish(goal)

        rate.sleep()


import argparse
import rospy

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Replay robot trajectory from a specified start time.")
    parser.add_argument("--start-time", required=True, help="Start time in format YYYY-MM-DD HH:MM:SS")
    parser.add_argument("--robot", default="scarab40", help="Robot name (default: scarab40)")
    parser.add_argument("--file", default="Real_World/RM2/robot_trajectories.pkl", help="Trajectory file path")

    args = parser.parse_args()

    try:
        traj = load_trajectory(args.file, args.robot)
        print(f"Loaded trajectory with {len(traj)} points.")
        wait_until(args.start_time)
        publish_trajectory(traj, robot=args.robot, interval=1.0)
    except rospy.ROSInterruptException:
        pass
