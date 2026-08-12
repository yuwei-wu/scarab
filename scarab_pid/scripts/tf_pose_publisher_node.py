#!/usr/bin/env python3
"""Publish a PoseStamped by looking up a TF transform."""

from __future__ import annotations

import sys

import rospy
import tf
from geometry_msgs.msg import PoseStamped


def main() -> int:
    rospy.init_node("tf_pose_publisher")

    base_frame = rospy.get_param("~base_frame_id", "base")
    map_frame = rospy.get_param("~map_frame_id", "map")
    rate_hz = float(rospy.get_param("~rate", 10.0))

    pose_pub = rospy.Publisher("pose", PoseStamped, queue_size=5)
    tf_listener = tf.TransformListener()

    rate = rospy.Rate(rate_hz)
    last_pub = rospy.Time.now()
    while not rospy.is_shutdown():
        try:
            trans, rot = tf_listener.lookupTransform(map_frame, base_frame, rospy.Time(0))
        except tf.Exception as err:
            age = (rospy.Time.now() - last_pub).to_sec()
            rospy.logwarn_throttle(
                2.0,
                "tf_pose_publisher: waiting for %s -> %s transform; "
                "last pose %.2fs ago: %s",
                map_frame,
                base_frame,
                age,
                err,
            )
            rate.sleep()
            continue

        pose = PoseStamped()
        pose.header.frame_id = map_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]
        pose_pub.publish(pose)
        last_pub = pose.header.stamp

        rate.sleep()

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(0)
