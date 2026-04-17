#!/usr/bin/env python3
"""Monitor a local MPC kinematic simulation and exit on pass/fail."""

from __future__ import annotations

import json
import math
import sys
from typing import Optional

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String


def _distance(a: PoseStamped, b: PoseStamped) -> float:
    return math.hypot(
        a.pose.position.x - b.pose.position.x,
        a.pose.position.y - b.pose.position.y,
    )


class MPCSimTestMonitor:
    def __init__(self) -> None:
        self.agent = rospy.get_param("~agent", "scarab42")
        self.pose_topic = rospy.get_param("~pose_topic", f"/{self.agent}/pose")
        self.status_topic = rospy.get_param("~status_topic", f"/{self.agent}/mpc/status")
        self.reference_path_topic = rospy.get_param(
            "~reference_path_topic", f"/{self.agent}/mpc/reference_path"
        )
        self.timeout = float(rospy.get_param("~timeout", 75.0))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.35))
        self.require_finished_status = bool(
            rospy.get_param("~require_finished_status", True)
        )

        self.latest_pose: Optional[PoseStamped] = None
        self.final_pose: Optional[PoseStamped] = None
        self.latest_status = {}
        self.start_time = rospy.Time.now()

        rospy.Subscriber(self.pose_topic, PoseStamped, self._on_pose, queue_size=1)
        rospy.Subscriber(self.status_topic, String, self._on_status, queue_size=10)
        rospy.Subscriber(
            self.reference_path_topic,
            Path,
            self._on_reference_path,
            queue_size=1,
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def _on_status(self, msg: String) -> None:
        try:
            self.latest_status = json.loads(msg.data)
        except ValueError:
            rospy.logwarn("Could not parse MPC status JSON: %s", msg.data)

    def _on_reference_path(self, msg: Path) -> None:
        if msg.poses:
            self.final_pose = msg.poses[-1]
            rospy.loginfo(
                "[%s sim monitor] final reference=(%.3f, %.3f)",
                self.agent,
                self.final_pose.pose.position.x,
                self.final_pose.pose.position.y,
            )

    def _final_distance(self) -> Optional[float]:
        if self.latest_pose is None or self.final_pose is None:
            return None
        return _distance(self.latest_pose, self.final_pose)

    def run(self) -> int:
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - self.start_time).to_sec()
            mode = self.latest_status.get("mode", "no_status")
            progress = self.latest_status.get("progress")
            tracking_error_xy = self.latest_status.get("tracking_error_xy")
            final_dist = self._final_distance()

            if final_dist is not None:
                rospy.loginfo_throttle(
                    2.0,
                    "[%s sim monitor] mode=%s progress=%s tracking_error_xy=%s final_dist=%.3f",
                    self.agent,
                    mode,
                    "None" if progress is None else f"{100.0 * progress:.1f}%",
                    "None" if tracking_error_xy is None else f"{tracking_error_xy:.3f}",
                    final_dist,
                )

            reached = final_dist is not None and final_dist <= self.goal_tolerance
            finished = mode == "finished"
            if reached and (finished or not self.require_finished_status):
                rospy.loginfo(
                    "[%s sim monitor] PASS final_dist=%.3f mode=%s elapsed=%.1fs",
                    self.agent,
                    final_dist,
                    mode,
                    elapsed,
                )
                return 0

            if elapsed > self.timeout:
                rospy.logerr(
                    "[%s sim monitor] FAIL timeout %.1fs mode=%s final_dist=%s",
                    self.agent,
                    self.timeout,
                    mode,
                    "None" if final_dist is None else f"{final_dist:.3f}",
                )
                return 1

            rate.sleep()
        return 1


def main() -> int:
    rospy.init_node("mpc_sim_test_monitor")
    return MPCSimTestMonitor().run()


if __name__ == "__main__":
    sys.exit(main())
