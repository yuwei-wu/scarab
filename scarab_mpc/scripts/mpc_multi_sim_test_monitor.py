#!/usr/bin/env python3
"""Monitor multiple MPC simulations and exit when all robots pass or any fail."""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String


def _as_robot_list(value: Any) -> List[str]:
    if isinstance(value, str):
        parts = value.replace(",", " ").split()
    else:
        parts = []
        for item in value:
            parts.extend(str(item).replace(",", " ").split())
    robots = [part.strip() for part in parts if part.strip()]
    if not robots:
        raise ValueError("~robots must contain at least one robot name")
    duplicates = sorted({robot for robot in robots if robots.count(robot) > 1})
    if duplicates:
        raise ValueError(f"Duplicate robot names are not allowed: {duplicates}")
    return robots


def _distance(a: PoseStamped, b: PoseStamped) -> float:
    return math.hypot(
        a.pose.position.x - b.pose.position.x,
        a.pose.position.y - b.pose.position.y,
    )


class MPCMultiSimTestMonitor:
    def __init__(self) -> None:
        self.robots = _as_robot_list(rospy.get_param("~robots", "scarab42 scarab40 scarab46"))
        self.pose_topic_suffix = str(rospy.get_param("~pose_topic_suffix", "pose")).strip("/")
        self.status_topic_suffix = str(
            rospy.get_param("~status_topic_suffix", "mpc/status")
        ).strip("/")
        self.reference_path_topic_suffix = str(
            rospy.get_param("~reference_path_topic_suffix", "mpc/reference_path")
        ).strip("/")
        self.timeout = float(rospy.get_param("~timeout", 120.0))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.35))
        self.require_finished_status = bool(
            rospy.get_param("~require_finished_status", True)
        )

        self.latest_pose: Dict[str, Optional[PoseStamped]] = {
            robot: None for robot in self.robots
        }
        self.final_pose: Dict[str, Optional[PoseStamped]] = {
            robot: None for robot in self.robots
        }
        self.latest_status: Dict[str, Dict[str, Any]] = {
            robot: {} for robot in self.robots
        }
        self.start_time = rospy.Time.now()

        for robot in self.robots:
            rospy.Subscriber(
                f"/{robot}/{self.pose_topic_suffix}",
                PoseStamped,
                self._on_pose,
                callback_args=robot,
                queue_size=1,
            )
            rospy.Subscriber(
                f"/{robot}/{self.status_topic_suffix}",
                String,
                self._on_status,
                callback_args=robot,
                queue_size=10,
            )
            rospy.Subscriber(
                f"/{robot}/{self.reference_path_topic_suffix}",
                Path,
                self._on_reference_path,
                callback_args=robot,
                queue_size=1,
            )

    def _on_pose(self, msg: PoseStamped, robot: str) -> None:
        self.latest_pose[robot] = msg

    def _on_status(self, msg: String, robot: str) -> None:
        try:
            self.latest_status[robot] = json.loads(msg.data)
        except ValueError:
            rospy.logwarn("[%s] Could not parse MPC status JSON: %s", robot, msg.data)

    def _on_reference_path(self, msg: Path, robot: str) -> None:
        if msg.poses:
            self.final_pose[robot] = msg.poses[-1]
            rospy.loginfo(
                "[%s multi monitor] final reference=(%.3f, %.3f)",
                robot,
                self.final_pose[robot].pose.position.x,
                self.final_pose[robot].pose.position.y,
            )

    def _final_distance(self, robot: str) -> Optional[float]:
        pose = self.latest_pose[robot]
        final_pose = self.final_pose[robot]
        if pose is None or final_pose is None:
            return None
        return _distance(pose, final_pose)

    def _robot_passed(self, robot: str) -> bool:
        final_dist = self._final_distance(robot)
        if final_dist is None or final_dist > self.goal_tolerance:
            return False
        mode = self.latest_status[robot].get("mode", "no_status")
        return mode == "finished" or not self.require_finished_status

    def _summary(self) -> str:
        parts = []
        for robot in self.robots:
            status = self.latest_status[robot]
            mode = status.get("mode", "no_status")
            progress = status.get("progress")
            tracking_error_xy = status.get("tracking_error_xy")
            final_dist = self._final_distance(robot)
            parts.append(
                "{}:mode={} progress={} err={} final={}".format(
                    robot,
                    mode,
                    "None" if progress is None else f"{100.0 * progress:.1f}%",
                    "None" if tracking_error_xy is None else f"{tracking_error_xy:.3f}",
                    "None" if final_dist is None else f"{final_dist:.3f}",
                )
            )
        return " | ".join(parts)

    def run(self) -> int:
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - self.start_time).to_sec()
            rospy.loginfo_throttle(
                2.0,
                "[multi MPC monitor] %s",
                self._summary(),
            )

            if all(self._robot_passed(robot) for robot in self.robots):
                rospy.loginfo(
                    "[multi MPC monitor] PASS robots=%s elapsed=%.1fs %s",
                    ",".join(self.robots),
                    elapsed,
                    self._summary(),
                )
                return 0

            if elapsed > self.timeout:
                rospy.logerr(
                    "[multi MPC monitor] FAIL timeout %.1fs %s",
                    self.timeout,
                    self._summary(),
                )
                return 1

            rate.sleep()
        return 1


def main() -> int:
    rospy.init_node("mpc_multi_sim_test_monitor")
    return MPCMultiSimTestMonitor().run()


if __name__ == "__main__":
    sys.exit(main())
