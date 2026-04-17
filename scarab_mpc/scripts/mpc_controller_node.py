#!/usr/bin/env python3
"""
ROS node for the Scarab MPC controller.

Each node instance controls one robot: subscribe to /<agent>/pose and
/<agent>/move/goal, track externally published waypoints, and publish
/<agent>/cmd_vel directly. Multiple instances can run in parallel under
different robot namespaces. The node does not start or call HFN.
"""

from __future__ import annotations

import json
import math
import os
import time as wall_time
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from scarab_msgs.msg import MoveActionGoal
from std_msgs.msg import String

from scarab_mpc.config import MPCConfig
from scarab_mpc.mpc_controller import MPCController
from scarab_mpc.reference_generator import ReferenceTrajectory
from scarab_mpc.ros_adapter import make_twist, normalize_angle, pose_to_state, zero_twist


def _param(name: str, default: Any = None) -> Any:
    return rospy.get_param("~" + name, default)


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _parse_start_time(value: Any) -> Optional[rospy.Time]:
    if value is None:
        return None

    if isinstance(value, (int, float)):
        return rospy.Time.from_sec(float(value))

    text = str(value).strip()
    if not text:
        return None

    try:
        return rospy.Time.from_sec(float(text))
    except ValueError:
        pass

    for fmt in ("%Y-%m-%d %H:%M:%S", "%Y-%m-%dT%H:%M:%S"):
        try:
            parsed = datetime.strptime(text, fmt)
            return rospy.Time.from_sec(wall_time.mktime(parsed.timetuple()))
        except ValueError:
            continue

    raise ValueError(
        "~start_time must be empty, epoch seconds, "
        "or 'YYYY-MM-DD HH:MM:SS'"
    )


def _matrix_param(name: str, default: np.ndarray) -> np.ndarray:
    value = _param(name, default.tolist())
    arr = np.array(value, dtype=np.float64)
    if arr.ndim == 1:
        return np.diag(arr)
    if arr.ndim == 2 and arr.shape[0] == arr.shape[1]:
        return arr
    raise ValueError(f"~{name} must be a diagonal list or square matrix")


def _yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _heading_between(points: List[List[float]], index: int) -> float:
    if len(points) < 2:
        return 0.0
    current = points[index]
    for next_index in range(index + 1, len(points)):
        candidate = points[next_index]
        dx = candidate[0] - current[0]
        dy = candidate[1] - current[1]
        if math.hypot(dx, dy) > 1e-8:
            return math.atan2(dy, dx)
    for prev_index in range(index - 1, -1, -1):
        candidate = points[prev_index]
        dx = current[0] - candidate[0]
        dy = current[1] - candidate[1]
        if math.hypot(dx, dy) > 1e-8:
            return math.atan2(dy, dx)
    return 0.0


def _make_path_from_waypoints(
    waypoints: List[List[float]],
    frame_id: str,
    stamp: rospy.Time,
) -> Path:
    path = Path()
    path.header.stamp = stamp
    path.header.frame_id = frame_id
    for index, point in enumerate(waypoints):
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = _yaw_to_quaternion(_heading_between(waypoints, index))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        path.poses.append(pose)
    return path


def _make_path_from_states(
    states: np.ndarray,
    frame_id: str,
    stamp: rospy.Time,
) -> Path:
    path = Path()
    path.header.stamp = stamp
    path.header.frame_id = frame_id
    for state in states:
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = float(state[0])
        pose.pose.position.y = float(state[1])
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = _yaw_to_quaternion(float(state[2]))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        path.poses.append(pose)
    return path


class ScarabMPCNode:
    def __init__(self) -> None:
        self.agent = str(_param("agent", os.environ.get("AGENT", "scarab42")))
        self.robot_index = int(_param("robot_index", 0))
        self.reference_source = str(_param("reference_source", "move_goal")).lower()
        if self.reference_source not in ("move_goal", "external", "topic"):
            raise ValueError(
                f"Unsupported ~reference_source={self.reference_source}. "
                "This controller only accepts externally published "
                "scarab_msgs/MoveActionGoal waypoints."
            )

        self.dry_run = _as_bool(_param("dry_run", True))
        self.publish_cmd = _as_bool(_param("publish_cmd", False))
        self.stop_on_exit = _as_bool(_param("stop_on_exit", True))
        self.pose_timeout = float(_param("pose_timeout", 0.5))
        self.control_rate = float(_param("control_rate", 10.0))
        self.start_delay = float(_param("start_delay", 0.0))
        self.start_time = _parse_start_time(_param("start_time", ""))
        self.goal_tolerance = float(_param("goal_tolerance", 0.12))
        self.use_neighbor_safety = _as_bool(_param("use_neighbor_safety", False))
        self.prepend_current_pose = _as_bool(_param("prepend_current_pose", True))
        self.prepend_current_pose_min_distance = float(
            _param("prepend_current_pose_min_distance", 0.05)
        )

        self.pose_topic = str(_param("pose_topic", f"/{self.agent}/pose"))
        self.cmd_vel_topic = str(_param("cmd_vel_topic", f"/{self.agent}/cmd_vel"))
        self.goal_topic = str(_param("goal_topic", f"/{self.agent}/move/goal"))
        self.status_topic = str(_param("status_topic", f"/{self.agent}/mpc/status"))
        self.predicted_path_topic = str(
            _param("predicted_path_topic", f"/{self.agent}/mpc/predicted_path")
        )
        self.reference_path_topic = str(
            _param("reference_path_topic", f"/{self.agent}/mpc/reference_path")
        )
        self.actual_path_topic = str(
            _param("actual_path_topic", f"/{self.agent}/mpc/actual_path")
        )
        self.actual_path_min_distance = float(_param("actual_path_min_distance", 0.03))
        self.actual_path_max_poses = int(_param("actual_path_max_poses", 2000))
        self.tracking_error_log_period = float(_param("tracking_error_log_period", 1.0))
        self.frame_id = str(_param("frame_id", f"{self.agent}/map"))

        if self.use_neighbor_safety:
            rospy.logwarn("~use_neighbor_safety is true, but Phase 1 ignores neighbors.")
            self.use_neighbor_safety = False

        self.cfg = self._load_mpc_config()
        self.controller = MPCController(
            robot_id=self.robot_index,
            cfg=self.cfg,
            formation_keeper=None,
        )

        self.reference: Optional[ReferenceTrajectory] = None
        self.reference_waypoints: List[List[float]] = []
        self.reference_robot = self.agent
        self.active_goal_id = ""
        self.pending_goal: Optional[MoveActionGoal] = None
        self.current_state: Optional[np.ndarray] = None
        self.last_pose_time: Optional[rospy.Time] = None
        self.first_ready_time: Optional[rospy.Time] = None
        self.tracking_start_time: Optional[rospy.Time] = None
        self.finished = False
        self.actual_path = Path()
        self.actual_path.header.frame_id = self.frame_id

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1)
        self.predicted_path_pub = rospy.Publisher(
            self.predicted_path_topic, Path, queue_size=1
        )
        self.reference_path_pub = rospy.Publisher(
            self.reference_path_topic, Path, queue_size=1, latch=True
        )
        self.actual_path_pub = rospy.Publisher(
            self.actual_path_topic, Path, queue_size=1, latch=True
        )
        self.pose_sub = rospy.Subscriber(
            self.pose_topic, PoseStamped, self._on_pose, queue_size=1
        )
        self.goal_sub = rospy.Subscriber(
            self.goal_topic, MoveActionGoal, self._on_goal, queue_size=1
        )

        rospy.on_shutdown(self._on_shutdown)
        self._log_startup()

    def _load_mpc_config(self) -> MPCConfig:
        default = MPCConfig()
        return MPCConfig(
            dt=float(_param("dt", default.dt)),
            N=int(_param("N", default.N)),
            v_min=float(_param("v_min", default.v_min)),
            v_max=float(_param("v_max", default.v_max)),
            omega_max=float(_param("omega_max", default.omega_max)),
            Q=_matrix_param("Q", default.Q),
            R=_matrix_param("R", default.R),
            Q_terminal=_matrix_param("Q_terminal", default.Q_terminal),
            lambda_formation=0.0,
            d_safe_robot=float(_param("d_safe_robot", default.d_safe_robot)),
            d_safe_obstacle=float(_param("d_safe_obstacle", default.d_safe_obstacle)),
            w_slack=float(_param("w_slack", default.w_slack)),
            slack_min=float(_param("slack_min", default.slack_min)),
            solver_max_iter=int(_param("solver_max_iter", default.solver_max_iter)),
            solver_print_level=int(_param("solver_print_level", default.solver_print_level)),
            warm_start=_as_bool(_param("warm_start", default.warm_start)),
            cruise_speed=float(_param("cruise_speed", default.cruise_speed)),
            turn_speed_factor=float(
                _param("turn_speed_factor", default.turn_speed_factor)
            ),
            turn_angle_threshold=float(
                _param("turn_angle_threshold", default.turn_angle_threshold)
            ),
        )

    def _on_goal(self, msg: MoveActionGoal) -> None:
        if msg.goal.stop:
            self._clear_reference("stop_goal")
            return

        if not msg.goal.target_poses:
            rospy.logwarn("[%s] Ignoring empty goal on %s", self.agent, self.goal_topic)
            return

        if self.current_state is None:
            self.pending_goal = msg
            rospy.loginfo(
                "[%s] Queued goal with %d target pose(s); waiting for pose.",
                self.agent,
                len(msg.goal.target_poses),
            )
            return

        self._accept_goal(msg)

    def _accept_goal(self, msg: MoveActionGoal) -> None:
        target_waypoints = self._waypoints_from_goal(msg)
        if not target_waypoints:
            rospy.logwarn("[%s] Ignoring goal with no usable target poses.", self.agent)
            return

        waypoints = self._waypoints_with_current_start(target_waypoints)
        self.reference = ReferenceTrajectory(waypoints, self.cfg)
        self.reference_waypoints = waypoints
        self.reference_robot = self.agent
        self.active_goal_id = msg.goal_id.id or f"goal_{msg.goal_id.stamp.to_nsec()}"
        self.pending_goal = None
        self.first_ready_time = None
        self.tracking_start_time = None
        self.finished = False
        self.controller.reset()
        self._publish_reference_path()

        rospy.loginfo(
            "[%s] Accepted external MPC goal id=%s targets=%d waypoints=%d "
            "total_time=%.2fs source=%s",
            self.agent,
            self.active_goal_id,
            len(target_waypoints),
            len(self.reference_waypoints),
            self.reference.total_time,
            self.goal_topic,
        )

    def _clear_reference(self, reason: str) -> None:
        self.reference = None
        self.reference_waypoints = []
        self.active_goal_id = ""
        self.pending_goal = None
        self.first_ready_time = None
        self.tracking_start_time = None
        self.finished = False
        self.controller.reset()
        self._publish_reference_path()
        self._publish_stop(reason)
        rospy.loginfo("[%s] Cleared MPC reference: %s", self.agent, reason)

    def _waypoints_from_goal(self, msg: MoveActionGoal) -> List[List[float]]:
        waypoints: List[List[float]] = []
        for pose_stamped in msg.goal.target_poses:
            frame_id = pose_stamped.header.frame_id or msg.header.frame_id
            if frame_id and frame_id != self.frame_id:
                rospy.logwarn_throttle(
                    5.0,
                    "[%s] Goal frame '%s' differs from MPC frame '%s'. "
                    "No TF transform is applied; coordinates are used as-is.",
                    self.agent,
                    frame_id,
                    self.frame_id,
                )
            waypoints.append([
                float(pose_stamped.pose.position.x),
                float(pose_stamped.pose.position.y),
            ])
        return waypoints

    def _waypoints_with_current_start(
        self,
        target_waypoints: List[List[float]],
    ) -> List[List[float]]:
        if self.current_state is None:
            return target_waypoints

        current = [float(self.current_state[0]), float(self.current_state[1])]
        first = target_waypoints[0]
        first_distance = math.hypot(current[0] - first[0], current[1] - first[1])

        if len(target_waypoints) == 1:
            return [current, first]

        if (
            self.prepend_current_pose
            and first_distance > self.prepend_current_pose_min_distance
        ):
            return [current] + target_waypoints

        return target_waypoints

    def _on_pose(self, msg: PoseStamped) -> None:
        self.current_state = pose_to_state(msg)
        self.last_pose_time = rospy.Time.now()
        self._record_actual_path(msg)

    def _record_actual_path(self, msg: PoseStamped) -> None:
        frame_id = msg.header.frame_id or self.frame_id
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        if not self.actual_path.poses:
            self.actual_path.header.frame_id = frame_id
        elif self.actual_path_min_distance > 0.0:
            last = self.actual_path.poses[-1].pose.position
            current = msg.pose.position
            if math.hypot(current.x - last.x, current.y - last.y) < self.actual_path_min_distance:
                return

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.actual_path.header.frame_id
        pose.pose.position.x = msg.pose.position.x
        pose.pose.position.y = msg.pose.position.y
        pose.pose.position.z = msg.pose.position.z
        pose.pose.orientation = msg.pose.orientation

        self.actual_path.poses.append(pose)
        if self.actual_path_max_poses > 0 and len(self.actual_path.poses) > self.actual_path_max_poses:
            self.actual_path.poses = self.actual_path.poses[-self.actual_path_max_poses :]
        self.actual_path.header.stamp = stamp
        self.actual_path_pub.publish(self.actual_path)

    def spin(self) -> None:
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            self._step()
            rate.sleep()

    def _step(self) -> None:
        now = rospy.Time.now()
        if self.current_state is None or self.last_pose_time is None:
            self._publish_status(
                "waiting_for_pose",
                now,
                pending_goal=self.pending_goal is not None,
            )
            rospy.logwarn_throttle(2.0, "[%s] Waiting for pose on %s", self.agent, self.pose_topic)
            return

        if self.pending_goal is not None:
            self._accept_goal(self.pending_goal)

        pose_age = (now - self.last_pose_time).to_sec()
        if self.pose_timeout > 0.0 and pose_age > self.pose_timeout:
            self.controller.reset()
            self._publish_stop("stale_pose")
            self._publish_status("stale_pose", now, pose_age=pose_age)
            rospy.logwarn_throttle(
                1.0,
                "[%s] Pose timeout: age=%.3fs > %.3fs. Publishing stop.",
                self.agent,
                pose_age,
                self.pose_timeout,
            )
            return

        if self.reference is None:
            self._publish_stop("waiting_for_goal")
            self._publish_status("waiting_for_goal", now, pose_age=pose_age)
            rospy.loginfo_throttle(
                2.0,
                "[%s] Waiting for external waypoint goal on %s",
                self.agent,
                self.goal_topic,
            )
            return

        if self.tracking_start_time is None:
            if self.first_ready_time is None:
                self.first_ready_time = now
            if self.start_time is not None:
                delay_remaining = (self.start_time - now).to_sec()
                if delay_remaining > 0.0:
                    self._publish_stop("start_time")
                    self._publish_status(
                        "start_time",
                        now,
                        delay_remaining=delay_remaining,
                        start_time=self.start_time.to_sec(),
                    )
                    return
                self.tracking_start_time = self.start_time
                self.controller.reset()
                rospy.loginfo(
                    "[%s] MPC tracking started at synchronized start_time %.3f.",
                    self.agent,
                    self.start_time.to_sec(),
                )
                return
            ready_age = (now - self.first_ready_time).to_sec()
            if ready_age < self.start_delay:
                self._publish_stop("start_delay")
                self._publish_status("start_delay", now, delay_remaining=self.start_delay - ready_age)
                return
            self.tracking_start_time = now
            self.controller.reset()
            rospy.loginfo("[%s] MPC tracking started.", self.agent)

        t_ref = (now - self.tracking_start_time).to_sec()
        if self.reference.is_finished(t_ref, self.current_state, self.goal_tolerance):
            if not self.finished:
                rospy.loginfo("[%s] MPC trajectory finished.", self.agent)
                self.finished = True
            self._publish_stop("finished")
            self._publish_status("finished", now, t_ref=t_ref)
            return

        x_ref, u_ref = self.reference.query_horizon(t_ref, self.cfg.N, self.cfg.dt)
        tracking_error = self._tracking_error(x_ref[0])
        v_cmd, omega_cmd = self.controller.solve(
            self.current_state,
            x_ref,
            u_ref,
            neighbor_states=None,
            obstacles=None,
        )
        v_cmd, omega_cmd = self._limit_command(v_cmd, omega_cmd)

        if self.dry_run:
            rospy.loginfo_throttle(
                1.0,
                "[%s] DRY RUN t=%.2f progress=%.1f%% cmd=(%.3f, %.3f)",
                self.agent,
                t_ref,
                100.0 * self.reference.get_progress(t_ref),
                v_cmd,
                omega_cmd,
            )
        elif not self.publish_cmd:
            rospy.loginfo_throttle(
                2.0,
                "[%s] publish_cmd=false; computed cmd=(%.3f, %.3f)",
                self.agent,
                v_cmd,
                omega_cmd,
            )
        else:
            self.cmd_pub.publish(make_twist(v_cmd, omega_cmd))

        if self.tracking_error_log_period > 0.0:
            rospy.loginfo_throttle(
                self.tracking_error_log_period,
                (
                    "[%s] tracking_error xy=%.3f x=%.3f y=%.3f "
                    "theta=%.3f ref=(%.3f, %.3f, %.3f) "
                    "pose=(%.3f, %.3f, %.3f) cmd=(%.3f, %.3f)"
                ),
                self.agent,
                tracking_error["tracking_error_xy"],
                tracking_error["tracking_error_x"],
                tracking_error["tracking_error_y"],
                tracking_error["tracking_error_theta"],
                tracking_error["reference_x"],
                tracking_error["reference_y"],
                tracking_error["reference_theta"],
                tracking_error["pose_x"],
                tracking_error["pose_y"],
                tracking_error["pose_theta"],
                v_cmd,
                omega_cmd,
            )

        self._publish_predicted_path(now)
        self._publish_status(
            "tracking",
            now,
            t_ref=t_ref,
            pose_age=pose_age,
            v=v_cmd,
            omega=omega_cmd,
            **tracking_error,
        )

    def _limit_command(self, v: float, omega: float) -> Tuple[float, float]:
        v_limited = min(max(float(v), self.cfg.v_min), self.cfg.v_max)
        omega_limited = min(max(float(omega), -self.cfg.omega_max), self.cfg.omega_max)
        return v_limited, omega_limited

    def _tracking_error(self, ref_state: np.ndarray) -> Dict[str, float]:
        if self.current_state is None:
            return {}
        err_x = float(self.current_state[0] - ref_state[0])
        err_y = float(self.current_state[1] - ref_state[1])
        err_theta = float(normalize_angle(self.current_state[2] - ref_state[2]))
        return {
            "pose_x": float(self.current_state[0]),
            "pose_y": float(self.current_state[1]),
            "pose_theta": float(self.current_state[2]),
            "reference_x": float(ref_state[0]),
            "reference_y": float(ref_state[1]),
            "reference_theta": float(ref_state[2]),
            "tracking_error_x": err_x,
            "tracking_error_y": err_y,
            "tracking_error_theta": err_theta,
            "tracking_error_xy": float(math.hypot(err_x, err_y)),
        }

    def _publish_stop(self, reason: str) -> None:
        if not self.dry_run and self.publish_cmd:
            self.cmd_pub.publish(zero_twist())
        rospy.loginfo_throttle(2.0, "[%s] stop command: %s", self.agent, reason)

    def _publish_status(self, mode: str, stamp: rospy.Time, **extra: Any) -> None:
        payload: Dict[str, Any] = {
            "agent": self.agent,
            "mode": mode,
            "reference_robot": self.reference_robot,
            "reference_source": self.reference_source,
            "goal_topic": self.goal_topic,
            "active_goal_id": self.active_goal_id,
            "has_reference": self.reference is not None,
            "waypoint_count": len(self.reference_waypoints),
            "dry_run": self.dry_run,
            "publish_cmd": self.publish_cmd,
            "stamp": stamp.to_sec(),
            "progress": None,
            "solve": self.controller.stats,
        }
        if self.reference is not None and self.tracking_start_time is not None:
            t_ref = (stamp - self.tracking_start_time).to_sec()
            payload["progress"] = float(self.reference.get_progress(t_ref))
        for key, value in extra.items():
            if isinstance(value, np.generic):
                value = value.item()
            payload[key] = value
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _publish_reference_path(self) -> None:
        stamp = rospy.Time.now()
        if self.reference_waypoints:
            path = _make_path_from_waypoints(self.reference_waypoints, self.frame_id, stamp)
        else:
            path = Path()
            path.header.stamp = stamp
            path.header.frame_id = self.frame_id
        self.reference_path_pub.publish(path)

    def _publish_predicted_path(self, stamp: rospy.Time) -> None:
        states = self.controller.last_predicted_states
        if states is None:
            return
        self.predicted_path_pub.publish(_make_path_from_states(states, self.frame_id, stamp))

    def _on_shutdown(self) -> None:
        if self.stop_on_exit:
            self._publish_stop("shutdown")

    def _log_startup(self) -> None:
        rospy.loginfo(
            "[%s] Scarab MPC node ready. pose=%s cmd_vel=%s dry_run=%s "
            "publish_cmd=%s goal=%s reference_source=%s",
            self.agent,
            self.pose_topic,
            self.cmd_vel_topic,
            self.dry_run,
            self.publish_cmd,
            self.goal_topic,
            self.reference_source,
        )
        if self.start_time is not None:
            rospy.loginfo(
                "[%s] MPC synchronized start_time=%.3f.",
                self.agent,
                self.start_time.to_sec(),
            )
        if self.dry_run or not self.publish_cmd:
            rospy.logwarn(
                "[%s] Motor publishing disabled by dry_run=%s publish_cmd=%s.",
                self.agent,
                self.dry_run,
                self.publish_cmd,
            )


def main() -> None:
    rospy.init_node("mpc_controller")
    node = ScarabMPCNode()
    node.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
