"""
ROS adapter helpers for the Scarab MPC controller.

The MPC core is intentionally ROS-independent. This module contains the small
message conversion layer needed by a future ROS node:

    Pose/PoseStamped -> [x, y, yaw]
    (v, omega)       -> geometry_msgs/Twist
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

import numpy as np


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, limit: Optional[float]) -> float:
    """Clamp value symmetrically if limit is provided."""
    if limit is None:
        return float(value)
    limit = abs(float(limit))
    return float(max(-limit, min(limit, value)))


def quaternion_to_yaw(q: Any) -> float:
    """Extract planar yaw from a ROS geometry_msgs/Quaternion-like object."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def pose_to_state(msg: Any) -> np.ndarray:
    """
    Convert geometry_msgs/Pose or PoseStamped into [x, y, yaw].

    The existing Scarab pose topics publish PoseStamped, but accepting Pose here
    makes the helper usable in tests and future nodes as well.
    """
    pose = msg.pose if hasattr(msg, "pose") else msg
    return np.array(
        [
            float(pose.position.x),
            float(pose.position.y),
            quaternion_to_yaw(pose.orientation),
        ],
        dtype=np.float64,
    )


def make_twist(
    v: float,
    omega: float,
    v_limit: Optional[float] = None,
    omega_limit: Optional[float] = None,
) -> Any:
    """Convert unicycle control into geometry_msgs/Twist."""
    from geometry_msgs.msg import Twist

    twist = Twist()
    twist.linear.x = clamp(v, v_limit)
    twist.angular.z = clamp(omega, omega_limit)
    return twist


def zero_twist() -> Any:
    """Return a zero geometry_msgs/Twist."""
    return make_twist(0.0, 0.0)


def neighbor_states(
    states_by_robot: Dict[str, Optional[np.ndarray]],
    self_robot: str,
) -> Dict[str, np.ndarray]:
    """Return all available neighbor states except the current robot."""
    return {
        name: state
        for name, state in states_by_robot.items()
        if name != self_robot and state is not None
    }
