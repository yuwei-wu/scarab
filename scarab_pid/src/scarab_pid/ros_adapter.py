"""
ROS adapter helpers for the Scarab PID controller.

The PID core is ROS-independent. This module holds the small message
conversion layer:

    Pose/PoseStamped -> [x, y, yaw]
    (v, omega)       -> geometry_msgs/Twist

Standalone copy owned by scarab_pid; no import from scarab_mpc.
"""

from __future__ import annotations

import math
from typing import Any, Optional

import numpy as np


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, limit: Optional[float]) -> float:
    """Clamp value symmetrically if a limit is provided."""
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
    """Convert geometry_msgs/Pose or PoseStamped into [x, y, yaw]."""
    pose = msg.pose if hasattr(msg, "pose") else msg
    return np.array(
        [
            float(pose.position.x),
            float(pose.position.y),
            quaternion_to_yaw(pose.orientation),
        ],
        dtype=np.float64,
    )


def yaw_to_quaternion(yaw: float):
    """Return (x, y, z, w) for a planar yaw."""
    half = 0.5 * float(yaw)
    return 0.0, 0.0, math.sin(half), math.cos(half)


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
