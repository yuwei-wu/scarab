"""
Controller Parameter Configuration
====================================
Centralized management of MPC parameters, robot physical parameters, weight matrices, etc.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class MPCConfig:
    """MPC high-level parameters tuned conservatively for real Scarabs.

    The MPC outputs geometry_msgs/Twist-style (v, omega). Wheel geometry,
    encoder conversion, and low-level motor PID remain owned by roboclaw_node
    and roboclaw/cfg/roboclaw.yaml.
    """

    # ---- Time parameters ----
    dt: float = 0.1
    """MPC control period (s), i.e., 10 Hz"""

    N: int = 8
    """Prediction horizon length"""

    # ---- Velocity limits ----
    v_min: float = -0.05
    """Minimum linear velocity (m/s), allows slight reverse"""

    v_max: float = 0.25
    """Maximum linear velocity (m/s)"""

    omega_max: float = 0.60
    """Maximum angular velocity (rad/s)"""

    # ---- Cost weights ----
    Q: np.ndarray = field(default_factory=lambda: np.diag([80.0, 80.0, 4.0]))
    """State tracking weight diag(q_x, q_y, q_theta)"""

    R: np.ndarray = field(default_factory=lambda: np.diag([2.0, 0.8]))
    """Control input weight diag(r_v, r_omega)"""

    Q_terminal: np.ndarray = field(default_factory=lambda: np.diag([120.0, 120.0, 6.0]))
    """Terminal state weight (encourages accuracy at the end of horizon)"""

    # ---- Formation keeping ----
    lambda_formation: float = 0.0
    """Formation consensus weight, 0 means relying solely on waypoint tracking for implicit keeping"""

    # ---- Safety constraints ----
    d_safe_robot: float = 0.45
    """Minimum safety distance between robots (m)"""

    d_safe_obstacle: float = 0.15
    """Minimum safety distance between robot and obstacle (m)"""

    w_slack: float = 200.0
    """Safety constraint slack penalty weight"""

    slack_min: float = -0.35
    """Slack variable lower bound (m), limits maximum violation"""

    # ---- Solver settings ----
    solver_max_iter: int = 80
    """IPOPT maximum iteration count"""

    solver_print_level: int = 0
    """IPOPT print level (0 = silent)"""

    warm_start: bool = True
    """Whether to use warm-start to accelerate solving"""

    # ---- Reference trajectory ----
    cruise_speed: float = 0.20
    """Cruise speed (m/s), used for waypoint time allocation"""

    turn_speed_factor: float = 0.5
    """Speed reduction factor during turns"""

    turn_angle_threshold: float = 0.5
    """Angle threshold for detecting turns (rad), approximately 28.6 degrees"""
