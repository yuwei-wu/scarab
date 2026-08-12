"""
Scarab PID Controller Core
==========================
Trajectory-tracking PID for Scarab differential-drive robots.

Fully independent of scarab_mpc: separate config, separate reference
generator, separate ROS node. The two packages share only the ROS interface
(/<agent>/pose in, /<agent>/move/goal in, /<agent>/cmd_vel out), so either
controller can drive the same robot with the same waypoint publisher.

Modules:
- config: PID gains, limits, and reference-trajectory parameters
- reference_generator: Waypoints to time-parameterized reference trajectory
- pid_controller: Feedforward + PID trajectory tracker (Kanayama structure)
- ros_adapter: ROS Pose/Twist conversion helpers
"""

from .config import PIDConfig
from .reference_generator import ReferenceTrajectory
from .pid_controller import PIDTrajectoryController

__all__ = [
    "PIDConfig",
    "ReferenceTrajectory",
    "PIDTrajectoryController",
]
