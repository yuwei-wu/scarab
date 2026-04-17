"""
Reference Trajectory Generator
================================
Converts discrete waypoints from the LLM planner into a time-parameterized continuous
reference trajectory, allowing the MPC controller to query reference states and inputs
within the prediction horizon.

Uses piecewise linear interpolation: strictly follows a straight line between adjacent waypoints.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from .config import MPCConfig


def _normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


class ReferenceTrajectory:
    """
    Build a time-parameterized reference trajectory from discrete waypoints.

    Time allocation strategy:
    - Compute segment time based on inter-waypoint distance and cruise speed
    - Automatically reduce speed at turns (heading change exceeding threshold)
    - Accumulate to obtain arrival time t_i for each waypoint

    Query interface:
    - query(t) -> (x_ref, u_ref): Single-time reference
    - query_horizon(t, N, dt) -> (X_ref, U_ref): MPC prediction horizon
    """

    def __init__(
        self,
        waypoints: List[List[float]],
        cfg: Optional[MPCConfig] = None,
        segment_times: Optional[np.ndarray] = None,
    ):
        """
        Args:
            waypoints: [[x0,y0], [x1,y1], ...] ordered 2D waypoints
            cfg: MPC configuration (contains cruise speed, turn parameters, etc.)
            segment_times: Externally provided segment time array (for multi-robot synchronization),
                           length must be len(waypoints)-1. If None, computed automatically.
        """
        self.cfg = cfg or MPCConfig()
        self.waypoints = np.array(waypoints, dtype=np.float64)
        self.n_wps = len(self.waypoints)

        if self.n_wps < 2:
            raise ValueError("At least 2 waypoints are required to build a reference trajectory")

        self._headings = self._compute_headings()
        self._seg_lengths = self._compute_segment_lengths()

        if segment_times is not None:
            if len(segment_times) != self.n_wps - 1:
                raise ValueError(
                    f"segment_times length ({len(segment_times)}) "
                    f"!= waypoints-1 ({self.n_wps - 1})"
                )
            self._seg_times = np.array(segment_times, dtype=np.float64)
        else:
            self._seg_times = self._compute_segment_times()

        self._cumulative_times = np.concatenate(
            [[0.0], np.cumsum(self._seg_times)]
        )
        self.total_time = self._cumulative_times[-1]

    def _compute_headings(self) -> np.ndarray:
        """Compute the heading angle at each waypoint."""
        headings = np.zeros(self.n_wps)
        for i in range(self.n_wps - 1):
            diff = self.waypoints[i + 1] - self.waypoints[i]
            headings[i] = math.atan2(diff[1], diff[0])
        headings[-1] = headings[-2]
        return headings

    def _compute_segment_lengths(self) -> np.ndarray:
        """Compute the distance of each segment."""
        diffs = np.diff(self.waypoints, axis=0)
        return np.linalg.norm(diffs, axis=1)

    def _compute_segment_times(self) -> np.ndarray:
        """Allocate time for each segment based on distance and speed."""
        cfg = self.cfg
        times = np.zeros(self.n_wps - 1)

        for i in range(self.n_wps - 1):
            dist = self._seg_lengths[i]
            if dist < 1e-8:
                times[i] = cfg.dt
                continue

            speed = cfg.cruise_speed

            if i < self.n_wps - 2:
                angle_diff = abs(_normalize_angle(
                    self._headings[i + 1] - self._headings[i]
                ))
                if angle_diff > cfg.turn_angle_threshold:
                    speed *= cfg.turn_speed_factor

            speed = max(speed, 0.01)
            times[i] = dist / speed

        return times

    def query(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Query the reference state and reference input at time t.

        Args:
            t: Query time (s)

        Returns:
            x_ref: [x, y, theta] reference state
            u_ref: [v, omega] reference input
        """
        t = np.clip(t, 0.0, self.total_time)

        seg_idx = np.searchsorted(self._cumulative_times, t, side="right") - 1
        seg_idx = np.clip(seg_idx, 0, self.n_wps - 2)

        t_seg_start = self._cumulative_times[seg_idx]
        t_seg = self._seg_times[seg_idx]

        if t_seg < 1e-8:
            alpha = 0.0
        else:
            alpha = (t - t_seg_start) / t_seg
        alpha = np.clip(alpha, 0.0, 1.0)

        p0 = self.waypoints[seg_idx]
        p1 = self.waypoints[seg_idx + 1]
        pos = p0 + alpha * (p1 - p0)

        heading = self._headings[seg_idx]

        dist = self._seg_lengths[seg_idx]
        v_ref = dist / max(t_seg, 1e-8)
        if seg_idx < self.n_wps - 2:
            omega_ref = _normalize_angle(
                self._headings[seg_idx + 1] - self._headings[seg_idx]
            ) / max(t_seg, 1e-8)
        else:
            omega_ref = 0.0

        x_ref = np.array([pos[0], pos[1], heading])
        u_ref = np.array([v_ref, omega_ref])

        return x_ref, u_ref

    def query_horizon(
        self, t: float, N: int, dt: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Query N+1 reference states and N reference inputs within the MPC prediction horizon.

        Args:
            t: Current time (s)
            N: Number of prediction steps
            dt: Prediction step size (s)

        Returns:
            X_ref: (N+1, 3) reference state sequence
            U_ref: (N, 2) reference input sequence
        """
        X_ref = np.zeros((N + 1, 3))
        U_ref = np.zeros((N, 2))

        for k in range(N + 1):
            tk = t + k * dt
            x_ref, u_ref = self.query(tk)
            X_ref[k] = x_ref
            if k < N:
                U_ref[k] = u_ref

        return X_ref, U_ref

    def get_progress(self, t: float) -> float:
        """Return the path completion progress [0, 1] for the current time."""
        if self.total_time < 1e-8:
            return 1.0
        return np.clip(t / self.total_time, 0.0, 1.0)

    def is_finished(self, t: float, pos: np.ndarray, threshold: float = 0.1) -> bool:
        """
        Check whether the goal has been reached.

        Args:
            t: Current time
            pos: Current position [x, y]
            threshold: Distance threshold for reaching the goal (m)
        """
        dist_to_goal = np.linalg.norm(pos[:2] - self.waypoints[-1])
        return t >= self.total_time and dist_to_goal < threshold


def build_synchronized_references(
    waypoints_by_robot: Dict[str, List[List[float]]],
    cfg: Optional[MPCConfig] = None,
) -> Dict[str, ReferenceTrajectory]:
    """
    Build per-robot reference trajectories with synchronized waypoint timing.

    The input paths must have the same number of waypoints and the same waypoint
    index must represent the same formation stage for every robot. For each
    segment, the slowest robot segment time is assigned to every robot, so all
    robots reach waypoint k at approximately the same reference time.

    Args:
        waypoints_by_robot: {robot_name: [[x0,y0], [x1,y1], ...]}
        cfg: MPC configuration used for speed and turn time allocation

    Returns:
        {robot_name: ReferenceTrajectory} with shared segment timing
    """
    if not waypoints_by_robot:
        raise ValueError("waypoints_by_robot cannot be empty")

    cfg = cfg or MPCConfig()

    lengths = {name: len(path) for name, path in waypoints_by_robot.items()}
    unique_lengths = set(lengths.values())
    if len(unique_lengths) != 1:
        raise ValueError(
            "Synchronized references require all robots to have the same "
            f"waypoint count, got {lengths}"
        )

    waypoint_count = next(iter(unique_lengths))
    if waypoint_count < 2:
        raise ValueError("Each robot path needs at least 2 waypoints")

    unsynced_refs = {
        name: ReferenceTrajectory(path, cfg)
        for name, path in waypoints_by_robot.items()
    }
    shared_segment_times = np.maximum.reduce(
        [ref._seg_times for ref in unsynced_refs.values()]
    )
    shared_segment_times = np.maximum(shared_segment_times, cfg.dt)

    return {
        name: ReferenceTrajectory(path, cfg, segment_times=shared_segment_times)
        for name, path in waypoints_by_robot.items()
    }
