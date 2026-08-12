"""
Reference Trajectory Generator
================================
Converts discrete waypoints into a time-parameterized reference so the PID
controller can ask "where should I be right now, and how fast should I be
going".

Piecewise linear: position interpolates straight between adjacent waypoints
and heading is the chord heading. omega_ref is reconstructed from the heading
difference between adjacent chords -- for a densely sampled smooth trajectory
(pub_pid/gen_trajectory.py samples so consecutive chords turn by only a few
degrees) this converges to the true turn rate and gives the controller its
omega feedforward. At a real corner (vertex turn above turn_angle_threshold)
the reconstruction is meaningless: smearing a 90 deg turn along the whole
incoming segment would steer the robot sideways off the straight, so corners
get omega_ref = 0 and the heading feedback takes them instead.

Past the final time the reference pins to the goal with zero feedforward, so
the controller regulates onto the endpoint instead of being pushed through it
at cruise speed.

Segment timing normally comes from the externally supplied trajectory (goal
stamps published by pub_pid); the internal distance / cruise_speed allocation
is only a fallback for unstamped legacy goals.

This is a standalone module owned by scarab_pid. It intentionally does not
import from scarab_mpc so the two controllers stay independent.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np

from .config import PIDConfig


def _normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


class ReferenceTrajectory:
    """Time-parameterized reference trajectory built from discrete waypoints.

    Time allocation:
      - segment_times supplied externally (from the goal's per-pose stamps)
        is the normal case; robots given the same stamps share one timeline,
        which is what keeps a multi-robot formation synchronized
      - fallback: segment time = distance / cruise_speed
    """

    def __init__(
        self,
        waypoints: List[List[float]],
        cfg: Optional[PIDConfig] = None,
        segment_times: Optional[np.ndarray] = None,
    ):
        """
        Args:
            waypoints: [[x0,y0], [x1,y1], ...] ordered 2D waypoints
            cfg: PID configuration (cruise speed, corner threshold, dt)
            segment_times: externally provided per-segment durations, length
                len(waypoints)-1. If None, computed from cruise_speed.
        """
        self.cfg = cfg or PIDConfig()
        self.waypoints = np.array(waypoints, dtype=np.float64)
        self.n_wps = len(self.waypoints)

        if self.n_wps < 2:
            raise ValueError(
                "At least 2 waypoints are required to build a reference trajectory"
            )

        self._seg_lengths = self._compute_segment_lengths()
        self._headings = self._compute_headings()

        if segment_times is not None:
            if len(segment_times) != self.n_wps - 1:
                raise ValueError(
                    f"segment_times length ({len(segment_times)}) "
                    f"!= waypoints-1 ({self.n_wps - 1})"
                )
            self._seg_times = np.array(segment_times, dtype=np.float64)
        else:
            self._seg_times = self._compute_segment_times()

        self._cumulative_times = np.concatenate([[0.0], np.cumsum(self._seg_times)])
        self.total_time = float(self._cumulative_times[-1])

    # ------------------------------------------------------------------
    # Construction helpers
    # ------------------------------------------------------------------
    def _compute_segment_lengths(self) -> np.ndarray:
        diffs = np.diff(self.waypoints, axis=0)
        return np.linalg.norm(diffs, axis=1)

    def _compute_headings(self) -> np.ndarray:
        """Per-segment chord headings; the last entry repeats for the goal.

        Degenerate (zero-length) segments inherit the nearest real segment's
        heading instead of atan2(0, 0) = 0, so a duplicated waypoint -- e.g. a
        commanded pause -- does not swing the reference heading toward 0.
        """
        headings = np.zeros(self.n_wps)
        valid = self._seg_lengths > 1e-8
        if not bool(valid.any()):
            return headings

        first_valid = int(np.argmax(valid))
        diff = self.waypoints[first_valid + 1] - self.waypoints[first_valid]
        last = math.atan2(diff[1], diff[0])
        for i in range(self.n_wps - 1):
            if valid[i]:
                diff = self.waypoints[i + 1] - self.waypoints[i]
                last = math.atan2(diff[1], diff[0])
            headings[i] = last
        headings[-1] = headings[-2]
        return headings

    def _compute_segment_times(self) -> np.ndarray:
        """Fallback constant-speed timing: distance / cruise_speed.

        Only used when the goal carries no usable stamps. There is no turn
        slowdown here: corner pacing belongs in the supplied stamps (pub_pid
        applies its own slowdown when it generates them), and sharp corners
        are taken by the controller's align-in-place mode anyway. Degenerate
        segments get one control period.
        """
        speed = max(self.cfg.cruise_speed, 0.01)
        dt_floor = self.cfg.dt if self.cfg.dt > 0.0 else 0.05
        return np.maximum(self._seg_lengths / speed, dt_floor)

    # ------------------------------------------------------------------
    # Query interface
    # ------------------------------------------------------------------
    def query(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        """Reference state and input at time t.

        Past the end of the schedule the reference is the goal itself with
        zero feedforward: the controller then regulates onto the endpoint
        (and ramps down through its slew limits) instead of being pushed
        through it at cruise speed.

        Returns:
            x_ref: [x, y, theta]
            u_ref: [v, omega]
        """
        if t >= self.total_time:
            goal = self.waypoints[-1]
            return (
                np.array([goal[0], goal[1], self._headings[-1]]),
                np.array([0.0, 0.0]),
            )

        t = max(float(t), 0.0)
        seg_idx = int(np.searchsorted(self._cumulative_times, t, side="right") - 1)
        seg_idx = int(np.clip(seg_idx, 0, self.n_wps - 2))

        t_seg_start = self._cumulative_times[seg_idx]
        t_seg = self._seg_times[seg_idx]

        alpha = 0.0 if t_seg < 1e-8 else (t - t_seg_start) / t_seg
        alpha = float(np.clip(alpha, 0.0, 1.0))

        p0 = self.waypoints[seg_idx]
        p1 = self.waypoints[seg_idx + 1]
        pos = p0 + alpha * (p1 - p0)

        v_ref = self._seg_lengths[seg_idx] / max(t_seg, 1e-8)

        # omega_ref: chord-difference reconstruction, gated by vertex angle.
        # Small turns are samples of a smooth curve -> true turn-rate
        # feedforward. Large turns are polyline corners -> no feedforward
        # (heading feedback + align mode take those).
        omega_ref = 0.0
        if seg_idx < self.n_wps - 2:
            turn = _normalize_angle(
                self._headings[seg_idx + 1] - self._headings[seg_idx]
            )
            if abs(turn) <= self.cfg.turn_angle_threshold:
                omega_ref = turn / max(t_seg, 1e-8)

        return (
            np.array([pos[0], pos[1], self._headings[seg_idx]]),
            np.array([v_ref, omega_ref]),
        )

    def get_progress(self, t: float) -> float:
        """Path completion progress in [0, 1]."""
        if self.total_time < 1e-8:
            return 1.0
        return float(np.clip(t / self.total_time, 0.0, 1.0))

    def is_finished(self, t: float, pos: np.ndarray, threshold: float = 0.1) -> bool:
        """True once the schedule has elapsed AND the goal is close enough."""
        dist_to_goal = float(np.linalg.norm(np.asarray(pos)[:2] - self.waypoints[-1]))
        return t >= self.total_time and dist_to_goal < threshold
