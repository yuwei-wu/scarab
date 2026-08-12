"""
Trajectory-tracking PID Controller
===================================
Standard feedforward + PID on body-frame tracking errors, for a
differential-drive (unicycle) robot following a time-parameterized
reference. This is the classic Kanayama structure: every error is taken
against the reference at the current time. There is deliberately no
preview/lookahead and no prediction horizon (both were measured and removed;
see config/pid.yaml).

    e_s =  cos(t)*(x_ref - x) + sin(t)*(y_ref - y)     along-track (schedule)
    e_n = -sin(t)*(x_ref - x) + cos(t)*(y_ref - y)     cross-track (path)
    e_t = wrap(theta_ref - theta)                      heading

    v     = v_ref * cos(e_t)  +  PID_along(e_s)
    omega = omega_ref  +  gate * PID_cross(e_n)  +  PID_head(e_t)

Why the feedforward matters: a plain PID on position error always trails a
moving reference by roughly v_ref / kp. Here v_ref and omega_ref are supplied
directly, so the feedback only has to reject disturbances, and the
along-track integral removes whatever residual lag comes from motor deadband,
friction, and the roboclaw velocity loop. That residual lag is exactly what
desynchronizes two robots carrying or pushing a shared load.

No dependency on scarab_mpc, CasADi, or IPOPT: this is pure numpy and runs in
tens of microseconds.
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, Optional, Tuple

import numpy as np

from .config import PIDConfig


def _wrap(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class _Channel:
    """One PID channel with a clamped integral and a filtered derivative.

    The most recent integral increment is remembered so the caller can roll it
    back when the summed command saturates (conditional anti-windup).
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        i_max: float,
        d_filter_tau: float,
    ) -> None:
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.i_max = abs(float(i_max))
        self.d_filter_tau = max(0.0, float(d_filter_tau))
        self._integral = 0.0
        self._last_error: Optional[float] = None
        self._d_filtered = 0.0
        self._last_increment = 0.0

    def reset(self) -> None:
        self._integral = 0.0
        self._last_error = None
        self._d_filtered = 0.0
        self._last_increment = 0.0

    def step(self, error: float, dt: float) -> float:
        error = float(error)

        # ---- Derivative on error, low-pass filtered (pose data is noisy) ----
        if self._last_error is None or dt <= 0.0:
            raw_derivative = 0.0
        else:
            raw_derivative = (error - self._last_error) / dt
        if self.d_filter_tau > 0.0 and dt > 0.0:
            alpha = dt / (self.d_filter_tau + dt)
            self._d_filtered += alpha * (raw_derivative - self._d_filtered)
        else:
            self._d_filtered = raw_derivative
        self._last_error = error

        # ---- Integral with hard clamp ----
        if self.ki != 0.0 and dt > 0.0:
            clamped = _clip(self._integral + self.ki * error * dt,
                            -self.i_max, self.i_max)
        else:
            clamped = self._integral
        self._last_increment = clamped - self._integral
        self._integral = clamped

        return self.kp * error + self._integral + self.kd * self._d_filtered

    def undo_integral(self) -> None:
        """Roll back the most recent integral increment (anti-windup)."""
        self._integral -= self._last_increment
        self._last_increment = 0.0

    @property
    def integral(self) -> float:
        return self._integral


class PIDTrajectoryController:
    """Trajectory-tracking PID for one differential-drive robot."""

    def __init__(
        self,
        robot_id: Any = 0,
        cfg: Optional[PIDConfig] = None,
    ) -> None:
        self.robot_id = robot_id
        self.cfg = cfg or PIDConfig()

        cfg_ = self.cfg
        self._along = _Channel(
            cfg_.kp_along, cfg_.ki_along, cfg_.kd_along,
            cfg_.i_max_along, cfg_.d_filter_tau,
        )
        self._cross = _Channel(
            cfg_.kp_cross, cfg_.ki_cross, cfg_.kd_cross,
            cfg_.i_max_cross, cfg_.d_filter_tau,
        )
        self._heading = _Channel(
            cfg_.kp_heading, cfg_.ki_heading, cfg_.kd_heading,
            cfg_.i_max_heading, cfg_.d_filter_tau,
        )

        self._solve_count = 0
        self._last_solve_time = 0.0
        self._prev_v = 0.0
        self._prev_omega = 0.0
        self._last_wall_time: Optional[float] = None
        self._diagnostics: Dict[str, Any] = {}

        self.last_predicted_states: Optional[np.ndarray] = None

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------
    def compute(
        self,
        state: np.ndarray,
        x_ref: np.ndarray,
        u_ref: np.ndarray,
        dt: Optional[float] = None,
    ) -> Tuple[float, float]:
        """Compute one control step.

        Args:
            state: current state [x, y, theta]
            x_ref: reference state [x, y, theta] at the current time
            u_ref: reference input [v, omega] at the current time
            dt: elapsed time since the previous call (s). When None, wall-clock
                time is measured internally. Pass it explicitly under
                /use_sim_time.

        Returns:
            (v, omega) command
        """
        t_start = time.time()
        cfg = self.cfg
        step_dt = self._resolve_dt(dt)

        x, y, theta = float(state[0]), float(state[1]), float(state[2])
        cos_t, sin_t = math.cos(theta), math.sin(theta)

        # ravel() also accepts an old-style (N, 3) horizon array and reads its
        # first row, so stray callers degrade gracefully instead of crashing.
        ref = np.asarray(x_ref, dtype=np.float64).ravel()
        u = np.asarray(u_ref, dtype=np.float64).ravel()
        v_ref = float(u[0]) if u.size >= 1 else 0.0
        omega_ref = float(u[1]) if u.size >= 2 else 0.0

        # ---- Body-frame errors, all at the current reference time ---------
        dx = float(ref[0]) - x
        dy = float(ref[1]) - y
        e_along = cos_t * dx + sin_t * dy
        e_cross = -sin_t * dx + cos_t * dy
        e_heading = _wrap(float(ref[2]) - theta)

        # ---- Linear velocity ----------------------------------------------
        v_feedforward = v_ref * math.cos(e_heading)
        v_feedback = self._along.step(e_along, step_dt)
        v_raw = v_feedforward + v_feedback

        # A large heading error means driving forward would take the robot
        # further off the path; rotate in place first. The along-track
        # integral increment is rolled back so it cannot wind up while the
        # robot is deliberately not driving.
        aligning = abs(e_heading) > cfg.align_angle
        if aligning:
            v_raw *= cfg.align_speed_factor
            self._along.undo_integral()

        # ---- Angular velocity ----------------------------------------------
        # Kanayama's v_r factor: lateral correction authority scales with the
        # reference speed. A stationary robot with a lateral offset must not
        # spin in place -- a differential drive cannot move sideways.
        gate = min(1.0, abs(v_ref) / max(cfg.cruise_speed, 1e-6))

        omega_feedback = (
            gate * self._cross.step(e_cross, step_dt)
            + self._heading.step(e_heading, step_dt)
        )
        omega_raw = omega_ref + omega_feedback

        # ---- Saturation with conditional anti-windup -----------------------
        v_cmd = _clip(v_raw, cfg.v_min, cfg.v_max)
        omega_cmd = _clip(omega_raw, -cfg.omega_max, cfg.omega_max)

        v_excess = v_raw - v_cmd
        if v_excess * e_along > 0.0:
            self._along.undo_integral()

        omega_excess = omega_raw - omega_cmd
        if omega_excess * e_cross > 0.0:
            self._cross.undo_integral()
        if omega_excess * e_heading > 0.0:
            self._heading.undo_integral()

        # ---- Slew limits ----------------------------------------------------
        if cfg.a_max > 0.0:
            dv = cfg.a_max * step_dt
            v_cmd = _clip(v_cmd, self._prev_v - dv, self._prev_v + dv)
        if cfg.alpha_max > 0.0:
            dw = cfg.alpha_max * step_dt
            omega_cmd = _clip(omega_cmd, self._prev_omega - dw,
                              self._prev_omega + dw)

        self._prev_v = v_cmd
        self._prev_omega = omega_cmd

        # ---- Bookkeeping -----------------------------------------------------
        self.last_predicted_states = self._rollout(
            np.array([x, y, theta]), v_cmd, omega_cmd
        )

        self._diagnostics = {
            "e_along": float(e_along),
            "e_cross": float(e_cross),
            "e_heading": float(e_heading),
            "i_along": float(self._along.integral),
            "i_cross": float(self._cross.integral),
            "i_heading": float(self._heading.integral),
            "v_feedforward": float(v_feedforward),
            "v_feedback": float(v_feedback),
            "omega_feedforward": float(omega_ref),
            "omega_feedback": float(omega_feedback),
            "aligning": bool(aligning),
            "step_dt": float(step_dt),
            "saturated_v": bool(abs(v_excess) > 1e-9),
            "saturated_omega": bool(abs(omega_excess) > 1e-9),
        }

        self._solve_count += 1
        self._last_solve_time = time.time() - t_start

        return float(v_cmd), float(omega_cmd)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _resolve_dt(self, dt: Optional[float]) -> float:
        """Return a sane control period, measuring wall time when not given."""
        now = time.monotonic()
        if dt is not None and dt > 0.0:
            measured = float(dt)
        elif self._last_wall_time is None:
            measured = self.cfg.dt
        else:
            measured = now - self._last_wall_time
        self._last_wall_time = now

        nominal = self.cfg.dt if self.cfg.dt > 0.0 else 0.05
        return _clip(measured, 0.2 * nominal, 5.0 * nominal)

    def _rollout(self, state: np.ndarray, v: float, omega: float) -> np.ndarray:
        """Constant-command unicycle rollout, for the RViz predicted path.

        This is an honest picture of what the PID is doing: it has no planned
        control sequence, so the prediction is simply "hold the current
        command". Display only; no effect on control.
        """
        steps = max(int(self.cfg.predict_steps), 1)
        dt = self.cfg.dt if self.cfg.dt > 0.0 else 0.05
        states = np.zeros((steps + 1, 3), dtype=np.float64)
        states[0] = state

        x, y, theta = float(state[0]), float(state[1]), float(state[2])
        for k in range(1, steps + 1):
            if abs(omega) < 1e-6:
                x += v * math.cos(theta) * dt
                y += v * math.sin(theta) * dt
            else:
                theta_next = theta + omega * dt
                x += (v / omega) * (math.sin(theta_next) - math.sin(theta))
                y -= (v / omega) * (math.cos(theta_next) - math.cos(theta))
                theta = theta_next
            states[k] = (x, y, _wrap(theta))
        return states

    def reset(self) -> None:
        """Clear integrators, derivative history, and slew state."""
        self._along.reset()
        self._cross.reset()
        self._heading.reset()
        self._solve_count = 0
        self._prev_v = 0.0
        self._prev_omega = 0.0
        self._last_wall_time = None
        self._diagnostics = {}
        self.last_predicted_states = None

    @property
    def stats(self) -> Dict:
        """Diagnostics for /<agent>/pid/status.

        Every value is a plain Python scalar so the node can json.dumps it
        without a numpy encoder.
        """
        payload: Dict[str, Any] = {
            "robot_id": self.robot_id,
            "controller": "pid",
            "solve_count": self._solve_count,
            "last_solve_time_ms": self._last_solve_time * 1000.0,
        }
        payload.update(self._diagnostics)
        return payload
