"""
Nonlinear MPC Controller
=========================
Uses CasADi Opti stack + IPOPT to solve distributed nonlinear MPC.

Each robot independently instantiates an MPCController to solve its own optimal control problem:

    min sum_{k=0}^{N-1} [
        ||x(k) - x_ref(k)||^2_Q       # Path tracking
      + ||u(k) - u_ref(k)||^2_R       # Control smoothness
      + lambda_f * formation_err(k)   # Formation keeping (optional)
    ] + ||x(N) - x_ref(N)||^2_Q_T     # Terminal cost
      + w_slack * sum(epsilon^2)      # Safety slack penalty

    s.t.  x(k+1) = f(x(k), u(k))     # Dynamics
          v_min <= v(k) <= v_max      # Velocity limits
          |omega(k)| <= omega_max     # Angular velocity limits
          ||x(k) - x_j||^2 >= d_safe^2 + eps  # Inter-robot safety distance
"""

from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Tuple

import casadi as ca
import numpy as np

from .config import MPCConfig
from .dynamics import UnicycleModel
from .formation_keeper import FormationKeeper


def _normalize_angle_casadi(angle: ca.SX) -> ca.SX:
    """CasADi symbolic angle normalization to [-pi, pi]."""
    return ca.atan2(ca.sin(angle), ca.cos(angle))


class MPCController:
    """
    Distributed nonlinear MPC controller (single robot instance).

    Uses CasADi Opti stack to build the optimization problem, solved by IPOPT.
    Supports warm-start to accelerate sequential solves.
    """

    def __init__(
        self,
        robot_id: int,
        cfg: Optional[MPCConfig] = None,
        formation_keeper: Optional[FormationKeeper] = None,
    ):
        """
        Args:
            robot_id: Robot identifier
            cfg: MPC configuration
            formation_keeper: Formation keeping module (None disables explicit formation keeping)
        """
        self.robot_id = robot_id
        self.cfg = cfg or MPCConfig()
        self.formation_keeper = formation_keeper
        self.model = UnicycleModel(self.cfg)

        self._prev_u_sol: Optional[np.ndarray] = None
        self._prev_x_sol: Optional[np.ndarray] = None
        self._solve_count = 0
        self._last_solve_time = 0.0
        self._last_solve_success = False
        self._last_error = ""
        self.last_control_sequence: Optional[np.ndarray] = None
        self.last_predicted_states: Optional[np.ndarray] = None

    def solve(
        self,
        state: np.ndarray,
        X_ref: np.ndarray,
        U_ref: np.ndarray,
        neighbor_states: Optional[Dict[Any, np.ndarray]] = None,
        obstacles: Optional[List[Dict]] = None,
    ) -> Tuple[float, float]:
        """
        Solve one-step MPC and return the optimal control for the current time step.

        Args:
            state: Current state [x, y, theta]
            X_ref: Reference state sequence (N+1, 3)
            U_ref: Reference input sequence (N, 2)
            neighbor_states: {robot_key: [x_j, y_j, theta_j]} neighbor states
            obstacles: [{"position": [x,y], "radius": r}, ...] obstacles

        Returns:
            (v, omega) optimal control input
        """
        t_start = time.time()
        cfg = self.cfg
        N = cfg.N
        NX = UnicycleModel.NX
        NU = UnicycleModel.NU

        opti = ca.Opti()

        # ---- Decision variables ----
        X = opti.variable(NX, N + 1)  # State trajectory
        U = opti.variable(NU, N)       # Control sequence

        # Safety distance slack variables
        n_neighbors = len(neighbor_states) if neighbor_states else 0
        n_obstacles = len(obstacles) if obstacles else 0
        n_slack = n_neighbors + n_obstacles
        if n_slack > 0:
            epsilon = opti.variable(n_slack)
        else:
            epsilon = None

        # ---- Initial state constraint ----
        opti.subject_to(X[:, 0] == state)

        # ---- Dynamics constraints ----
        for k in range(N):
            x_next = self.model.f_discrete(X[:, k], U[:, k])
            opti.subject_to(X[:, k + 1] == x_next)

        # ---- Input constraints ----
        for k in range(N):
            opti.subject_to(opti.bounded(cfg.v_min, U[0, k], cfg.v_max))
            opti.subject_to(opti.bounded(-cfg.omega_max, U[1, k], cfg.omega_max))

        # ---- Cost function ----
        cost = 0.0

        Q = np.array(cfg.Q)
        R = np.array(cfg.R)
        Q_T = np.array(cfg.Q_terminal)

        for k in range(N):
            # State error
            x_err = X[:2, k] - X_ref[k, :2]
            theta_err = _normalize_angle_casadi(X[2, k] - X_ref[k, 2])
            state_err = ca.vertcat(x_err, theta_err)
            cost += ca.mtimes([state_err.T, Q, state_err])

            # Control error
            u_err = U[:, k] - U_ref[k]
            cost += ca.mtimes([u_err.T, R, u_err])

        # Terminal cost
        x_err_T = X[:2, N] - X_ref[N, :2]
        theta_err_T = _normalize_angle_casadi(X[2, N] - X_ref[N, 2])
        state_err_T = ca.vertcat(x_err_T, theta_err_T)
        cost += ca.mtimes([state_err_T.T, Q_T, state_err_T])

        # ---- Formation keeping term (optional) ----
        if (
            self.formation_keeper is not None
            and cfg.lambda_formation > 0
            and neighbor_states
        ):
            for k in range(N):
                heading = X_ref[k, 2]
                cos_h = ca.cos(heading)
                sin_h = ca.sin(heading)

                consensus_pos = ca.SX.zeros(2)
                n_count = 0

                for j, state_j in neighbor_states.items():
                    if j not in self.formation_keeper._relative_offsets:
                        continue
                    offset = self.formation_keeper._relative_offsets[j]
                    rot_dx = offset[0] * cos_h - offset[1] * sin_h
                    rot_dy = offset[0] * sin_h + offset[1] * cos_h
                    consensus_pos += ca.vertcat(
                        state_j[0] + rot_dx,
                        state_j[1] + rot_dy,
                    )
                    n_count += 1

                if n_count > 0:
                    consensus_pos += X_ref[k, :2]
                    consensus_pos /= (n_count + 1)
                    f_err = X[:2, k] - consensus_pos
                    cost += cfg.lambda_formation * ca.dot(f_err, f_err)

        # ---- Safety distance constraints ----
        slack_idx = 0

        if neighbor_states and n_neighbors > 0:
            for j, state_j in neighbor_states.items():
                opti.subject_to(opti.bounded(
                    cfg.slack_min, epsilon[slack_idx], 0.0
                ))
                for k in range(N):
                    dx = X[0, k] - state_j[0]
                    dy = X[1, k] - state_j[1]
                    dist_sq = dx ** 2 + dy ** 2
                    opti.subject_to(
                        dist_sq >= (cfg.d_safe_robot + epsilon[slack_idx]) ** 2
                    )
                cost += cfg.w_slack * epsilon[slack_idx] ** 2
                slack_idx += 1

        if obstacles and n_obstacles > 0:
            for obs in obstacles:
                obs_pos = obs["position"]
                obs_r = obs.get("radius", 0.0)
                safe_dist = cfg.d_safe_obstacle + obs_r
                opti.subject_to(opti.bounded(
                    cfg.slack_min, epsilon[slack_idx], 0.0
                ))
                for k in range(N):
                    dx = X[0, k] - obs_pos[0]
                    dy = X[1, k] - obs_pos[1]
                    dist_sq = dx ** 2 + dy ** 2
                    opti.subject_to(
                        dist_sq >= (safe_dist + epsilon[slack_idx]) ** 2
                    )
                cost += cfg.w_slack * epsilon[slack_idx] ** 2
                slack_idx += 1

        opti.minimize(cost)

        # ---- Solver settings ----
        solver_opts = {
            "ipopt.max_iter": cfg.solver_max_iter,
            "ipopt.print_level": cfg.solver_print_level,
            "print_time": 0,
            "ipopt.acceptable_tol": 1e-4,
            "ipopt.warm_start_init_point": "yes" if cfg.warm_start else "no",
        }
        opti.solver("ipopt", solver_opts)

        # ---- Warm-start ----
        if self._prev_u_sol is not None and cfg.warm_start:
            # Shift initialization: discard first control, duplicate last one at the end
            u_init = np.vstack([self._prev_u_sol[1:], self._prev_u_sol[-1:]])
            for k in range(N):
                opti.set_initial(U[:, k], u_init[k])

            if self._prev_x_sol is not None:
                x_init = np.vstack([self._prev_x_sol[1:], self._prev_x_sol[-1:]])
                for k in range(N + 1):
                    opti.set_initial(X[:, k], x_init[k])
        else:
            # Cold start: initialize with reference trajectory
            for k in range(N + 1):
                opti.set_initial(X[:, k], X_ref[k])
            for k in range(N):
                opti.set_initial(U[:, k], U_ref[k])

        if epsilon is not None:
            for s in range(n_slack):
                opti.set_initial(epsilon[s], 0.0)

        # ---- Solve ----
        try:
            sol = opti.solve()
            u_opt = sol.value(U)
            x_opt = sol.value(X)

            self._prev_u_sol = u_opt.T
            self._prev_x_sol = x_opt.T
            self.last_control_sequence = u_opt.T
            self.last_predicted_states = x_opt.T
            self._last_solve_success = True
            self._last_error = ""
            v_out = float(u_opt[0, 0])
            omega_out = float(u_opt[1, 0])

        except RuntimeError as e:
            # Solve failed: use reference input as fallback, keep x_sol for next warm-start
            import logging
            logging.warning(f"[MPC robot {self.robot_id}] Solver failed: {e}. Using fallback.")
            v_out = float(U_ref[0, 0])
            omega_out = float(U_ref[0, 1])
            self._prev_u_sol = None
            self.last_control_sequence = None
            self.last_predicted_states = None
            self._last_solve_success = False
            self._last_error = str(e)

        self._solve_count += 1
        self._last_solve_time = time.time() - t_start

        return v_out, omega_out

    def reset(self) -> None:
        """Reset warm-start cache."""
        self._prev_u_sol = None
        self._prev_x_sol = None
        self._solve_count = 0
        self._last_solve_success = False
        self._last_error = ""
        self.last_control_sequence = None
        self.last_predicted_states = None

    @property
    def stats(self) -> Dict:
        """Return solver statistics."""
        return {
            "robot_id": self.robot_id,
            "solve_count": self._solve_count,
            "last_solve_time_ms": self._last_solve_time * 1000,
            "last_solve_success": self._last_solve_success,
            "last_error": self._last_error,
        }
