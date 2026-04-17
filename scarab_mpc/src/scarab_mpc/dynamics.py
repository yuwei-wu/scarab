"""
Unicycle Dynamics Model (CasADi)
=================================
Provides symbolic discrete-time dynamics functions for MPC.

State:   x = [x, y, theta]
Control: u = [v, omega]
Model:   dx/dt = v * cos(theta)
         dy/dt = v * sin(theta)
         dtheta/dt = omega
"""

from __future__ import annotations

import casadi as ca
import numpy as np
from typing import Optional

from .config import MPCConfig


class UnicycleModel:
    """
    CasADi symbolic unicycle model.

    Provides:
    - continuous_dynamics(): Continuous-time ODE
    - discrete_dynamics(): RK4-discretized one-step transition function
    - Symbolic expressions directly usable in CasADi Opti stack
    """

    NX = 3  # state dim: [x, y, theta]
    NU = 2  # control dim: [v, omega]

    def __init__(self, cfg: Optional[MPCConfig] = None):
        self.cfg = cfg or MPCConfig()
        self.dt = self.cfg.dt
        self._build_symbolic()

    def _build_symbolic(self) -> None:
        """Build CasADi symbolic functions."""
        x = ca.SX.sym("x", self.NX)
        u = ca.SX.sym("u", self.NU)

        # Continuous-time ODE: xdot = f(x, u)
        xdot = ca.vertcat(
            u[0] * ca.cos(x[2]),   # dx/dt = v * cos(theta)
            u[0] * ca.sin(x[2]),   # dy/dt = v * sin(theta)
            u[1],                  # dtheta/dt = omega
        )
        self.f_continuous = ca.Function("f_cont", [x, u], [xdot],
                                        ["x", "u"], ["xdot"])

        # RK4 discretization
        x_next = self._rk4_step(x, u, self.dt)
        self.f_discrete = ca.Function("f_disc", [x, u], [x_next],
                                      ["x", "u"], ["x_next"])

    def _rk4_step(self, x: ca.SX, u: ca.SX, dt: float) -> ca.SX:
        """Single-step RK4 integration."""
        k1 = self.f_continuous(x, u)
        k2 = self.f_continuous(x + dt / 2 * k1, u)
        k3 = self.f_continuous(x + dt / 2 * k2, u)
        k4 = self.f_continuous(x + dt * k3, u)
        return x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    def predict(self, x0: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Numerical one-step prediction (for testing/debugging).

        Args:
            x0: Current state [x, y, theta]
            u: Control input [v, omega]

        Returns:
            Next state [x, y, theta]
        """
        result = self.f_discrete(x0, u)
        return np.array(result).flatten()

    def predict_trajectory(
        self, x0: np.ndarray, u_seq: np.ndarray
    ) -> np.ndarray:
        """
        Numerical multi-step trajectory prediction.

        Args:
            x0: Initial state [x, y, theta]
            u_seq: Control sequence (N, 2)

        Returns:
            State trajectory (N+1, 3), including initial state
        """
        N = u_seq.shape[0]
        traj = np.zeros((N + 1, self.NX))
        traj[0] = x0
        for k in range(N):
            traj[k + 1] = self.predict(traj[k], u_seq[k])
        return traj
