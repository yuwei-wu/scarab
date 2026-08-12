"""
PID Controller Parameter Configuration
=======================================
Self-contained configuration for the Scarab PID trajectory tracker.

This package is deliberately independent of scarab_mpc: it shares no Python
modules with it, so the two controllers can be tuned, launched, and modified
without affecting each other. The only thing they have in common is the ROS
interface (pose in, MoveActionGoal in, cmd_vel out).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PIDConfig:
    """Everything the PID tracker and its reference generator need.

    The controller outputs geometry_msgs/Twist-style (v, omega). Wheel
    geometry, encoder conversion, and the low-level motor PID remain owned by
    roboclaw_node and roboclaw/cfg/roboclaw.yaml.

    This is a standard trajectory-tracking PID (Kanayama structure): every
    error is taken against the reference at the current time and expressed in
    the robot body frame. There is no preview/lookahead and no prediction
    horizon -- corner anticipation belongs in the externally supplied
    reference trajectory (denser samples, slower stamps into corners), not in
    the controller.

    Three independent feedback channels:

        along  (e_s) -> corrects v      : timing / schedule error
        cross  (e_n) -> corrects omega  : lateral deviation from the path
        head   (e_t) -> corrects omega  : heading deviation

    Feedforward (v_ref, omega_ref) is always applied on top, which is what
    keeps the robot on schedule instead of permanently lagging the reference.
    """

    # ---- Timing ----
    dt: float = 0.05
    """Nominal control period (s). 0.05 s = 20 Hz. The controller measures its
    real period each cycle; this value is the first-cycle fallback and the
    grid for the RViz prediction rollout. Keep it equal to 1 / control_rate."""

    # ---- Velocity limits ----
    v_min: float = 0.0
    """Minimum linear velocity (m/s). 0.0 means forward-only."""

    v_max: float = 0.25
    """Maximum linear velocity (m/s)."""

    omega_max: float = 0.60
    """Maximum angular velocity (rad/s)."""

    # ---- Reference trajectory ----
    cruise_speed: float = 0.2175
    """Cruise speed (m/s) used to time the waypoints when the goal carries no
    usable per-pose stamps (pub_pid always stamps, so this is a fallback).

    Also the accuracy budget: the controller can only correct an error with
    the velocity authority left between cruise_speed and v_max. It further
    normalizes the cross-track v_r scaling (see the cross-track channel).
    """

    turn_angle_threshold: float = 0.5
    """Vertex heading change (rad) above which a waypoint is a corner.

    At or below this, omega_ref is reconstructed from chord heading
    differences -- dense timed trajectories (gen_trajectory samples at ~3 deg
    steps) rely on that feedforward to hold curves without steady heading
    error. Above it the reconstruction would smear a discrete turn along the
    whole incoming segment and push the robot off the straight, so corners
    get omega_ref = 0 and heading feedback takes them.
    """

    # ---- Along-track channel: drives v, owns schedule accuracy ----
    kp_along: float = 1.2
    ki_along: float = 0.8
    kd_along: float = 0.0

    # ---- Cross-track channel: drives omega, owns path accuracy ----
    # The correction is always scaled by min(1, |v_ref| / cruise_speed) --
    # the v_r factor in Kanayama's law. A differential-drive robot cannot
    # move sideways, so a stationary robot with a lateral offset must not
    # spin in place trying to fix it.
    kp_cross: float = 2.0
    ki_cross: float = 0.0
    kd_cross: float = 0.15

    # ---- Heading channel: drives omega, damps the cross-track loop ----
    kp_heading: float = 1.5
    ki_heading: float = 0.0
    kd_heading: float = 0.05

    # ---- Integral clamps, in output units (m/s for along, rad/s otherwise) ----
    i_max_along: float = 0.08
    i_max_cross: float = 0.15
    i_max_heading: float = 0.15

    # ---- Derivative low-pass time constant (s). 0 disables filtering. ----
    d_filter_tau: float = 0.08

    # ---- Output slew limits (0 disables) ----
    a_max: float = 0.6
    """Linear acceleration limit (m/s^2) applied to the command."""

    alpha_max: float = 2.5
    """Angular acceleration limit (rad/s^2) applied to the command."""

    # ---- Large-heading-error handling ----
    align_angle: float = 1.2
    """Above this heading error (rad) the robot mostly rotates in place.
    While active, the along-track integral is frozen so it cannot wind up."""

    align_speed_factor: float = 0.0
    """Linear speed scale applied while |e_heading| > align_angle. 0.0 = pure
    in-place rotation; raise to ~0.2 if in-place rotation stutters on the
    real floor."""

    # ---- Goal handling ----
    goal_tolerance: float = 0.3
    """Distance (m) within which the final waypoint counts as reached."""

    # ---- Visualization ----
    predict_steps: int = 30
    """Length (in dt steps) of the constant-command rollout published for
    RViz on /<agent>/pid/predicted_path. Display only; no effect on control."""
