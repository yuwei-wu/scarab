"""
Formation Keeping Module
=========================
Consensus-theory-based formation error computation, referencing DEFORM paper Eq. (6).

Provides two modes:
1. Implicit formation: LLM waypoints already contain formation info; tracking naturally maintains formation
2. Explicit formation: Computes consensus reference state as an additional term in the MPC cost function

Formula:
    x_i^r = 1/(N_i + 1) * (sum_{j in N_i} (x_j + p_{f,ij}) + x_ref_i)
    formation_error_i = ||x_i - x_i^r||^2

where p_{f,ij} = p_{f,i} - p_{f,j} is the relative position in formation coordinates.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np


class FormationKeeper:
    """
    Formation keeping calculator.

    Uses positions from FormationPattern to compute desired relative positions
    between robots, and calculates consensus-based formation reference states.
    """

    def __init__(
        self,
        formation_positions: List[Tuple[float, float]],
        robot_index: int,
    ):
        """
        Args:
            formation_positions: Relative positions of each robot in the formation
                                 [(dx0,dy0), (dx1,dy1), ...], index 0 is the leader
            robot_index: Index of this robot in the formation
        """
        self.positions = np.array(formation_positions, dtype=np.float64)
        self.num_robots = len(formation_positions)
        self.robot_index = robot_index

        self._relative_offsets = self._compute_relative_offsets()

    def _compute_relative_offsets(self) -> Dict[int, np.ndarray]:
        """
        Pre-compute desired relative positions p_{f,ij} between this robot and others.

        Returns:
            {j: p_{f,ij}} dictionary, where p_{f,ij} = p_{f,i} - p_{f,j}
        """
        i = self.robot_index
        offsets = {}
        for j in range(self.num_robots):
            if j != i:
                offsets[j] = self.positions[i] - self.positions[j]
        return offsets

    def compute_consensus_reference(
        self,
        neighbor_states: Dict[int, np.ndarray],
        waypoint_ref: np.ndarray,
        heading: float,
    ) -> np.ndarray:
        """
        Compute consensus-based formation reference state x_i^r.

        x_i^r = 1/(N_i + 1) * (sum_{j} (x_j + R(heading) * p_{f,ij}) + x_ref_i)

        where R(heading) rotates the formation coordinate offsets to the world frame.

        Args:
            neighbor_states: {robot_j: [x_j, y_j, theta_j]} current neighbor states
            waypoint_ref: [x_ref, y_ref, theta_ref] this robot's waypoint reference
            heading: Overall formation heading (rad), used to rotate formation coordinates

        Returns:
            x_ref_consensus: [x, y, theta] consensus reference state
        """
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)

        sum_pos = np.zeros(2)
        sin_sum = 0.0
        cos_sum = 0.0
        n_neighbors = 0

        for j, state_j in neighbor_states.items():
            if j not in self._relative_offsets:
                continue
            offset = self._relative_offsets[j]
            # Rotate to world frame
            rotated_dx = offset[0] * cos_h - offset[1] * sin_h
            rotated_dy = offset[0] * sin_h + offset[1] * cos_h
            sum_pos[0] += state_j[0] + rotated_dx
            sum_pos[1] += state_j[1] + rotated_dy
            sin_sum += math.sin(state_j[2])
            cos_sum += math.cos(state_j[2])
            n_neighbors += 1

        if n_neighbors == 0:
            return waypoint_ref.copy()

        sum_pos += waypoint_ref[:2]
        consensus_xy = sum_pos / (n_neighbors + 1)

        # Use circular mean to compute angular consensus
        sin_sum += math.sin(waypoint_ref[2])
        cos_sum += math.cos(waypoint_ref[2])
        consensus_theta = math.atan2(sin_sum, cos_sum)

        return np.array([consensus_xy[0], consensus_xy[1], consensus_theta])

    def compute_formation_error(
        self,
        current_state: np.ndarray,
        neighbor_states: Dict[int, np.ndarray],
        waypoint_ref: np.ndarray,
        heading: float,
    ) -> float:
        """
        Compute current formation error ||x_i - x_i^r||^2 (position part only).

        Args:
            current_state: This robot's current state [x, y, theta]
            neighbor_states: Neighbor states dictionary
            waypoint_ref: This robot's waypoint reference
            heading: Formation heading

        Returns:
            Squared sum of formation position error
        """
        ref = self.compute_consensus_reference(
            neighbor_states, waypoint_ref, heading
        )
        pos_error = current_state[:2] - ref[:2]
        return float(np.dot(pos_error, pos_error))

    def compute_desired_position(
        self,
        leader_pos: np.ndarray,
        heading: float,
    ) -> np.ndarray:
        """
        Compute this robot's desired world position based on leader position and formation heading.

        Args:
            leader_pos: [x, y] leader world coordinates
            heading: Formation heading (rad)

        Returns:
            [x, y] this robot's desired position
        """
        offset = self.positions[self.robot_index]
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        dx = offset[0] * cos_h - offset[1] * sin_h
        dy = offset[0] * sin_h + offset[1] * cos_h
        return np.array([leader_pos[0] + dx, leader_pos[1] + dy])

    @staticmethod
    def compute_inter_robot_distances(
        states: Dict[int, np.ndarray],
    ) -> Dict[Tuple[int, int], float]:
        """
        Compute distances between all robot pairs (for safety checking).

        Args:
            states: {robot_id: [x, y, theta]}

        Returns:
            {(i, j): distance} dictionary
        """
        ids = sorted(states.keys())
        distances = {}
        for idx_a in range(len(ids)):
            for idx_b in range(idx_a + 1, len(ids)):
                i, j = ids[idx_a], ids[idx_b]
                dist = float(np.linalg.norm(
                    states[i][:2] - states[j][:2]
                ))
                distances[(i, j)] = dist
        return distances
