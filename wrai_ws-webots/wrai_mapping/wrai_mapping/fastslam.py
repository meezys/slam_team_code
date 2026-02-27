import numpy as np
from typing import List, Tuple
import random
from copy import deepcopy

from .particle import Particle
from .cone_type import ConeType
from .landmark import Landmark

STATE_SIZE: int = 3


class FastSLAM:
    """Run the FastSLAM Algorithm."""

    def __init__(
        self, start_state: np.ndarray, num_particles: int = 50, *particle_args
    ) -> None:
        self.num_particles: int = num_particles
        self.particles: List[Particle] = []

        self.estimated_state = start_state
        self.estimated_landmarks: List[Landmark] = []

        for _ in range(num_particles):
            self.particles.append(Particle(start_state, *particle_args))

    def step(
        self, motion: np.ndarray, observations: List[Tuple[np.ndarray, ConeType]]
    ) -> None:
        """Perform all the steps to update the filter one step.

        Args:
            motion (np.ndarray): How the vehicle has moved between steps
            observations (List[Tuple[np.ndarray, ConeType]]): Observations of landmarks
        """
        for p in self.particles:
            p.move(motion)  # prediction with motion model
            if len(observations) > 0:
                p.update(observations)  # update from observation

        self.normalize_weights()
        # must be done before resampling because that resets weights
        self.estimated_state = self.estimate_state()
        self.estimated_landmarks = self.estimate_landmarks()

        self.resample_particles()

    def normalize_weights(self) -> None:
        """Normalize particle weights so all weights sum to 1."""
        sum_w = sum([p.weight for p in self.particles])
        if abs(sum_w - 1.0) < 0.01:
            return  # already normalized

        if sum_w == 0.0 or np.isnan(sum_w):
            # weigh uniformly
            new_weight = 1.0 / self.num_particles
            for i in range(self.num_particles):
                self.particles[i].weight = new_weight

        else:
            for i in range(self.num_particles):
                self.particles[i].weight /= sum_w

    def resample_particles(self) -> None:
        """Generate new particles based on the weights of the previous particles.

        uses low variance resampling.
        """
        new_particles = []
        weight = [p.weight for p in self.particles]
        index = int(random.random() * self.num_particles)
        beta = 0.0
        mw = max(weight)
        for _ in range(self.num_particles):
            beta += random.random() * 2.0 * mw
            while beta > weight[index]:
                beta -= weight[index]
                index = (index + 1) % self.num_particles
            new_particle = deepcopy(self.particles[index])
            new_particle.weight = 1
            new_particles.append(new_particle)

        self.particles = new_particles

    def estimate_state(self) -> np.ndarray:
        """Estimate the state using a weighted average of all particles.

        Particle weights must be normalized.

        Returns:
            np.ndarray: the state estimate
        """
        state_estimate = np.zeros(STATE_SIZE)

        # self.normalize_weights()
        particles = self.particles

        sin_sum = 0
        cos_sum = 0
        for i in range(self.num_particles):
            p = particles[i]

            x = p.pos_x
            y = p.pos_y
            heading = p.heading
            w = p.weight

            if np.isnan(x) or np.isnan(y):
                continue

            if np.isnan(w):
                w = 1.0 / self.num_particles

            state_estimate[0] += w * x
            state_estimate[1] += w * y

            # circular mean
            sin_sum += np.sin(heading)
            cos_sum += np.cos(heading)

        state_estimate[2] = np.arctan2(sin_sum, cos_sum)
        return state_estimate

    def estimate_landmarks(self) -> List:
        """Estimates landmarks by taking the particle with the highest weight.

        This could be improved by taking samples from all the particles, but this
        is complex for matching up landmarks between particles, and especially
        dealing with mismatches in numbers of landmarks. This could be fixed with
        some kind of take-all-then-cluster approach.

        Returns:
            List: A list of landmarks
        """
        best_weight = 0.0
        best_particle = self.particles[0]

        for p in self.particles:
            if p.weight > best_weight:
                best_weight = p.weight
                best_particle = p

        return best_particle.landmarks
