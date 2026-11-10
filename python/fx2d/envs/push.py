"""Push a box onto a goal, seen from above."""

import math

import numpy as np
from gymnasium import spaces

import fx2d

from .base import Fx2DEnv

CENTRE = 3.0  # the arena is a 4 m square around (3, 3) in a 6 m world
HALF = 2.0


class PushEnv(Fx2DEnv):
    """Push a box onto a goal with a disc, top-down, in a walled 4 m arena.

    The agent drives a disc by force, up to ``max_force`` along each axis (the action is in
    [-1, 1] on both). The disc and the box slide on a floor that drags them to rest, and touch
    with friction, so a push off the box's centre turns it as well as moving it; the walls stop
    both. The reward is minus the box's distance to the goal and a tenth of the disc's distance
    to the box. Bringing the box's centre within 0.15 m of the goal ends the episode with a
    bonus of 10.

    Observation (13), positions measured from the arena's centre: the disc's position and
    velocity, the box's position, the cos and sin of its angle, its velocity and spin, and the
    goal's position.
    """

    def __init__(self, max_force=10.0, dt=0.05, goal_radius=0.15):
        self.max_force = float(max_force)
        self.dt = float(dt)
        self.goal_radius = float(goal_radius)
        big = np.finfo(np.float32).max
        high = np.full(13, big, dtype=np.float32)
        high[[0, 1, 4, 5, 11, 12]] = HALF
        high[[6, 7]] = 1.0
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)

        self.scene = fx2d.Scene(size=(6, 6), gravity=(0, 0))
        for name, position, size in (
            ("wall_left", (CENTRE - HALF - 0.1, CENTRE), (0.2, 2 * HALF + 0.4)),
            ("wall_right", (CENTRE + HALF + 0.1, CENTRE), (0.2, 2 * HALF + 0.4)),
            ("wall_bottom", (CENTRE, CENTRE - HALF - 0.1), (2 * HALF + 0.4, 0.2)),
            ("wall_top", (CENTRE, CENTRE + HALF + 0.1), (2 * HALF + 0.4, 0.2)),
        ):
            self.scene.add_box(name, position=position, size=size, static=True, elasticity=0.0)
        self.disc = self.scene.add_circle("disc", position=(CENTRE - 1, CENTRE), radius=0.2,
                                          elasticity=0.0, static_friction=0.6,
                                          dynamic_friction=0.5)
        self.box = self.scene.add_box("box", position=(CENTRE + 1, CENTRE), size=(0.5, 0.5),
                                      elasticity=0.0, static_friction=0.6, dynamic_friction=0.5)
        for body in (self.disc, self.box):
            body.damping = 4.0  # floor drag: 10 N moves the 1 kg disc at most 2.5 m/s
        self.goal = np.zeros(2, dtype=np.float32)

    def _observation(self):
        dx, dy, _ = self.disc.pose
        dvx, dvy, _ = self.disc.velocity
        bx, by, angle = self.box.pose
        bvx, bvy, spin = self.box.velocity
        return np.array(
            [dx - CENTRE, dy - CENTRE, dvx, dvy, bx - CENTRE, by - CENTRE, math.cos(angle),
             math.sin(angle), bvx, bvy, spin, self.goal[0], self.goal[1]],
            dtype=np.float32,
        )

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.scene.reset()
        rng = self.np_random
        while True:
            goal = rng.uniform(-1.5, 1.5, size=2)
            box = rng.uniform(-1.2, 1.2, size=2)
            disc = rng.uniform(-1.6, 1.6, size=2)
            if np.linalg.norm(box - goal) > 0.8 and np.linalg.norm(disc - box) > 0.6:
                break
        self.goal = goal.astype(np.float32)
        self.box.set_state((CENTRE + box[0], CENTRE + box[1], rng.uniform(-math.pi, math.pi)),
                           (0.0, 0.0, 0.0))
        self.disc.set_state((CENTRE + disc[0], CENTRE + disc[1], 0.0), (0.0, 0.0, 0.0))
        return self._observation(), {}

    def _act(self, action):
        force = np.clip(np.asarray(action, dtype=np.float32).reshape(2), -1.0, 1.0)
        self.disc.apply_force((float(force[0]) * self.max_force,
                               float(force[1]) * self.max_force))

    def _outcome(self):
        obs = self._observation()
        to_goal = float(np.hypot(obs[4] - obs[11], obs[5] - obs[12]))
        to_box = float(np.hypot(obs[0] - obs[4], obs[1] - obs[5]))
        success = to_goal < self.goal_radius
        reward = -to_goal - 0.1 * to_box + (10.0 if success else 0.0)
        return obs, reward, success, False, {"is_success": success}
