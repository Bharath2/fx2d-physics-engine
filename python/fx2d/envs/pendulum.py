"""Swing a pendulum up and hold it upright."""

import math

import numpy as np
from gymnasium import spaces

import fx2d

from .base import Fx2DEnv


class PendulumEnv(Fx2DEnv):
    """Pendulum swing-up, simulated by Fx2D.

    A 1 m, 1 kg rod hangs from a hinge whose motor gives at most 2 N m, less than gravity's
    5 N m pull on a level rod, so the rod must be swung up rather than lifted. The reward
    penalises the angle from upright, the spin and the torque used:
    -(angle^2 + 0.1 spin^2 + 0.001 torque^2).

    Observation: cos and sin of the rod's angle from upright, and its angular velocity.
    Action: the motor torque in N m.
    """

    def __init__(self, max_torque=2.0, dt=0.05, gravity=10.0):
        self.max_torque = float(max_torque)
        self.dt = float(dt)
        high = np.array([1.0, 1.0, np.finfo(np.float32).max], dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        self.action_space = spaces.Box(-self.max_torque, self.max_torque, shape=(1,),
                                       dtype=np.float32)

        self.scene = fx2d.Scene(size=(4, 4), gravity=(0, -float(gravity)))
        self.scene.add_box("pivot", position=(2, 2), size=(0.1, 0.1), static=True)
        self.scene.add_box("rod", position=(2, 2.5), size=(0.1, 1.0), mass=1.0)
        self.hinge = self.scene.add_revolute_joint("hinge", "pivot", "rod", anchor=(0, 0))
        self.hinge.control_mode = fx2d.ControlMode.EFFORT
        self.hinge.set_torque(0.0)
        self._cost = 0.0

    def _observation(self):
        angle = self.hinge.angle
        return np.array([math.cos(angle), math.sin(angle), self.hinge.angular_velocity],
                        dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.scene.reset()
        angle = self.np_random.uniform(-math.pi, math.pi)
        spin = self.np_random.uniform(-1.0, 1.0)
        r = (-0.5 * math.sin(angle), 0.5 * math.cos(angle))  # pivot to the rod's centre
        self.scene["rod"].set_state((2.0 + r[0], 2.0 + r[1], angle),
                                    (-spin * r[1], spin * r[0], spin))
        return self._observation(), {}

    def _act(self, action):
        torque = float(np.clip(np.asarray(action, dtype=np.float32).reshape(-1)[0],
                               -self.max_torque, self.max_torque))
        angle, spin = self.hinge.angle, self.hinge.angular_velocity
        self._cost = angle ** 2 + 0.1 * spin ** 2 + 0.001 * torque ** 2  # the state acted on
        self.hinge.set_torque(torque)

    def _outcome(self):
        return self._observation(), -self._cost, False, False, {}
