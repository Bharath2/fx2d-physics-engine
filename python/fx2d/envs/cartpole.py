"""Balance a pole on a cart that slides along a rail."""

import math

import numpy as np
from gymnasium import spaces

import fx2d

from .base import Fx2DEnv


class CartPoleEnv(Fx2DEnv):
    """The cart-pole balancing task, simulated by Fx2D.

    A pole is hinged on top of a cart that slides along a frictionless rail. Each step pushes the
    cart left (action 0) or right (action 1) with a fixed force through the rail's motor, and
    earns a reward of 1. The episode ends when the pole leans more than 12 degrees or the cart
    strays more than 2.4 m from the middle of the track.

    Observation: cart position, cart velocity, pole angle (positive when leaning toward +x) and
    the pole's angular velocity.
    """

    def __init__(self, force=10.0, dt=0.02):
        self.force = float(force)
        self.dt = float(dt)
        self.x_limit = 2.4
        self.theta_limit = math.radians(12.0)
        big = np.finfo(np.float32).max
        high = np.array([2 * self.x_limit, big, 2 * self.theta_limit, big], dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        self.action_space = spaces.Discrete(2)

        self.scene = fx2d.Scene(size=(12, 6), gravity=(0, -9.8))
        self.scene.add_box("rail", position=(6, 1), size=(10, 0.05), static=True)
        self.scene.add_box("cart", position=(6, 2), size=(0.5, 0.3), mass=1.0)
        self.scene.add_box("pole", position=(6, 2.5), size=(0.05, 1.0), mass=0.1)
        self.slider = self.scene.add_prismatic_joint("slider", "rail", "cart", axis=(1, 0))
        self.hinge = self.scene.add_revolute_joint("hinge", "cart", "pole", anchor=(0, 0))
        for joint in (self.slider, self.hinge):
            joint.control_mode = fx2d.ControlMode.EFFORT  # no motor until the agent pushes
        self.hinge.set_torque(0.0)
        self.slider.set_force(0.0)

    def _observation(self):
        return np.array(
            [self.slider.position, self.slider.velocity, -self.hinge.angle,
             -self.hinge.angular_velocity],
            dtype=np.float32,
        )

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.scene.reset()
        x, x_dot, theta, theta_dot = self.np_random.uniform(-0.05, 0.05, size=4)
        cart = (6.0 + x, 2.0)
        self.scene["cart"].set_state((cart[0], cart[1], 0.0), (x_dot, 0.0, 0.0))
        # The scene measures angles counter-clockwise; the observation leans toward +x.
        angle, spin = -theta, -theta_dot
        r = (-0.5 * math.sin(angle), 0.5 * math.cos(angle))
        self.scene["pole"].set_state(
            (cart[0] + r[0], cart[1] + r[1], angle),
            (x_dot - spin * r[1], spin * r[0], spin),
        )
        return self._observation(), {}

    def _act(self, action):
        if not self.action_space.contains(action):
            raise ValueError(f"action must be 0 or 1, got {action!r}")
        self.slider.set_force(self.force if action == 1 else -self.force)

    def _outcome(self):
        obs = self._observation()
        terminated = bool(abs(obs[0]) > self.x_limit or abs(obs[2]) > self.theta_limit)
        return obs, 1.0, terminated, False, {}
