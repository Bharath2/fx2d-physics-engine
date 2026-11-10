"""Gymnasium environments simulated by Fx2D. Importing this module registers them."""

from gymnasium.envs.registration import register

from .cartpole import CartPoleEnv
from .pendulum import PendulumEnv

register(id="fx2d/CartPole-v0", entry_point="fx2d.envs:CartPoleEnv", max_episode_steps=500)
register(id="fx2d/Pendulum-v0", entry_point="fx2d.envs:PendulumEnv", max_episode_steps=200)

__all__ = ["CartPoleEnv", "PendulumEnv"]
