"""Gymnasium environments simulated by Fx2D. Importing this module registers them.

``gym.make("fx2d/Push-v0")`` builds one environment; ``gym.make_vec("fx2d/Push-v0",
num_envs=64)`` builds an ``Fx2DVectorEnv`` that steps all 64 scenes in one parallel call.
"""

from functools import partial

from gymnasium.envs.registration import register

from .base import Fx2DEnv, Fx2DVectorEnv
from .cartpole import CartPoleEnv
from .pendulum import PendulumEnv
from .push import PushEnv

for _name, _env, _steps in (
    ("CartPole", CartPoleEnv, 500),
    ("Pendulum", PendulumEnv, 200),
    ("Push", PushEnv, 200),
):
    register(
        id=f"fx2d/{_name}-v0",
        entry_point=_env,
        vector_entry_point=partial(Fx2DVectorEnv, _env),
        max_episode_steps=_steps,
    )

__all__ = ["CartPoleEnv", "Fx2DEnv", "Fx2DVectorEnv", "PendulumEnv", "PushEnv"]
