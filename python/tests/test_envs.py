import numpy as np
import pytest

gym = pytest.importorskip("gymnasium")
from gymnasium.utils.env_checker import check_env  # noqa: E402

import fx2d.envs  # noqa: E402,F401

ENV_IDS = ["fx2d/CartPole-v0", "fx2d/Pendulum-v0"]


@pytest.mark.parametrize("env_id", ENV_IDS)
def test_passes_gymnasium_checks(env_id):
    check_env(gym.make(env_id).unwrapped, skip_render_check=True)


@pytest.mark.parametrize("env_id", ENV_IDS)
def test_same_seed_same_episode(env_id):
    def rollout():
        env = gym.make(env_id)
        env.action_space.seed(7)
        obs, _ = env.reset(seed=7)
        out = [obs]
        for _ in range(50):
            obs, *_ = env.step(env.action_space.sample())
            out.append(obs)
        return np.array(out)

    assert np.array_equal(rollout(), rollout())


def test_cartpole_falls_without_control_and_balances_with_it():
    env = gym.make("fx2d/CartPole-v0")

    def episode_length(policy, seed):
        obs, _ = env.reset(seed=seed)
        for n in range(1, 1000):
            obs, _, terminated, truncated, _ = env.step(policy(obs))
            if terminated or truncated:
                return n, truncated

    for seed in range(3):
        n, truncated = episode_length(lambda o: 1, seed)
        assert n < 20 and not truncated
        pd = lambda o: int(10 * o[2] + 2 * o[3] + 0.3 * o[0] + 0.6 * o[1] > 0)  # noqa: E731
        n, truncated = episode_length(pd, seed)
        assert n == 500 and truncated


def test_pendulum_motor_alone_cannot_lift_the_rod():
    env = gym.make("fx2d/Pendulum-v0")
    env.reset(seed=0)
    pendulum = env.unwrapped
    pendulum.scene["rod"].set_state((2.0, 1.5, np.pi), (0.0, 0.0, 0.0))  # hanging straight down
    for _ in range(200):
        obs, *_ = env.step(np.array([2.0], dtype=np.float32))
        assert obs[0] < 0.0  # full torque one way never gets it above level
