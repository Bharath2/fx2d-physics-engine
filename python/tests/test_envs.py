import numpy as np
import pytest

gym = pytest.importorskip("gymnasium")
from gymnasium.utils.env_checker import check_env  # noqa: E402

import fx2d.envs  # noqa: E402,F401

ENV_IDS = ["fx2d/CartPole-v0", "fx2d/Pendulum-v0", "fx2d/Push-v0"]


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


@pytest.mark.parametrize("env_id", ENV_IDS)
def test_vector_env_matches_gymnasium_sync_vector_env(env_id):
    fast = gym.make_vec(env_id, num_envs=6, vectorization_mode="vector_entry_point")
    sync = gym.make_vec(env_id, num_envs=6, vectorization_mode="sync")
    assert isinstance(fast.unwrapped, fx2d.envs.Fx2DVectorEnv)
    assert np.array_equal(fast.reset(seed=3)[0], sync.reset(seed=3)[0])
    fast.action_space.seed(1)
    for _ in range(250):  # long enough for episodes to end and autoreset
        actions = fast.action_space.sample()
        for a, b in zip(fast.step(actions)[:4], sync.step(actions)[:4]):
            assert np.array_equal(a, b)


def test_vector_env_cuts_episodes_and_resets_next_step():
    env = gym.make_vec("fx2d/Pendulum-v0", num_envs=4, vectorization_mode="vector_entry_point")
    env.reset(seed=0)
    zero = np.zeros((4, 1), dtype=np.float32)
    for _ in range(199):
        _, _, terminated, truncated, _ = env.step(zero)
        assert not terminated.any() and not truncated.any()
    _, _, _, truncated, _ = env.step(zero)
    assert truncated.all()  # 200 steps
    _, rewards, _, truncated, _ = env.step(zero)
    assert not truncated.any() and (rewards == 0).all()  # the reset step


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


def push_behind_the_box(obs):
    """Get behind the box, as seen from the goal, then drive through it."""
    disc, velocity, box, goal = obs[0:2], obs[2:4], obs[4:6], obs[11:13]
    toward = (goal - box) / (np.linalg.norm(goal - box) + 1e-9)
    offset = disc - box
    if offset @ toward > -0.3:  # on the wrong side: go round
        side = np.array([-toward[1], toward[0]])
        side = side if offset @ side >= 0 else -side
        target = box + 0.7 * side - 0.3 * toward
    elif np.linalg.norm(disc - (box - 0.55 * toward)) > 0.2:
        target = box - 0.55 * toward
    else:
        target = box + 0.6 * toward
    return np.clip(3.0 * (target - disc) - 0.8 * velocity, -1, 1).astype(np.float32)


def test_push_is_solved_by_pushing_and_not_by_chance():
    env = gym.make("fx2d/Push-v0")
    rng = np.random.default_rng(0)

    def solved(policy):
        count = 0
        for seed in range(20):
            obs, _ = env.reset(seed=seed)
            for _ in range(200):
                obs, _, terminated, truncated, info = env.step(policy(obs))
                if terminated or truncated:
                    break
            count += bool(info["is_success"])
        return count

    assert solved(push_behind_the_box) >= 8
    assert solved(lambda o: rng.uniform(-1, 1, 2).astype(np.float32)) <= 1
