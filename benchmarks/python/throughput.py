"""Environment steps per second for the fx2d Gymnasium environments.

For each environment: one copy through gym.make, then batches through Gymnasium's
SyncVectorEnv (vectorization_mode="sync") and through fx2d's own vector environment, which
steps every copy's scene in one fx2d.step_many call. Actions are random and drawn before the
timing starts; episodes reset as they end.

Run: python benchmarks/python/throughput.py [seconds per measurement]
"""

import itertools
import os
import platform
import sys
import time

import gymnasium as gym

import fx2d.envs  # noqa: F401  (registers the environments)

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0


def presampled(space, count=1024):
    """Random actions drawn up front, so sampling is not timed with the stepping."""
    space.seed(0)
    return itertools.cycle([space.sample() for _ in range(count)])


def rate(step, batch):
    step()  # warm up
    start = time.perf_counter()
    calls = 0
    while time.perf_counter() - start < SECONDS:
        step()
        calls += 1
    return calls * batch / (time.perf_counter() - start)


def single(env_id):
    env = gym.make(env_id)
    env.reset(seed=0)
    actions = presampled(env.action_space)

    def step():
        _, _, terminated, truncated, _ = env.step(next(actions))
        if terminated or truncated:
            env.reset()

    return rate(step, 1)


def vector(env_id, num_envs, mode):
    env = gym.make_vec(env_id, num_envs=num_envs, vectorization_mode=mode)
    env.reset(seed=0)
    actions = presampled(env.action_space)
    return rate(lambda: env.step(next(actions)), num_envs)


print(f"{platform.processor() or platform.machine()}, {os.cpu_count()} logical cores, "
      f"Python {platform.python_version()}, fx2d {fx2d.__version__}")
print(f"{'environment':18s} {'copies':>6s} {'gym.make':>10s} {'sync':>10s} {'fx2d':>10s}")
for env_id in ("fx2d/CartPole-v0", "fx2d/Pendulum-v0", "fx2d/Push-v0"):
    one = single(env_id)
    for num_envs in (16, 64, 256):
        sync = vector(env_id, num_envs, "sync")
        fast = vector(env_id, num_envs, "vector_entry_point")
        print(f"{env_id:18s} {num_envs:6d} {one:10.0f} {sync:10.0f} {fast:10.0f}")
