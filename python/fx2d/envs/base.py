"""What the fx2d environments share, and the vector environment that steps many at once."""

import gymnasium as gym
import numpy as np
from gymnasium.vector import AutoresetMode, VectorEnv
from gymnasium.vector.utils import batch_space, iterate

import fx2d


class Fx2DEnv(gym.Env):
    """An environment built on one ``fx2d.Scene``, stepped ``dt`` seconds per action.

    Subclasses build ``self.scene`` and implement ``_act``, which applies an action to the scene,
    and ``_outcome``, which reads the result once the scene has stepped. Keeping the physics step
    between the two is what lets ``Fx2DVectorEnv`` step every copy's scene in one call.
    """

    metadata = {"render_modes": []}
    dt = 0.02

    def step(self, action):
        self._act(action)
        self.scene.step(self.dt)
        return self._outcome()

    def _act(self, action):
        raise NotImplementedError

    def _outcome(self):
        """``(observation, reward, terminated, truncated, info)`` after the step."""
        raise NotImplementedError


class Fx2DVectorEnv(VectorEnv):
    """``num_envs`` copies of an fx2d environment, their physics stepped in one call.

    Each step applies every copy's action, steps all of their scenes with ``fx2d.step_many``
    (on ``threads`` threads, every core by default, with the GIL released), then reads every
    copy's outcome. A copy that finished on the previous step is reset instead of stepped, as
    in Gymnasium's own vector environments (next-step autoreset), and episodes are cut at
    ``max_episode_steps``. Each copy gives exactly the results it would give run on its own.

    ``gym.make_vec("fx2d/CartPole-v0", num_envs=64)`` builds one of these.
    """

    metadata = {"autoreset_mode": AutoresetMode.NEXT_STEP}

    def __init__(self, env_class, num_envs, max_episode_steps=None, threads=0, **kwargs):
        if num_envs < 1:
            raise ValueError(f"num_envs must be at least 1, got {num_envs}")
        self.envs = [env_class(**kwargs) for _ in range(num_envs)]
        self.num_envs = num_envs
        self.max_episode_steps = max_episode_steps
        self.threads = threads
        first = self.envs[0]
        self.single_observation_space = first.observation_space
        self.single_action_space = first.action_space
        self.observation_space = batch_space(first.observation_space, num_envs)
        self.action_space = batch_space(first.action_space, num_envs)
        self.metadata = {**first.metadata, "autoreset_mode": AutoresetMode.NEXT_STEP}
        self._observations = np.zeros(self.observation_space.shape,
                                      dtype=self.observation_space.dtype)
        self._rewards = np.zeros(num_envs, dtype=np.float64)
        self._terminations = np.zeros(num_envs, dtype=np.bool_)
        self._truncations = np.zeros(num_envs, dtype=np.bool_)
        self._autoreset = np.zeros(num_envs, dtype=np.bool_)
        self._steps = np.zeros(num_envs, dtype=np.int64)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is None:
            seeds = [None] * self.num_envs
        elif isinstance(seed, int):
            seeds = [seed + i for i in range(self.num_envs)]
        else:
            seeds = list(seed)
            if len(seeds) != self.num_envs:
                raise ValueError(f"expected {self.num_envs} seeds, got {len(seeds)}")
        infos = {}
        for i, (env, env_seed) in enumerate(zip(self.envs, seeds)):
            self._observations[i], info = env.reset(seed=env_seed, options=options)
            infos = self._add_info(infos, info, i)
        self._terminations[:] = False
        self._truncations[:] = False
        self._autoreset[:] = False
        self._steps[:] = 0
        return self._observations.copy(), infos

    def step(self, actions):
        infos = {}
        stepping = []
        for i, (env, action) in enumerate(zip(self.envs, iterate(self.action_space, actions))):
            if self._autoreset[i]:
                self._observations[i], info = env.reset()
                self._rewards[i] = 0.0
                self._terminations[i] = False
                self._truncations[i] = False
                self._steps[i] = 0
                infos = self._add_info(infos, info, i)
            else:
                env._act(action)
                stepping.append(i)

        fx2d.step_many([self.envs[i].scene for i in stepping], self.envs[0].dt,
                       threads=self.threads)

        for i in stepping:
            obs, reward, terminated, truncated, info = self.envs[i]._outcome()
            self._steps[i] += 1
            if self.max_episode_steps is not None and self._steps[i] >= self.max_episode_steps:
                truncated = True
            self._observations[i] = obs
            self._rewards[i] = reward
            self._terminations[i] = terminated
            self._truncations[i] = truncated
            infos = self._add_info(infos, info, i)
        self._autoreset = self._terminations | self._truncations
        return (self._observations.copy(), self._rewards.copy(), self._terminations.copy(),
                self._truncations.copy(), infos)
