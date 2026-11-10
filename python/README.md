# fx2d

Deterministic 2D physics for reinforcement learning and control. `fx2d` is the Python package of
[Fx2D](https://github.com/Bharath2/fx2d-physics-engine), a C++20 rigid-body engine: Gymnasium
environments, vector environments that step every copy's physics in one parallel call, scenes
built in a few lines or loaded from YAML, and results that are the same bits on every platform.

```bash
pip install fx2d            # the engine; NumPy is its only dependency
pip install "fx2d[gym]"     # with the Gymnasium environments
```

## Train on an environment

```python
import gymnasium as gym
import fx2d.envs  # registers the environments

envs = gym.make_vec("fx2d/Push-v0", num_envs=64)
obs, info = envs.reset(seed=0)
for _ in range(1000):
    obs, reward, terminated, truncated, info = envs.step(envs.action_space.sample())
```

| id | task | actions |
|---|---|---|
| `fx2d/Push-v0` | push a box onto a goal with a disc, seen from above, on a floor with drag | force on the disc, 2-D in [-1, 1] |
| `fx2d/CartPole-v0` | keep a pole hinged on a sliding cart upright | push left or right |
| `fx2d/Pendulum-v0` | swing a rod up with a motor too weak to lift it, and hold it upright | torque in [-2, 2] N m |

`gym.make` builds one environment. `gym.make_vec` builds an `fx2d.envs.Fx2DVectorEnv`, which
applies every copy's action, steps all of their scenes on all cores with the GIL released, then
reads every copy's result; each copy gives exactly what it gives run on its own. For `Push-v0` on
an 8-core laptop that is 49,800 steps per second at 256 copies, against 18,600 through
Gymnasium's `SyncVectorEnv`.

## Build a scene

```python
import fx2d

scene = fx2d.Scene(size=(20, 10), gravity=(0, -10))
scene.add_box("ground", position=(10, 0.5), size=(20, 1), static=True)
scene.add_box("crate", position=(10, 5), size=(1, 1), angle=0.3)

scene.step(1 / 60, steps=120)          # two seconds
print(scene["crate"].pose)             # (x, y, angle)
print(scene.poses())                   # every body, as an (n, 3) NumPy array
```

Every shape the engine has is a `fx2d.Shape`: boxes and convex polygons, rounded or not,
circles, capsules, and edges and chains for terrain. `scene.add_body(name, shape, ...)` adds one,
static, kinematic or free. Bodies and joints are held by name, and the scene hands out shared
handles to the C++ objects: `scene["crate"] is scene["crate"]`, and wherever a body or joint is
asked for, its name and its object both do. Groups, collision filters, ray and overlap queries
and contact events are there too, and scenes written for the C++ engine load unchanged with
`fx2d.Scene.from_yaml(path_or_text)`.

## Joints and motors

```python
scene = fx2d.Scene(size=(10, 10), gravity=(0, 0))
scene.add_box("base", position=(5, 5), size=(0.4, 0.4), static=True)
scene.add_box("arm", position=(6, 5), size=(2, 0.2))
hinge = scene.add_revolute_joint("hinge", "base", "arm", anchor=(0, 0))

hinge.control_mode = fx2d.ControlMode.EFFORT
hinge.set_torque(2.0)
scene.step(0.01, steps=100)
print(hinge.angle, hinge.angular_velocity)
```

`POSITION` and `VELOCITY` modes drive the joint to a target through its PID gains (`hinge.pid`).

## Many scenes at once

`fx2d.step_many(scenes, dt, steps)` steps independent scenes in parallel with the GIL released.
Each scene is stepped by one thread, so every scene ends exactly where stepping it alone would
have left it.

## Determinism

The same scene stepped the same way gives the same bits, run to run and machine to machine. The
engine's test suite checks stored hashes of simulated state on Linux, macOS and Windows, x86-64
and 64-bit ARM, so a changed result in an experiment is a real change, never noise.

## Links

- [Python guide](https://bharath2.github.io/fx2d-physics-engine/guides/python)
- [Engine documentation](https://bharath2.github.io/fx2d-physics-engine/)
- [Source and issues](https://github.com/Bharath2/fx2d-physics-engine)

BSD 3-Clause licensed.
