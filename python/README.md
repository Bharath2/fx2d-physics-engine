# fx2d

Python bindings for [Fx2D](https://github.com/Bharath2/fx2d-physics-engine), a 2D rigid-body
physics engine written in C++20 for simulation and control: joints with position, velocity and
effort motors, contacts and ray casts, scenes built in code or loaded from YAML, and results
that are bit-for-bit the same on every platform and compiler it is tested on.

```bash
pip install fx2d            # the engine; NumPy is its only dependency
pip install "fx2d[gym]"     # with the Gymnasium environments
```

## A scene in a few lines

```python
import fx2d

scene = fx2d.Scene(size=(20, 10), gravity=(0, -10))
scene.add_box("ground", position=(10, 0.5), size=(20, 1), static=True)
scene.add_box("crate", position=(10, 5), size=(1, 1), angle=0.3)

scene.step(1 / 60, steps=120)          # two seconds
print(scene["crate"].pose)             # (x, y, angle)
print(scene.poses())                   # every body, as an (n, 3) NumPy array
```

Scenes written for the C++ engine load unchanged with `fx2d.Scene.from_yaml(path_or_text)`.

## Joints and motors

```python
scene = fx2d.Scene(size=(10, 10))
scene.add_box("base", position=(5, 5), size=(0.4, 0.4), static=True)
scene.add_box("arm", position=(6, 5), size=(2, 0.2))
hinge = scene.add_revolute_joint("hinge", "base", "arm", anchor=(0, 0))

hinge.control_mode = fx2d.ControlMode.EFFORT
hinge.set_torque(2.0)
scene.step(0.01, steps=100)
print(hinge.angle, hinge.angular_velocity)
```

`POSITION` and `VELOCITY` modes drive the joint to a target through its PID gains (`hinge.pid`).

## Gymnasium environments

```python
import gymnasium as gym
import fx2d.envs  # registers the environments

env = gym.make("fx2d/CartPole-v0")
obs, info = env.reset(seed=0)
```

| id | task | actions |
|---|---|---|
| `fx2d/CartPole-v0` | balance a pole on a cart that slides on a rail | push left or right |
| `fx2d/Pendulum-v0` | swing a pendulum up and hold it there | torque in [-2, 2] N m |

## Many scenes at once

`fx2d.step_many(scenes, dt, steps)` steps independent scenes on all cores with the GIL released.
Each scene is stepped by one thread, so every scene ends exactly where stepping it alone would
have left it.

## Determinism

The same scene stepped the same way gives the same bits, run to run and machine to machine. The
engine's test suite checks stored hashes of the simulated state, and they match on Windows and
Linux, on x86-64 and 64-bit ARM, from GCC, Clang and MSVC builds.

## Links

- [Python guide](https://bharath2.github.io/fx2d-physics-engine/guides/python)
- [Engine documentation](https://bharath2.github.io/fx2d-physics-engine/)
- [Source and issues](https://github.com/Bharath2/fx2d-physics-engine)

BSD 3-Clause licensed.
