# Python

The `fx2d` Python package runs the same engine as the C++ library: the same solver, the same
YAML scenes, the same bits. It adds NumPy views of a scene's state, a call that steps many scenes
on all cores, and Gymnasium environments.

## Install

From a clone of the repository, with a C++20 compiler and CMake 3.16 or newer:

```bash
pip install .            # the engine; NumPy is its only dependency
pip install ".[gym]"     # with the Gymnasium environments
```

CMake downloads and builds the YAML loader into the module, so nothing else needs installing.

## A first scene

```python
import fx2d

scene = fx2d.Scene(size=(20, 10), gravity=(0, -10))
scene.add_box("ground", position=(10, 0.5), size=(20, 1), static=True)
scene.add_box("crate", position=(10, 5), size=(1, 1), angle=0.3)
scene.add_circle("ball", position=(12, 3), radius=0.4, elasticity=0.5)

scene.step(1 / 60, steps=240)          # four seconds
print(scene["crate"].pose)             # (x, y, angle)
print(scene["crate"].sleeping)         # True: it has come to rest
```

The world spans `[0, width] x [0, height]`, and bodies that reach its edges bounce back in.
`step(dt)` takes steps of at least 0.001 s; a step longer than 0.06 s is shortened to 0.06 s, so
take more steps to cover a longer span.

## Bodies

`add_box` and `add_circle` take a name, a position and a size, then optional `angle`, `mass`
(1 kg), `static`, `elasticity`, `static_friction` and `dynamic_friction`. Materials left out keep
the engine's defaults: elasticity 0.1 and no friction. A static body never moves and is not
pushed by anything.

A body's `pose`, `velocity`, `mass`, material values and `sleeping` flag are attributes.
`set_state(pose, velocity)` places it as if it had started there: awake, with no forces pending.

`apply_force(force, point=None)` and `apply_torque(torque)` act for the whole of the next step
and then lapse, so continuous control applies them before every step. `apply_impulse` changes
the velocity once.

## Joints and motors

```python
scene = fx2d.Scene(size=(10, 10), gravity=(0, 0))
scene.add_box("base", position=(5, 5), size=(0.4, 0.4), static=True)
scene.add_box("arm", position=(6, 5), size=(2, 0.2))
hinge = scene.add_revolute_joint("hinge", "base", "arm", anchor=(0, 0))

hinge.control_mode = fx2d.ControlMode.EFFORT
hinge.set_torque(1.0)
scene.step(0.01, steps=100)
print(hinge.angle, hinge.angular_velocity)
```

The anchor of a revolute joint, and the axis of a prismatic one, are given in the first body's
frame. Every joint starts as a position motor holding its starting angle or offset with a gain
of 1, as in C++; set `control_mode` to `EFFORT` for a free joint. `POSITION` and `VELOCITY` drive
the joint to `set_angle`/`set_position` or `set_angular_velocity`/`set_velocity` through its
`pid` gains, limited by `max_effort`. See [Joints and motors](/guides/joints) for how each mode
behaves.

## State in bulk

`scene.poses()` and `scene.velocities()` return `(n, 3)` NumPy arrays of every body's
`(x, y, angle)` and `(vx, vy, angular)`, in the order of `scene.bodies`. They are copies, safe
to keep after the next step.

## Contacts and ray casts

`scene.contacts()` lists the pairs touching after the last step, each with both body names, the
normal (pointing from `body1` to `body2`), the touching points, the depth and the normal impulse.
`scene.raycast(origin, direction, max_distance)` returns the nearest hit, with the body's name,
point, normal and distance, or `None`.

## YAML scenes

`fx2d.Scene.from_yaml(path_or_text)` loads a scene file written for the C++ engine, or YAML text.
Shapes the Python API does not build yet, such as polygons, capsules and chains, come in this
way. See [Scene authoring](/guides/scene-authoring) for the format.

## Many scenes at once

```python
scenes = [make_scene(seed) for seed in range(64)]
fx2d.step_many(scenes, 1 / 60, steps=600)           # every core, the GIL released
```

Each scene is stepped from start to finish by one thread, so every scene ends exactly where
stepping it alone would have left it, whatever the thread count. `threads=0`, the default, uses
every core. A scene listed twice is refused, since two threads would then step it at once.

## Determinism

The same scene, stepped the same way, gives the same bits on every run and machine. The engine's
test suite checks stored hashes of simulated state; they match on Windows and Linux, x86-64 and
64-bit ARM, and GCC, Clang and MSVC builds. `scene.reset()` puts a scene back as it was when it
first stepped, so a reset scene replays the same trajectory.

## Gymnasium environments

```python
import gymnasium as gym
import fx2d.envs  # registers the environments

env = gym.make("fx2d/CartPole-v0")
obs, info = env.reset(seed=0)
obs, reward, terminated, truncated, info = env.step(1)
```

| id | task | observation | action |
|---|---|---|---|
| `fx2d/CartPole-v0` | keep a pole hinged on a sliding cart upright, 500 steps | cart position and velocity, pole angle and rate | push left (0) or right (1), 10 N |
| `fx2d/Pendulum-v0` | swing a rod up from any angle and hold it upright, 200 steps | cos and sin of the angle from upright, angular rate | torque in [-2, 2] N m |

Both are built from an ordinary `fx2d.Scene`, which each environment keeps as `env.unwrapped.scene`,
and both are seeded through `reset(seed=...)`.
