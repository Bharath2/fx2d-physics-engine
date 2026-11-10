# Python

The `fx2d` Python package runs the same engine as the C++ library: the same solver, the same
YAML scenes, the same bits. It adds NumPy views of a scene's state, a call that steps many scenes
on all cores, and Gymnasium environments with vector versions built on that call.

## Install

```bash
pip install fx2d            # the engine; NumPy is its only dependency
pip install "fx2d[gym]"     # with the Gymnasium environments
```

Wheels are built for CPython 3.10 and newer on Linux (x86-64 and ARM64), macOS and Windows. To
build from a clone instead, with a C++20 compiler and CMake 3.16 or newer, run `pip install .`
in the repository; CMake downloads the YAML loader and builds it into the module.

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

## Building scenes

A scene is built the way the C++ engine builds one, and the physics runs in C++: Python only
describes the scene and reads it back. Every shape the engine has is a `fx2d.Shape`, given in
the body's own frame:

```python
scene = fx2d.Scene(size=(20, 10), gravity=(0, -10))
scene.add_body("terrain", fx2d.Shape.chain([(1, 7), (8, 2), (19, 2)]), static=True)
scene.add_body("wall", fx2d.Shape.edge((19, 2), (19, 8)), static=True)
scene.add_body("wedge", fx2d.Shape.polygon([(-0.5, -0.3), (0.5, -0.3), (0, 0.4)]),
               position=(4, 7))
scene.add_body("log", fx2d.Shape.capsule(1.2, 0.2), position=(6, 6), angle=0.4)
scene.add_box("crate", position=(10, 4), size=(0.8, 0.8), rounding=0.05)
scene.add_circle("ball", position=(12, 5), radius=0.3)
```

| shape | what it is |
|---|---|
| `Shape.box(size, rounding=0)` | a width x height box, its corners optionally rounded |
| `Shape.circle(radius)` | a disc |
| `Shape.polygon(vertices, rounding=0)` | a convex polygon, re-centred on the mean of its vertices |
| `Shape.capsule(length, radius)` | a segment along the body's x axis, rounded by `radius` |
| `Shape.edge(start, end)` | a segment with no thickness, for static bodies |
| `Shape.chain(points)` | an open polyline for static terrain, solid on the left of the direction its points run |

`add_body(name, shape, position, angle, mass, ...)` adds a body of any shape; `add_box` and
`add_circle` are shortcuts for the two commonest. Each also takes `static`, `kinematic`,
`elasticity`, `static_friction` and `dynamic_friction`. Materials left out keep the engine's
defaults: elasticity 0.1 and no friction. A static body never moves. A kinematic body moves only
by the velocity it is given, a moving platform or a paddle; neither is pushed by contacts,
forces or gravity. Edges and chains have no area, so they are static.

A body's `pose`, `velocity`, `mass`, materials, `enabled`, `is_sensor` and `sleeping` are
attributes. `damping` adds drag to speed and spin alike, as a floor does to bodies seen from
above: about e^-1 of each is left after `1 / damping` seconds. `set_state(pose, velocity)`
places a body as if it had started there: awake, with no forces pending. `to_world`,
`to_local` and `velocity_at` convert points between frames, `shape` is a copy of the body's
geometry, and `world_vertices()` gives its vertices where the body is now, to draw it.

`apply_force(force, point=None)` and `apply_torque(torque)` act for the whole of the next step
and then lapse, so continuous control applies them before every step. `apply_impulse` changes
the velocity once.

## Names and handles

Everything in a scene has a name, and the scene hands out shared handles to the C++ objects,
much as `get_entity` hands out a `std::shared_ptr` in C++:

```python
crate = scene["crate"]                  # or scene.body("crate")
assert scene["crate"] is crate          # one name, one object, every time
hinge = scene.add_revolute_joint("hinge", "wall", crate)   # names and handles both work
assert hinge.body2 is crate and scene.joint("hinge") is hinge
```

Wherever a body or a joint is asked for, its name and its object both do. Contacts, contact
events, ray hits and queries return the `Body` objects themselves, so `contact.body1.name` and
`contact.body1 is crate` both work. A handle keeps its body alive: after `scene.remove("crate")`,
or once the scene itself is gone, `crate` can still be read, though nothing moves it.

`"crate" in scene`, `len(scene)` and `scene.bodies` look the scene up. `scene.remove(body)` or
`del scene["crate"]` takes a body out along with the joints that hold it, and
`scene.remove_joint(joint)` takes out one joint.

## Groups and collision filtering

```python
for name, x in (("upper", 14), ("lower", 15), ("hand", 16), ("gripper", 17)):
    scene.add_box(name, position=(x, 8), size=(0.9, 0.2))
arm = scene.add_group("arm", ["upper", "lower", "hand"])   # members never collide with each other
scene.add_to_group("arm", "gripper")
arm.set_enabled(False)                                       # the whole arm leaves the simulation
scene.remove_group("arm")                                    # and its bodies with it
```

`add_group(name, bodies, self_collide=False)` keeps its members from colliding with one another
unless `self_collide` is set, which suits the links of one robot. `disable_collision(a, b)`
exempts a single pair, and bodies sharing a negative `collision_group` never collide with each
other. Bodies joined by a joint never collide with each other either.

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

`add_distance_joint(name, body1, body2, anchor1, anchor2, min_length, max_length, compliance)`
links an anchor on each body, in each body's frame, and keeps them between the two lengths;
lengths left out take the distance the anchors are apart now, so equal limits make a rigid rod,
and `compliance` stretches it by that many metres per newton of load. `add_rope_joint` is the
same with no minimum: it stops the anchors parting further than its length and goes slack when
they come closer. Rotation stays free at both ends, so a chain of them hangs like a rope. Their
motors drive the separation, through `set_length`, `set_rate` or `set_force`.

## State in bulk

`scene.poses()` and `scene.velocities()` return `(n, 3)` NumPy arrays of every body's
`(x, y, angle)` and `(vx, vy, angular)`, in the order of `scene.bodies`. They are copies, safe
to keep after the next step.

## Contacts, queries and events

`scene.contacts()` lists the pairs touching after the last step, each with its two bodies, the
normal (pointing from `body1` to `body2`), the touching points, the depth, the normal impulse and
whether a sensor was involved. `contacts_began()` and `contacts_ended()` list the pairs that
started or stopped touching during the last step, which is how a sensor reports what entered it.

| query | returns |
|---|---|
| `raycast(origin, direction, max_distance)` | the nearest hit, with its body, point, normal and distance, or `None` |
| `raycast_all(origin, direction, max_distance)` | every hit along the ray, nearest first |
| `overlap(shape, position, angle=0)` | the bodies a shape placed there would overlap |
| `overlap_circle(centre, radius)`, `overlap_box(centre, size)` | the bodies overlapping a circle or an upright box |
| `body_at(point)` | the first body covering a point, or `None` |

## YAML scenes

`fx2d.Scene.from_yaml(path_or_text)` loads a scene file written for the C++ engine, or YAML text,
and the bodies, joints and groups it defines are then reached by name like any other. A scene can
be loaded from YAML and then extended from Python. See [Scene authoring](/guides/scene-authoring)
for the format.

`reset()` puts a scene back as it was when it first stepped. To keep bodies added after that,
call `capture_initial_state()` once they are in.

## Many scenes at once

```python
scenes = [make_scene(seed) for seed in range(64)]
fx2d.step_many(scenes, 1 / 60, steps=600)           # every core, the GIL released
```

Each scene is stepped from start to finish by one thread, so every scene ends exactly where
stepping it alone would have left it, whatever the thread count. `threads=0`, the default, uses a
thread per four scenes up to every core: waking a thread costs about as much as stepping a small
scene, so a few scenes go faster on fewer threads. The threads are kept between calls. A scene listed twice is refused, since two threads would then step it at once.

## Determinism

The same scene, stepped the same way, gives the same bits on every run and machine. The engine's
test suite checks stored hashes of simulated state, and CI finds them identical on Linux, macOS
and Windows, x86-64 and 64-bit ARM, and GCC, Clang, Apple Clang and MSVC builds. `scene.reset()` puts a scene back as it was when it
first stepped, so a reset scene replays the same trajectory.

## Gymnasium environments

```python
import gymnasium as gym
import fx2d.envs  # registers the environments

env = gym.make("fx2d/Push-v0")
obs, info = env.reset(seed=0)
obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
```

| id | task | observation | action |
|---|---|---|---|
| `fx2d/Push-v0` | push a box onto a goal with a disc in a walled 4 m arena, seen from above, 200 steps; the floor drags both and their contact has friction, so a push off the box's centre turns it | disc position and velocity, box position, angle (cos, sin), velocity and spin, goal position | force on the disc, 2-D in [-1, 1], up to 10 N |
| `fx2d/CartPole-v0` | keep a pole hinged on a sliding cart upright, 500 steps | cart position and velocity, pole angle and rate | push left (0) or right (1), 10 N |
| `fx2d/Pendulum-v0` | swing a rod up from any angle and hold it upright, 200 steps | cos and sin of the angle from upright, angular rate | torque in [-2, 2] N m |

`Push-v0` rewards minus the box's distance to the goal and a tenth of the disc's distance to the
box, and ends with a bonus of 10, and `info["is_success"]`, once the box is within 0.15 m of the
goal. All three are seeded through `reset(seed=...)`, and each keeps its scene as
`env.unwrapped.scene`.

## Vector environments

```python
envs = gym.make_vec("fx2d/Push-v0", num_envs=256)
obs, info = envs.reset(seed=0)          # copy i is seeded with 0 + i
obs, rewards, terminated, truncated, info = envs.step(envs.action_space.sample())
```

`gym.make_vec` builds an `Fx2DVectorEnv`. Each step applies every copy's action, steps all of
their scenes in one `step_many` call, then reads each copy's result. A copy that finished on the
previous step is reset instead, as in Gymnasium's own vector environments (next-step autoreset),
and the results match Gymnasium's `SyncVectorEnv` bit for bit. Pass `threads=` to `make_vec` to
fix the thread count.

Measured with [benchmarks/python/throughput.py](https://github.com/Bharath2/fx2d-physics-engine/blob/main/benchmarks/python/throughput.py)
on an 8-core Intel i7-11800H laptop under Linux, in environment steps per second:

| `fx2d/Push-v0` | 16 copies | 64 copies | 256 copies |
|---|---|---|---|
| Gymnasium `SyncVectorEnv` | 17,900 | 18,300 | 18,600 |
| `Fx2DVectorEnv` | 25,200 | 32,700 | 49,800 |

The physics alone runs about 4-6x faster across 256 scenes than stepping them one by one; the
rest of each step is Python reading observations and computing rewards. CartPole and Pendulum
are so cheap to simulate that Python dominates, and they gain only from batches of hundreds.

## Writing your own environment

Subclass `fx2d.envs.Fx2DEnv`: build `self.scene` in `__init__`, set `self.dt`, and split a step
into `_act(action)`, which applies the action to the scene, and `_outcome()`, which returns
`(observation, reward, terminated, truncated, info)` once the scene has stepped. The base class
steps the scene between the two, and `Fx2DVectorEnv(MyEnv, num_envs=64)` batches it with no
further code.

```python
import numpy as np
from gymnasium import spaces

import fx2d
from fx2d.envs import Fx2DEnv


class Hover(Fx2DEnv):
    """Hold a falling box at a height of 3 m by thrusting up."""

    def __init__(self):
        self.dt = 0.05
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(2,), dtype=np.float32)
        self.action_space = spaces.Box(0.0, 1.0, shape=(1,), dtype=np.float32)
        self.scene = fx2d.Scene(size=(4, 6), gravity=(0, -9.81))
        self.body = self.scene.add_box("body", position=(2, 3), size=(0.5, 0.5))

    def _observation(self):
        return np.array([self.body.position[1] - 3.0, self.body.velocity[1]], dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.scene.reset()
        self.body.set_state((2.0, 3.0 + self.np_random.uniform(-1, 1), 0.0))
        return self._observation(), {}

    def _act(self, action):
        self.body.apply_force((0.0, 20.0 * float(action[0])))  # up to twice its weight

    def _outcome(self):
        obs = self._observation()
        return obs, -abs(float(obs[0])), False, False, {}
```

Register it with `gymnasium.register(id=..., entry_point=Hover,
vector_entry_point=functools.partial(fx2d.envs.Fx2DVectorEnv, Hover), max_episode_steps=200)`
to get `gym.make` and `gym.make_vec` for it.
