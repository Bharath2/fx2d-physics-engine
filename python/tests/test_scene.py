import math

import numpy as np
import pytest

import fx2d

SCENE_YAML = """
scene:
    size: [12, 8]
    gravity: [0, -10]
entities:
    ground:
        pose: [6.0, 0.5, 0.0]
        physics: {mass: 0.0, gravity_scale: 0.0, external_forces_enabled: false}
        visual: {geometry: {rectangle: [12.0, 1.0]}}
        collision: {geometry: {rectangle: [12.0, 1.0]}}
    crate:
        pose: [6.0, 4.0, 0.2]
        physics: {mass: 2.0}
        visual: {geometry: {rectangle: [1.0, 1.0]}}
        collision: {geometry: {rectangle: [1.0, 1.0]}}
"""


def crate_scene(x=10.0, angle=0.3):
    scene = fx2d.Scene(size=(20, 10), gravity=(0, -10))
    scene.add_box("ground", position=(10, 0.5), size=(20, 1), static=True)
    scene.add_box("crate", position=(x, 5), size=(1, 1), angle=angle)
    scene.add_circle("ball", position=(x + 2, 3), radius=0.4, elasticity=0.5)
    return scene


def test_bodies_fall_and_come_to_rest():
    scene = crate_scene()
    scene.step(1 / 60, steps=240)
    crate = scene["crate"]
    assert crate.sleeping
    assert crate.position[1] == pytest.approx(1.5, abs=2e-3)
    assert math.remainder(crate.angle, math.pi / 2) == pytest.approx(0.0, abs=1e-3)
    assert scene.time == pytest.approx(4.0)
    touching = {frozenset((c.body1, c.body2)): c for c in scene.contacts()}
    resting = touching[frozenset((scene["ground"], crate))]  # the bodies themselves
    assert len(resting.points) == 2
    assert abs(resting.normal[1]) == pytest.approx(1.0, abs=1e-3)


def test_poses_and_velocities_follow_bodies_order():
    scene = crate_scene()
    scene.step(1 / 60, steps=30)
    poses, velocities = scene.poses(), scene.velocities()
    names = [b.name for b in scene.bodies]
    assert names == ["ground", "crate", "ball"]
    assert poses.shape == (3, 3) and poses.dtype == np.float32
    for row, body in zip(poses, scene.bodies):
        assert tuple(row) == pytest.approx(body.pose)
    for row, body in zip(velocities, scene.bodies):
        assert tuple(row) == pytest.approx(body.velocity)


def test_same_scene_same_bits():
    a, b = crate_scene(), crate_scene()
    a.step(1 / 60, steps=200)
    for _ in range(200):
        b.step(1 / 60)
    assert np.array_equal(a.poses(), b.poses())
    first = a.poses()
    a.reset()
    a.step(1 / 60, steps=200)
    assert np.array_equal(a.poses(), first)


def test_step_many_matches_stepping_alone():
    alone = [crate_scene(x=8 + 0.1 * i, angle=0.05 * i) for i in range(8)]
    together = [crate_scene(x=8 + 0.1 * i, angle=0.05 * i) for i in range(8)]
    for scene in alone:
        scene.step(1 / 60, steps=120)
    fx2d.step_many(together, 1 / 60, steps=120, threads=4)
    for a, b in zip(alone, together):
        assert np.array_equal(a.poses(), b.poses())
        assert np.array_equal(a.velocities(), b.velocities())


def test_applied_force_acts_for_one_step():
    scene = fx2d.Scene(size=(20, 20), gravity=(0, 0))
    body = scene.add_box("body", position=(10, 10), size=(1, 1), mass=2.0)
    body.apply_force((10.0, 0.0))
    scene.step(0.05)
    assert body.velocity[0] == pytest.approx(10.0 / 2.0 * 0.05, rel=1e-3)
    scene.step(0.05)
    assert body.velocity[0] == pytest.approx(0.25, rel=1e-3)


def test_set_state_wakes_and_places_the_body():
    scene = crate_scene()
    scene.step(1 / 60, steps=240)
    crate = scene["crate"]
    assert crate.sleeping
    crate.set_state((4.0, 6.0, 0.0), (1.0, 0.0, 0.0))
    assert not crate.sleeping
    assert crate.pose == pytest.approx((4.0, 6.0, 0.0))
    scene.step(1 / 60)
    assert crate.position[0] > 4.0


def test_revolute_motor_turns_the_arm():
    scene = fx2d.Scene(size=(10, 10), gravity=(0, 0))
    scene.add_box("base", position=(5, 5), size=(0.4, 0.4), static=True)
    scene.add_box("arm", position=(6, 5), size=(2, 0.2))
    hinge = scene.add_revolute_joint("hinge", "base", "arm")
    assert isinstance(scene.joint("hinge"), fx2d.RevoluteJoint)
    hinge.control_mode = fx2d.ControlMode.EFFORT
    hinge.set_torque(1.0)
    scene.step(0.01, steps=50)
    assert hinge.angle > 0.05 and hinge.angular_velocity > 0
    arm = scene["arm"]
    # The hinge holds the arm's end at the base's centre while it turns.
    end = (arm.position[0] - math.cos(arm.angle), arm.position[1] - math.sin(arm.angle))
    assert end == pytest.approx((5.0, 5.0), abs=1e-2)


def test_raycast():
    scene = crate_scene()
    hit = scene.raycast((10, 9), (0, -1), 20)
    assert hit.body is scene["crate"]
    assert hit.distance == pytest.approx(9 - 5.0 - 0.5 / math.cos(0.3), abs=1e-3)  # top face
    assert scene.raycast((1, 9), (0, 1), 5) is None


def test_from_yaml_text():
    scene = fx2d.Scene.from_yaml(SCENE_YAML)
    assert "crate" in scene and scene["crate"].mass == pytest.approx(2.0)
    scene.step(1 / 60, steps=180)
    assert scene["crate"].position[1] < 4.0


def test_errors():
    scene = crate_scene()
    with pytest.raises(ValueError):
        scene.add_box("crate", position=(1, 1), size=(1, 1))
    with pytest.raises(KeyError):
        scene["missing"]
    with pytest.raises(KeyError):
        scene.add_revolute_joint("j", "crate", "missing")
    with pytest.raises(ValueError):
        fx2d.Scene(size=(0, 10))
    with pytest.raises(TypeError):
        fx2d.Scene(size=(12.5, 10))  # the world is whole units
    with pytest.raises(ValueError):
        fx2d.step_many([scene, None], 1 / 60)
    with pytest.raises(ValueError):
        fx2d.step_many([scene, crate_scene(), scene], 1 / 60)
    with pytest.raises(ValueError):
        scene.step(1e-4)
