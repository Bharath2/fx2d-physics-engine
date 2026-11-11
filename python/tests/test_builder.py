"""Building scenes from Python: shapes, names and handles, groups, removal, queries, events."""

import gc
import math

import numpy as np
import pytest

import fx2d


def floor_scene():
    scene = fx2d.Scene(size=(20, 10), gravity=(0, -10))
    scene.add_box("ground", position=(10, 0.5), size=(20, 1), static=True)
    return scene


def test_every_shape_comes_to_rest_on_the_ground():
    scene = floor_scene()
    triangle = [(-0.5, -0.4), (0.5, -0.4), (0.0, 0.5)]
    bodies = [
        scene.add_body("triangle", fx2d.Shape.polygon(triangle), position=(3, 3)),
        scene.add_body("pill", fx2d.Shape.capsule(1.0, 0.25), position=(6, 3)),
        scene.add_box("rounded", position=(9, 3), size=(0.8, 0.8), rounding=0.1),
        scene.add_circle("ball", position=(12, 3), radius=0.3),
    ]
    scene.step(1 / 60, steps=300)
    for body in bodies:
        assert body.sleeping, body.name
        assert 1.0 < body.position[1] < 2.0, body.name
    assert [b.shape.kind for b in bodies] == ["polygon", "capsule", "polygon", "circle"]
    assert scene["pill"].shape.rounding == pytest.approx(0.25)


def test_chains_and_edges_are_static_terrain():
    scene = fx2d.Scene(size=(20, 10), gravity=(0, -10))
    scene.add_body("slope", fx2d.Shape.chain([(1, 6), (8, 2), (19, 2)]), static=True)
    scene.add_body("wall", fx2d.Shape.edge((19, 2), (19, 8)), static=True)
    ball = scene.add_circle("ball", position=(2, 7), radius=0.3)
    lowest, furthest = 10.0, 0.0
    for _ in range(240):  # slides down the slope, along the flat and into the wall
        scene.step(1 / 60)
        lowest, furthest = min(lowest, ball.position[1]), max(furthest, ball.position[0])
    assert lowest > 2.25  # never through the chain
    assert 18.0 < furthest < 18.75  # reached the wall and was stopped by it
    with pytest.raises(ValueError):
        scene.add_body("loose", fx2d.Shape.edge((0, 0), (1, 0)))  # an edge has no area


def test_polygon_vertices_are_recentred_and_follow_the_body():
    shape = fx2d.Shape.polygon([(0, 0), (2, 0), (2, 1), (0, 1)])
    assert np.allclose(shape.vertices.mean(axis=0), 0.0)
    scene = fx2d.Scene(size=(10, 10), gravity=(0, 0))
    body = scene.add_body("plank", shape, position=(5, 5), angle=math.pi / 2)
    corners = body.world_vertices()
    assert corners.shape == (4, 2)
    assert np.allclose(sorted(corners[:, 0]), [4.5, 4.5, 5.5, 5.5], atol=1e-5)
    assert body.to_world((1.0, 0.0)) == pytest.approx((5.0, 6.0), abs=1e-5)
    assert body.to_local((5.0, 6.0)) == pytest.approx((1.0, 0.0), abs=1e-5)


def test_names_and_handles_are_interchangeable():
    scene = floor_scene()
    arm = scene.add_box("arm", position=(6, 5), size=(2, 0.2))
    scene.add_box("base", position=(5, 5), size=(0.4, 0.4), static=True)
    by_name = scene.add_revolute_joint("by_name", "base", "arm")
    by_handle = scene.add_prismatic_joint("by_handle", scene["base"], arm)
    assert by_name.body2 is arm and by_handle.body2 is arm
    assert scene.joint("by_name") is by_name and scene.joint(by_name) is by_name
    other = fx2d.Scene()
    stranger = other.add_box("arm", position=(1, 1), size=(1, 1))
    with pytest.raises(ValueError):
        scene.add_revolute_joint("wrong", "base", stranger)  # same name, another scene


def test_removed_bodies_stay_readable_handles():
    scene = floor_scene()
    crate = scene.add_box("crate", position=(10, 3), size=(1, 1))
    scene.step(1 / 60, steps=10)
    scene.remove(crate)
    assert "crate" not in scene and len(scene) == 1
    scene.step(1 / 60, steps=10)  # stepping without it is fine
    assert crate.name == "crate" and len(crate.pose) == 3
    del scene
    gc.collect()
    assert crate.position[1] > 1.0  # the handle outlives its scene too


def test_groups_do_not_collide_among_themselves_and_go_together():
    scene = fx2d.Scene(size=(10, 10), gravity=(0, -10))
    scene.add_box("ground", position=(5, 0.5), size=(10, 1), static=True)
    low = scene.add_box("low", position=(5, 1.5), size=(1, 1))
    high = scene.add_box("high", position=(5, 2.5), size=(1, 1))
    group = scene.add_group("ghosts", [low, "high"])
    assert len(group) == 2 and group.bodies[1] is high
    scene.step(1 / 60, steps=60)
    assert high.position[1] < 2.0  # fell through its group mate
    scene.remove_group("ghosts")
    assert "low" not in scene and "high" not in scene


def test_kinematic_bodies_push_and_are_not_pushed():
    scene = fx2d.Scene(size=(20, 10), gravity=(0, -10))
    scene.add_box("ground", position=(10, 0.5), size=(20, 1), static=True)
    crate = scene.add_box("crate", position=(8, 1.5), size=(1, 1))
    paddle = scene.add_box("paddle", position=(5, 1.6), size=(0.4, 1.0), kinematic=True)
    paddle.velocity = (2.0, 0.0, 0.0)
    scene.step(1 / 60, steps=120)
    assert paddle.position == pytest.approx((9.0, 1.6), abs=1e-3)  # unmoved by gravity or crate
    assert crate.position[0] > 9.5  # shoved along


def test_queries_return_bodies():
    scene = floor_scene()
    a = scene.add_box("a", position=(5, 3), size=(1, 1))
    b = scene.add_box("b", position=(5, 5), size=(1, 1))
    hits = scene.raycast_all((5, 9), (0, -1), 20)
    assert [h.body for h in hits] == [b, a, scene["ground"]]
    assert scene.body_at((5, 3)) is a and scene.body_at((1, 8)) is None
    assert set(scene.overlap_circle((5, 4), 0.6)) == {a, b}
    assert scene.overlap_box((5, 5), (0.5, 0.5)) == [b]
    assert scene.overlap(fx2d.Shape.circle(0.2), (5, 3)) == [a]


def test_contact_events_name_their_bodies():
    scene = floor_scene()
    ball = scene.add_circle("ball", position=(10, 2), radius=0.3, elasticity=0.9)
    began, ended = [], []
    for _ in range(240):
        scene.step(1 / 60)
        began += [{e.body1, e.body2} for e in scene.contacts_began()]
        ended += [{e.body1, e.body2} for e in scene.contacts_ended()]
    pair = {scene["ground"], ball}
    assert began.count(pair) >= 2 and ended.count(pair) >= 1  # it bounced


def test_capture_initial_state_keeps_late_bodies_through_reset():
    scene = floor_scene()
    scene.step(1 / 60)
    scene.add_box("late", position=(10, 4), size=(1, 1))
    scene.capture_initial_state()
    scene.step(1 / 60, steps=60)
    scene.reset()
    assert "late" in scene and scene["late"].position == pytest.approx((10.0, 4.0))


def test_distance_and_rope_joints():
    scene = fx2d.Scene(size=(10, 10), gravity=(0, -10))
    scene.add_box("ceiling", position=(5, 9), size=(4, 0.2), static=True)
    rod_bob = scene.add_circle("rod_bob", position=(4, 7), radius=0.1)
    rope_bob = scene.add_circle("rope_bob", position=(6.2, 8.5), radius=0.1)
    rod = scene.add_distance_joint("rod", "ceiling", rod_bob, anchor1=(-1, 0))
    rope = scene.add_rope_joint("rope", "ceiling", rope_bob, anchor1=(1, 0), length=2.0)
    assert isinstance(scene.joint("rope"), fx2d.DistanceJoint)
    assert rod.min_length == pytest.approx(2.0, abs=1e-4) == rod.max_length  # as authored
    assert rope.min_length == 0.0 and rope.max_length == pytest.approx(2.0)
    scene.step(1 / 60, steps=180)
    assert rod.length == pytest.approx(2.0, abs=2e-3)  # a rigid link holds its length
    assert rope.length == pytest.approx(2.0, abs=2e-3)  # the rope went taut
    assert rope_bob.position[1] < 9.0 - 1.9  # swinging low on its full length, not resting slack
