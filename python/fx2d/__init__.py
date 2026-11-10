"""Fx2D: deterministic 2D rigid-body physics for simulation and control.

Build a scene in code from shapes, bodies, joints and groups, or load one from YAML; the
physics runs in C++. Bodies and joints are held by name, and the same name always gives back
the same object. Gymnasium environments are in ``fx2d.envs``.
"""

from importlib.metadata import PackageNotFoundError, version

from ._fx2d import (
    Body,
    Contact,
    ContactEvent,
    ControlMode,
    Group,
    Joint,
    PrismaticJoint,
    RayHit,
    RevoluteJoint,
    Scene,
    Shape,
    step_many,
)

try:
    __version__ = version("fx2d")
except PackageNotFoundError:
    __version__ = "0+unknown"

__all__ = [
    "Body",
    "Contact",
    "ContactEvent",
    "ControlMode",
    "Group",
    "Joint",
    "PrismaticJoint",
    "RayHit",
    "RevoluteJoint",
    "Scene",
    "Shape",
    "step_many",
]
