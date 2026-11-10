"""Fx2D: deterministic 2D rigid-body physics for simulation and control.

Build a scene in code or load one from YAML, step it, and read poses back as NumPy arrays.
Gymnasium environments are in ``fx2d.envs``.
"""

from importlib.metadata import PackageNotFoundError, version

from ._fx2d import (
    Body,
    Contact,
    ControlMode,
    Joint,
    PrismaticJoint,
    RayHit,
    RevoluteJoint,
    Scene,
    step_many,
)

try:
    __version__ = version("fx2d")
except PackageNotFoundError:
    __version__ = "0+unknown"

__all__ = [
    "Body",
    "Contact",
    "ControlMode",
    "Joint",
    "PrismaticJoint",
    "RayHit",
    "RevoluteJoint",
    "Scene",
    "step_many",
]
