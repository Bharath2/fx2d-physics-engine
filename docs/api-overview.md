# Fx2D C++ API {#mainpage}

The reference is organized around how an application uses the engine. Start with the module that owns the problem you are working on; every module contains its related types and functions.

## Browse by module

- @ref fx2d_world "World & entities" — scene stepping, rigid bodies, contacts, groups, and solver constraints.
- @ref fx2d_geometry "Geometry & queries" — shapes, bounding boxes, the broad phase, and ray/overlap results.
- @ref fx2d_math "Math & arrays" — vectors, matrices, fixed-size arrays, and geometric values.
- @ref fx2d_joints "Joints & motors" — revolute/prismatic joints and position, velocity, or effort control.
- @ref fx2d_renderer "Renderer" — the raylib/ImGui visual loop and coordinate transforms.
- @ref fx2d_input "Input" — keyboard, mouse, and headless injection.
- @ref fx2d_yaml "YAML scene building" — loading scenes and constructing entities, shapes, and joints.

## Integration first

The generated reference tells you exact declarations. For practical integration guidance—header selection, linking `Fx2Dlib`, headless use, and renderer setup—return to the [Fx2D API guide](https://bharath2.github.io/fx2d-physics-engine/api/).

## A useful rule of thumb

`FxScene` owns the world. `FxEntity` is a body in it. `FxShape` describes collision geometry. Link your application to `Fx2Dlib`, then include only the public headers your use case needs.
