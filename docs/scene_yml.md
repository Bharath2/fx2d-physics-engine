# Scene YAML Reference

Fx2D scenes can be described entirely in YAML and loaded at runtime using `FxYAML::buildScene()`. Include via:

```cpp
#include "Fx2D/Core.h"
```

```cpp
// Load from file
auto scene = FxYAML::buildScene("./Scene.yml");

// Or load from an inline YAML string
auto scene = FxYAML::buildScene(R"(
    scene:
        size: [12, 8]
        gravity: [0, -10]
    entities:
        ball:
            pose: [6, 4, 0]
            physics:
                mass: 1.0
            visual:
                geometry:
                    circle: 0.5
                texture: [255, 100, 0, 255]
            collision:
                geometry:
                    circle: 0.5
)");
```

---

## Top-level Structure

A valid YAML file must have a `scene` block and may also have `entities` and `joints` blocks:

```yaml
scene:
    size: [12, 8]
    gravity: [0, -10]
    background: [255, 255, 255, 255]

entities:
    my_entity:
        pose: [6, 4, 0]
        ...

joints:
    my_joint:
        type: revolute
        parent: body
        child: wheel
        ...
```

---

## `scene` Block

| Key | Type | Required | Description |
|---|---|---|---|
| `size` | `[width, height]` | **Yes** | World dimensions in physics units |
| `gravity` | `[gx, gy]` | No | Gravity acceleration vector (default: `[0, 0]`) |
| `background` | `[R, G, B, A]` or path string | No | Background fill colour (0–255) or image file path |

```yaml
scene:
    size: [12, 8]
    gravity: [0, -10]
    background: [230, 230, 230, 255]     # solid light-grey
    # background: ./assets/sky.png       # or an image file
```

---

## `entities` Block

Each key under `entities` becomes the entity's unique name. The parser iterates keys in order, so declaration order is preserved.

```yaml
entities:
    ground:     # entity name
        pose: ...
        physics: ...
        visual: ...
        collision: ...
    ball:
        ...
```

---

### `pose`

Initial world-space pose: `[x, y, theta]`.

- `x`, `y` — position in physics units
- `theta` — orientation in **degrees**

```yaml
pose: [6.0, 4.0, 45.0]     # x=6, y=4, rotated 45°
```

---

### `init_velocity` *(optional)*

Initial velocity: `[vx, vy, omega]`.

- `vx`, `vy` — linear velocity (units/s)
- `omega` — angular velocity (degrees/s)

```yaml
init_velocity: [-5.0, 0.0, 0.0]    # moving left at 5 units/s
```

---

### `physics` Block

Controls how the entity responds to forces, gravity, and collisions.

| Key | Type | Default | Description |
|---|---|---|---|
| `mass` | float | `1.0` | Mass in kg. Set to `0` for a static (immovable) body |
| `inertia` | float | computed | Rotational inertia. If omitted, calculated from the visual geometry |
| `gravity_scale` | float | `1.0` | Multiplier on scene gravity. `0.0` disables gravity for this entity |
| `vel_damping` | float | `0.0` | Linear/angular velocity damping applied each step |
| `elasticity` | float | `1.0` | Coefficient of restitution (0 = inelastic, 1 = perfectly elastic) |
| `static_friction` | float | `0.0` | Static friction coefficient used in contact force calculation |
| `dynamic_friction` | float | `0.0` | Kinetic friction coefficient used in contact force calculation |
| `external_forces_enabled` | bool | `true` | If `false`, the entity ignores external forces and collisions (gravity still applies) |

```yaml
physics:
    mass: 5.0
    gravity_scale: 1.0
    vel_damping: 0.01
    elasticity: 0.7
    static_friction: 0.4
    dynamic_friction: 0.3
    external_forces_enabled: true
```

> **Static bodies:** set `mass: 0` (which sets `inv_mass = 0`) and `gravity_scale: 0.0`. The body will participate in collision detection but receive no correction from the solver.

> **Inertia:** if `inertia` is omitted, it is automatically computed from the visual geometry and mass. Specify it explicitly only when you need a custom value.

---

### `visual` Block

Controls how the entity is drawn. The shape is defined by `geometry`.

| Key | Type | Default | Description |
|---|---|---|---|
| `pose` | `[x, y, theta]` | `[0, 0, 0]` | Offset from body frame origin (local space) |
| `geometry` | map | — | Shape definition (see below) |
| `texture` | `[R, G, B, A]` or path | — | Fill colour or image file path |
| `border_color` | `[R, G, B, A]` | — | Outline colour (only active when `border_thickness > 0`) |
| `border_thickness` | float | — | Outline thickness in screen pixels |

```yaml
visual:
    pose: [0, 0, 0]
    geometry:
        rectangle: [1.0, 0.5]
    texture: ./assets/crate.png
    border_color: [0, 0, 0, 255]
    border_thickness: 2.0
```

---

### `collision` Block

Defines the shape used for physics collision detection. May differ from the visual shape.

| Key | Type | Default | Description |
|---|---|---|---|
| `pose` | `[x, y, theta]` | `[0, 0, 0]` | Offset from body frame origin (local space) |
| `geometry` | map | — | Shape definition (see below) |

```yaml
collision:
    pose: [0, 0, 0]
    geometry:
        rectangle: [1.0, 0.5]
```

> Having separate `visual` and `collision` shapes is useful when you want a simplified hitbox (e.g. a circle collision shape for a roughly round sprite) or an invisible static collider larger than the visual (e.g. a wider ground plane).

---

## Geometry Types

Used inside both `visual` and `collision` blocks.

### Circle

```yaml
geometry:
    circle: 0.5     # radius
```

### Rectangle

```yaml
geometry:
    rectangle: [1.0, 0.5]   # [width, height]
```

The rectangle is axis-aligned in local space and centred at the shape's pose offset.

### Polygon

```yaml
geometry:
    polygon:
        - [0.0, 0.0]
        - [2.0, 0.0]
        - [2.0, 1.0]
        - [0.0, 1.0]
```

Each entry is a 2D vertex `[x, y]` in local space. Vertices are specified in order (clockwise or counter-clockwise). For collision, convex polygons give the most reliable SAT results.

---

## Complete Entity Example

```yaml
entities:
    ball:
        pose: [10.0, 6.0, 0.0]
        init_velocity: [-6.0, 0.0, 0.0]
        physics:
            mass: 5.0
            gravity_scale: 1.0
            vel_damping: 0.0
            elasticity: 0.8
            static_friction: 0.3
            dynamic_friction: 0.2
            external_forces_enabled: true
        visual:
            pose: [0, 0, 0]
            geometry:
                circle: 0.5
            texture: ./assets/ball.png
            border_color: [0, 0, 0, 255]
            border_thickness: 5.0
        collision:
            pose: [0, 0, 0]
            geometry:
                circle: 0.5

    ground:
        pose: [6.0, 0.5, 0.0]
        physics:
            gravity_scale: 0.0
            external_forces_enabled: false
            elasticity: 0.6
            static_friction: 0.3
            dynamic_friction: 0.2
        visual:
            geometry:
                rectangle: [12.0, 1.0]
            texture: [50, 50, 50, 255]
            border_thickness: 3.0
        collision:
            geometry:
                rectangle: [12.0, 1.0]
```

---

## `joints` Block

Each key under `joints` becomes the joint's unique name.

| Key | Type | Required | Description |
|---|---|---|---|
| `type` | string | **Yes** | Joint type: `revolute` or `prismatic` |
| `parent` | string | **Yes** | Name of the parent entity |
| `child` | string | **Yes** | Name of the child entity |
| `pid` | `[p, i, d]` | No | PID gains, defaults to `[1, 0, 0]` |
| `control_mode` | string | No | `position`, `velocity`, or `effort` |
| `target` | float | No | Target value interpreted by the chosen control mode |
| `max_effort` | float | No | Shared effort limit for motor output |
| `entities_collide` | bool | No | Parsed from YAML, but joint-linked bodies are currently kept non-colliding in the scene |

### Revolute Joint

Additional revolute fields:

| Key | Type | Default | Description |
|---|---|---|---|
| `anchor` | `[x, y]` | `[0, 0]` | Anchor point in the parent body's local frame |
| `angle_min` | float | `-pi` | Lower angular limit in radians |
| `angle_max` | float | `pi` | Upper angular limit in radians |
| `max_torque` | float | - | Backward-compatible alias for `max_effort` |

`target` is interpreted as:

- angle in radians when `control_mode: position`
- angular velocity in radians/s when `control_mode: velocity`
- torque effort when `control_mode: effort`

```yaml
joints:
    wheel_hinge:
        type: revolute
        parent: chassis
        child: wheel
        anchor: [0.0, -0.5]
        angle_min: -0.6
        angle_max: 0.6
        control_mode: effort
        target: 12.0
        max_effort: 20.0
        pid: [5.0, 0.2, 0.1]
```

### Prismatic Joint

Additional prismatic fields:

| Key | Type | Default | Description |
|---|---|---|---|
| `axis` | `[x, y]` | `[1, 0]` | Motion axis in the parent body's local frame |
| `position_min` | float | `-1000` | Lower translation limit |
| `position_max` | float | `1000` | Upper translation limit |
| `max_force` | float | - | Backward-compatible alias for `max_effort` |

`target` is interpreted as:

- translation along the axis when `control_mode: position`
- linear velocity along the axis when `control_mode: velocity`
- force effort when `control_mode: effort`

```yaml
joints:
    slider:
        type: prismatic
        parent: rail
        child: carriage
        axis: [1.0, 0.0]
        position_min: -2.0
        position_max: 2.0
        control_mode: velocity
        target: 1.5
        max_force: 8.0
        pid: [4.0, 0.0, 0.2]
```

---

## Notes

**Joint units** — entity pose angles in YAML remain in degrees, but joint limits and joint position targets are currently parsed exactly as provided by the joint loader. Revolute limits and revolute position targets should therefore be written in radians.

**Inline YAML strings** are de-indented automatically, so they can be written with natural indentation inside C++ raw string literals without alignment issues.

**Field defaults** — all `physics` fields are optional; the parser substitutes sensible defaults if missing. Missing `visual` or `collision` blocks mean the entity has no visual/collision geometry respectively.

**Unit system** — positions and sizes are in physics units as defined by `scene.size`. Angles are in **degrees** in YAML (converted internally to radians). Velocities are in units/s and degrees/s.
