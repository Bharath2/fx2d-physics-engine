# XPBD Solver

Fx2D uses **Extended Position-Based Dynamics (XPBD)** to simulate rigid body physics. The solver runs inside `FxScene::step()` and splits each frame into multiple substeps for stability. Include via:

```cpp
#include "Fx2D/Core.h"
```

---

## Overview

Each call to `FxScene::step(dt)` divides the frame time into **N substeps** (default: 11):

$$h = \frac{dt}{N}$$

Constraints and collisions are solved once per substep. A smaller `h` increases stability and accuracy at the cost of performance.

The substep count can be configured when building a scene:

```cpp
scene.set_substeps(20);   // higher count = more stable, slower
```

---

## Per-Substep Pipeline

Each substep executes the following stages in order:

### 0. Joint Controls

PID-controlled joints apply motor forces/torques before integration:

```
joints.for_each(j => j->apply_controls(h))
```

This ensures joint targets are reflected in the upcoming integration step.

---

### 1. Position Prediction

`FxEntity::step(gravity, h)` is called on every active, non-sleeping entity:

```
v*  = v + h · a(x, v)     // semi-implicit Euler: update velocity first
x*  = x + h · v*           // then advance position
```

Where acceleration `a` includes gravity scaled by `gravity_scale`, plus any user-applied forces and torques divided by mass/inertia. The previous pose `x_prev` is saved before the update for velocity derivation in step 5.

After position update, the entity's collision shape and AABB are synchronised to the new pose.

---

### 2. Collision Detection

Contacts are recomputed every substep using a two-phase process:

1. **Broad phase** — entities whose AABBs overlap are identified as candidate pairs.
2. **Narrow phase** — `FxSolver::collision_check()` runs SAT/geometry tests on each candidate pair, producing `FxContact` structs.

Sleeping entities that receive a new contact are woken up automatically.

See [collision_resolution.md](collision_resolution.md) for the full detection pipeline.

---

### 3. Penetration Correction (Position Level)

For each valid `FxContact`, `FxSolver::resolve_penetration(contact, h)` pushes overlapping bodies apart along the contact normal. This is a soft positional constraint solved with the XPBD kernel (compliance = `1e-8`).

Both `pose` and `prev_pose` are corrected so the velocity derivation in step 5 does not introduce a spurious velocity from the positional fix.

See [collision_resolution.md](collision_resolution.md) for the equations.

---

### 4. Structural Constraint Solving

All `FxConstraint` objects (joints, limits, anchors, etc.) are iterated sequentially and each calls `resolve(h)`:

```
constraints.for_each(seq, c => c->resolve(h))
```

Sequential solving is intentional — it mirrors the Gauss-Seidel-style iteration used in extended PBD and allows later constraints to benefit from earlier corrections within the same substep.

See [The XPBD Kernel](#the-xpbd-kernel) below for the math.

---

### 5. Velocity Derivation

After all positional corrections have been applied, each entity's velocity is recomputed from the displacement over the substep:

$$\mathbf{v} \leftarrow \frac{\mathbf{x} - \mathbf{x}_\text{prev}}{h}$$

$$\omega \leftarrow \frac{\theta - \theta_\text{prev}}{h}$$

This replaces the velocity that was integrated in step 1, letting positional corrections automatically feed back into the velocity state.

---

### 6. Velocity-Level Impulses

Restitution (bounce) and friction are applied as velocity impulses after position solving. This order ensures friction and restitution act on the corrected positions and velocities.

See [collision_resolution.md](collision_resolution.md) for the impulse equations.

---

## The XPBD Kernel

`FxConstraint::resolve(double dt)` in `src/Constraints.cpp` implements the single-iteration XPBD update.

### Inputs

Each constraint provides an `evaluate()` function that computes:

| Symbol | Meaning |
|---|---|
| `C` | Scalar constraint violation (negative = violated for inequality constraints) |
| `g1`, `g2` | Linear gradient `∂C/∂x` for entity 1 and entity 2 |
| `gθ1`, `gθ2` | Rotational gradient `∂C/∂θ` for entity 1 and entity 2 |
| `active` | Whether the constraint is currently active (e.g. inequality within bounds → inactive) |

### Algorithm

**1. Compute the XPBD compliance term:**

$$\alpha = \frac{\text{compliance}}{h^2}$$

Low compliance (default `0.0`) → stiff/rigid. High compliance → soft/springy.

**2. Compute the effective mass denominator:**

$$\text{denom} = w_1 |\mathbf{g}_1|^2 + w_2 |\mathbf{g}_2|^2 + I_1 g_{\theta 1}^2 + I_2 g_{\theta 2}^2 + \alpha$$

Where $w_i = 1/m_i$ (inverse mass) and $I_i = 1/J_i$ (inverse inertia). Static/kinematic bodies have $w = I = 0$ and are unaffected.

**3. Compute the Lagrange multiplier increment:**

$$\Delta\lambda = \frac{-C}{\text{denom}}$$

**4. Apply positional and angular corrections:**

$$\mathbf{x}_1 \mathrel{+}= w_1 \cdot \Delta\lambda \cdot \mathbf{g}_1$$
$$\mathbf{x}_2 \mathrel{+}= w_2 \cdot \Delta\lambda \cdot \mathbf{g}_2$$
$$\theta_1 \mathrel{+}= I_1 \cdot \Delta\lambda \cdot g_{\theta 1}$$
$$\theta_2 \mathrel{+}= I_2 \cdot \Delta\lambda \cdot g_{\theta 2}$$

> **Note:** This implementation uses **single-iteration XPBD** — there is no accumulated `λ` carried between substeps. The compliance `α` alone provides softness. This simplifies the implementation at the cost of some accuracy for very stiff constraints with large substep counts.

---

## Constraint Types

| Constraint | Violation `C` | Linear Gradients | Angular Gradients |
|---|---|---|---|
| `FxAngleLockConstraint` | `wrap(θ₂ − θ₁ − target)` | none | `gθ₁ = −1`, `gθ₂ = +1` |
| `FxAngularLimitConstraint` | `rel_angle − bound` (one-sided, only when violated) | none | `gθ₁ = +1`, `gθ₂ = −1` |
| `FxAnchorConstraint` | `n̂ · (a₁ − a₂)` where `n̂ = (a₁−a₂)/‖a₁−a₂‖` | `g₁ = n̂`, `g₂ = −n̂` | `gθ` from `r × n̂` at each anchor |
| `FxSeparationConstraint` | `(axis · (x₂−x₁)) − initial − bound` (one-sided) | `g₁ = −axis`, `g₂ = axis` | none |
| `FxMotionAlongAxisConstraint` | `axis⊥ · (x₁−x₂) − initial` | `g₁ = axis⊥`, `g₂ = −axis⊥` | none |

All gradients are evaluated in world space each substep, keeping the solver geometry-consistent as bodies rotate.

---

## Design Notes

**No lambda accumulation (structural constraints).** Unlike some XPBD formulations that track `λ` across substeps for warm-starting, structural constraint solving resets each substep. This avoids drift artefacts from stale lambda values when constraint topology changes (e.g. contact arrival/departure).

**Contact warm starting.** Cached impulses from the previous frame are loaded into `jn_warm` / `jt_warm`. At the start of the first substep only, `FxSolver::warm_start()` applies those guesses and seeds `jn_accumulated` / `jt_accumulated` for the current substep. Substeps 2–N leave accumulation at zero so warm impulses are not double-counted after `entity->step()`.

**Static and kinematic bodies.** Any entity with `inv_mass = 0` (set via zero mass in the YAML or API) participates in collision detection but receives zero correction. The same applies to rotational DOF when `inv_inertia = 0`.

**Sleep system.** Entities below both `sleep_threshold_linear` and `sleep_threshold_angular` for `sleep_time_required` consecutive seconds are frozen and excluded from integration and constraint solving. They are woken automatically when a new contact is detected against them.

**Time-step clamping.** `FxScene::step()` throws `std::invalid_argument` if `dt` is below the minimum threshold (`1e-3` s). Above the minimum, `dt` is clamped to the maximum (`0.06` s) before dividing into substeps. This prevents instability from large frame spikes (e.g. window drag on Windows).
