# Collision Resolution

Fx2D handles collisions in two distinct phases per substep: **detection** (broad + narrow phase geometry tests) and **response** (positional correction followed by velocity-level impulses). Both phases are implemented in `FxSolver` (`src/Collisions.cpp`, declared in `include/Fx2D/Solver.h`).

---

## The `FxContact` Struct

A successful collision check produces an `FxContact`:

| Field | Type | Description |
|---|---|---|
| `entity1`, `entity2` | `shared_ptr<FxEntity>` | The two colliding bodies |
| `normal` | `FxVec2f` | Unit contact normal, pointing **entity1 → entity2** |
| `position` | `FxVec2fArray` | World-space contact point(s) — up to 2 |
| `count` | `size_t` | Number of active contact points (0, 1, or 2) |
| `penetration_depth` | `float` | Signed overlap along `normal` (positive = penetrating) |

`is_valid(full_check)` returns `true` when `count > 0`, `penetration_depth > 0`, and (with `full_check`) both entities are non-null and enabled.

---

## Detection Pipeline

### Broad Phase

`FxSolver::aabb_overlap_check(entity1, entity2)` tests whether the axis-aligned bounding boxes of two entities overlap. It is a cheap pre-filter; pairs that fail are skipped entirely. AABBs are recomputed every substep inside `FxEntity::step()`.

### Narrow Phase

`FxSolver::collision_check(entity1, entity2)` dispatches to `compute_contact_one_way()` based on the shape types of the two entities.

#### Circle–Circle

```
d     = center2 − center1
dist  = ‖d‖
depth = rA + rB − dist

normal = d / dist
contact_point = center1 + normal * rA
```

Active when `depth > 0`.

#### Circle–Polygon

The nearest point on any polygon edge to the circle centre is found. The penetration depth is `r − dist_to_nearest_point`. The contact normal points **from the circle centre toward the nearest polygon edge point** (i.e. in the A → B direction, from circle toward polygon).

#### Polygon–Circle

Internally flips to Circle–Polygon and negates the resulting normal so the convention (`entity1 → entity2`) is preserved.

#### Polygon–Polygon (SAT)

The **Separating Axis Theorem** is applied using every edge normal of shape A as a candidate separating axis:

1. For each edge normal `n̂` of shape A, project all vertices of shape B onto `n̂`.
2. If the minimum projection of B is greater than the maximum projection of A on `n̂`, a separating axis exists → no collision, return early.
3. Track the axis with the **minimum overlap** — this becomes the contact normal and penetration depth.

Once the reference edge (on A) and incident edge (on B) are identified, up to **2 contact points** are computed via `clip_edge()` — a Sutherland–Hodgman-style clipping of the incident edge against the side planes of the reference edge. This gives stable multi-point contacts for flat-face collisions (e.g. a box resting on a plane).

The final normal is re-oriented so it always points **entity1 → entity2**.

---

## Position Correction — `resolve_penetration`

```cpp
FxSolver::resolve_penetration(const FxContact& contact, double dt);
```

Called once per contact per substep, **before** velocity derivation. For each contact point:

**1. Compute moment arms:**

$$\mathbf{r}_A = \mathbf{p}_\text{contact} - \mathbf{x}_A, \qquad \mathbf{r}_B = \mathbf{p}_\text{contact} - \mathbf{x}_B$$

**2. Compute scalar cross products with the normal:**

$$r_{An} = \mathbf{r}_A \times \mathbf{n}, \qquad r_{Bn} = \mathbf{r}_B \times \mathbf{n}$$

**3. Compute effective mass along the normal (with soft compliance):**

$$K_n = w_A + w_B + I_A \, r_{An}^2 + I_B \, r_{Bn}^2 + \frac{\varepsilon}{h^2}$$

Where $\varepsilon = 10^{-8}$ is the compliance of the contact, giving it a small amount of softness to improve numerical stability.

**4. Compute correction magnitude:**

$$\lambda_P = \frac{d - \tau}{n_\text{contacts} \cdot K_n}$$

Where $d$ is `penetration_depth` and $\tau$ is a small slop tolerance. Dividing by contact count distributes work evenly across multi-point contacts.

**5. Apply position corrections:**

$$\mathbf{x}_A \mathrel{-}= w_A \cdot \lambda_P \cdot \mathbf{n}, \qquad \mathbf{x}_B \mathrel{+}= w_B \cdot \lambda_P \cdot \mathbf{n}$$
$$\theta_A \mathrel{-}= I_A \cdot \lambda_P \cdot r_{An}, \qquad \theta_B \mathrel{+}= I_B \cdot \lambda_P \cdot r_{Bn}$$

**`prev_pose` is also corrected by the same amount.** This is critical: velocity is derived as `(pose − prev_pose) / h` in step 5 of the substep pipeline. Without correcting `prev_pose`, the positional fix would generate a large spurious velocity pointing back into the penetration.

---

## Velocity Resolution — `resolve_velocities`

```cpp
FxSolver::resolve_velocities(const FxContact& contact);
```

Called once per contact per substep, **after** velocity derivation. Applies impulses for bounce (restitution) and surface friction.

### Normal Impulse (Restitution)

**1. Compute relative velocity at the contact point:**

$$\mathbf{v}_\text{rel} = (\mathbf{v}_B + \omega_B \times \mathbf{r}_B) - (\mathbf{v}_A + \omega_A \times \mathbf{r}_A)$$

$$v_n = \mathbf{v}_\text{rel} \cdot \mathbf{n}$$

**2. Apply restitution bias** — bounce only if bodies are approaching and the relative normal velocity exceeds a threshold (`1e-3`):

$$e = \min(e_A, e_B), \qquad \text{bias} = \begin{cases} e & v_n < -10^{-3} \\ 0 & \text{otherwise} \end{cases}$$

**3. Compute and apply normal impulse (with accumulation):**

The solver uses **accumulated impulses** per contact point (`jn_accumulated[i]`), clamped to remain non-negative (no tensile forces):

$$j_{n,\text{fresh}} = \frac{-(1 + \text{bias}) \cdot v_n}{K_n}$$
$$j_{n,\text{new}} = \max(0,\; j_{n,\text{accumulated}} + j_{n,\text{fresh}})$$
$$\Delta j_n = j_{n,\text{new}} - j_{n,\text{accumulated}}$$

Only the delta $\Delta j_n$ is applied to velocities. This iterates **twice** over the contact points to improve convergence for multi-point contacts:

$$\mathbf{v}_A \mathrel{-}= w_A \cdot \Delta j_n \cdot \mathbf{n}, \qquad \mathbf{v}_B \mathrel{+}= w_B \cdot \Delta j_n \cdot \mathbf{n}$$
$$\omega_A \mathrel{-}= I_A \cdot \Delta j_n \cdot r_{An}, \qquad \omega_B \mathrel{+}= I_B \cdot \Delta j_n \cdot r_{Bn}$$

### Tangential Impulse (Coulomb Friction)

**1. Compute tangential relative velocity** (`t` = contact tangent, perpendicular to `n`):

$$v_t = \mathbf{v}_\text{rel} \cdot \mathbf{t}$$

**2. Compute unclamped friction impulse:**

$$j_t = \frac{-v_t}{K_t}$$

Where $K_t$ is the effective mass along the tangent, computed analogously to $K_n$.

**3. Clamp to Coulomb cone:**

$$\mu_s = \min(\mu_{s,A},\, \mu_{s,B}), \qquad \mu_k = \min(\mu_{k,A},\, \mu_{k,B})$$

$$j_t = \begin{cases} j_t & |j_t| \leq \mu_s \cdot j_n \quad \text{(static friction — no slip)} \\ \text{sign}(j_t) \cdot \mu_k \cdot j_n & |j_t| > \mu_s \cdot j_n \quad \text{(kinetic friction — sliding)} \end{cases}$$

**4. Apply tangential impulse** to both linear and angular velocities along `t`.

---

## Key Notes

**Normal convention.** The contact normal always points from `entity1` to `entity2`. Corrections push `entity1` in the `−n` direction and `entity2` in the `+n` direction.

**Warm starting.** On the first substep of each frame, `FxSolver::warm_start()` pre-applies the accumulated impulses from the previous frame to both bodies' velocities. This provides a good initial guess for the iterative velocity solver, reducing jitter on resting/stacking contacts. Warm starting is skipped on substeps 2 onward to avoid double-counting impulses already present in the integrated velocity.

**Order of response.** Position correction runs before velocity derivation; velocity impulses run after. This means the restitution and friction impulses work on the velocities that already reflect all positional fixes from constraints and penetration resolution.

**`prev_pose` adjustment.** Both `pose` and `prev_pose` are shifted by the same positional correction so that velocity derivation (`v = Δx/h`) sees a clean, pre-correction baseline.

**Static/kinematic bodies.** Entities with `inv_mass = 0` receive zero correction from all impulse calculations. Their contribution to the effective mass denominator is also zero, meaning the full correction is applied to the dynamic body only.

**Multi-point contacts.** When `count = 2` (e.g. a box face resting on a flat surface), the penetration correction is split evenly across both contact points, preventing over-correction.
