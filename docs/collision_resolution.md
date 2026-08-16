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
| `penetration_depth` | `float` | Signed overlap along `normal` (positive = penetrating, negative = a speculative gap not yet closed) |
| `jn_accumulated`, `jt_accumulated` | `float[2]` | Normal / tangent impulse actually applied this substep, per contact point |
| `jn_warm`, `jt_warm` | `float[2]` | Previous substep's impulses, used as the warm-start guess |
| `vn_pre` | `float[2]` | Closing speed captured at substep start, fixing the restitution target for every sweep |

`is_valid(full_check)` returns `true` when the contact was constructed valid and — with `full_check`, the default — both entities are non-null, `count != 0`, `penetration_depth` is finite, and `normal` is non-degenerate (`norm() > 1e-3`). Note the depth test is finiteness, not positivity: speculative contacts carry a **negative** depth (a gap that will close within the substep) and are still valid.

Contacts are no longer discarded once solved. `FxScene` retains them for the duration of the step and exposes them, along with begin/end contact events and sensor overlaps — see [contacts_and_events.md](contacts_and_events.md).

---

## Detection Pipeline

### Broad Phase

`FxSolver::aabb_overlap_check(entity1, entity2)` tests whether the axis-aligned bounding boxes of two entities overlap. It is a cheap pre-filter; pairs that fail are skipped entirely. AABBs are recomputed every substep inside `FxEntity::step()`.

### Narrow Phase

`FxSolver::collision_check(entity1, entity2)` dispatches to `compute_contact_one_way()` based on the shape types of the two entities.

#### Unified shape model

All shapes are stored as `vertices[] + skin_radius`:

| Shape   | Vertex count | Skin radius |
|---------|--------------|-------------|
| Circle  | 0            | radius      |
| Capsule | 2 (segment endpoints) | end-cap radius |
| Edge    | 2 (segment endpoints) | 0 (zero-thickness segment) |
| Polygon | ≥ 3          | 0 (sharp) or > 0 (rounded corners) |
| Chain   | ≥ 3 (open polyline) | 0 (zero-thickness segments) |

Every contact computation works in two steps: find the closest features between the two raw cores (point, segment, or polygon), then subtract `rA + rB` from the resulting gap. The contact point lies on the skin surface of B (i.e. shifted by `-rB` along the normal from the raw core contact).

#### Circle–Circle

```
d     = center2 − center1
dist  = ‖d‖
depth = rA + rB − dist
```

Active when `depth > 0`. `normal = d / dist`; `contact_point = center1 + normal * (rA - depth/2)`.

#### Circle–Polygon

The nearest point on any polygon edge to the circle centre is found. The penetration depth is `(rA + rB) − dist`. The contact normal points **from the circle centre toward the nearest polygon edge point** (i.e. in the A → B direction, from circle toward polygon). For rounded polygons, the contact point is shifted by `-rB · normal` to lie on the polygon's skin surface.

#### Polygon–Circle

Internally flips to Circle–Polygon and negates the resulting normal so the convention (`entity1 → entity2`) is preserved.

#### Capsule–X

Capsule collisions reduce to "virtual circle at closest segment point" vs the other shape:

| Other shape | Reduction |
|-------------|-----------|
| Circle      | closest point on capsule segment to circle centre → circle–circle |
| Capsule     | closest pair between the two segments (`seg_seg_closest`) → circle–circle |
| Polygon     | closest segment-to-edge pair across all polygon edges → circle–polygon |

The capsule's `skin_radius` plays the role of the virtual circle's radius. A zero-length capsule reduces exactly to a circle; a zero-radius capsule is a bare line segment (an edge).

#### Edge–X

An edge is a zero-skin capsule, so edge–circle and edge–capsule pairs use the capsule reductions above with `rA = 0`.

Edge–polygon needs its own query (`edge_polygon_contact`). The capsule reduction measures the distance to the polygon *boundary*, which stays positive once the segment lies inside the polygon, so a bare segment would report no contact exactly when it is most deeply penetrating. Instead the segment's line becomes the reference axis: the normal is the segment perpendicular oriented toward the polygon centroid, and the polygon vertices lying behind that line (within the segment's span, offset by the polygon's own skin) supply up to two contact points, deepest first.

Edge–edge pairs never produce contacts, since neither shape has volume to resolve. Edges are also skipped by speculative-contact CCD: the distance-minus-radii gap math degenerates without a skin, and edges are static level geometry where discrete contacts suffice.

#### Chain–X

A chain is a run of edges sharing one entity, so it resolves to the deepest contact any single
segment makes: each segment is handed to the edge paths above and the best result wins. No new
geometry is involved, which also means a chain inherits every edge limitation. Chain–chain and
chain–edge pairs produce nothing, neither side having volume to resolve, and chains are excluded
from speculative contacts, so a fast enough body passes through one.

Reporting only the deepest segment costs nothing in practice: a body spanning two segments at a
joint is pushed out along the deeper of the two, and the next substep resolves the other.

#### Polygon–Polygon (SAT, skin-aware)

The **Separating Axis Theorem** is applied using every edge normal of shape A as a candidate separating axis:

1. For each edge normal `n̂` of shape A, project all vertices of shape B onto `n̂`.
2. Compute the skin-inclusive gap as `min_B_projection − (rA + rB)`. If positive on any axis, a separating axis exists → no collision, return early.
3. Track the axis with the **minimum (most-negative) gap** — this becomes the contact normal and `−gap` is the penetration depth.

Contact-point clipping (`clip_edge`, Sutherland–Hodgman style) operates on the raw vertices, then the resulting points are shifted by `-rB · normal` to land on B's skin surface. This unifies sharp-corner polygons (`rB = 0`) and rounded ones with a single code path.

The final normal is re-oriented so it always points **entity1 → entity2**.

Once the reference edge (on A) and incident edge (on B) are identified, up to **2 contact points** are computed via `clip_edge()` — a Sutherland–Hodgman-style clipping of the incident edge against the side planes of the reference edge. This gives stable multi-point contacts for flat-face collisions (e.g. a box resting on a plane).

### Speculative Contacts (CCD)

Discrete detection samples geometry at substep boundaries, so a body moving fast enough to cross a thin collider between two samples passes straight through it. `FxSolver::speculative_contact_check()` closes most of that gap. It runs only when `collision_check()` found no overlap **and** at least one entity has `enable_ccd = true` (YAML `ccd: true`).

The idea is to create the contact *before* the shapes touch, so the solver can arrest the motion in time:

1. Measure the current skin-inclusive `gap` between the two shapes, reusing the same reductions as the discrete path — circle/capsule pairs via closest points minus radii, rounded-vs-polygon via the closest point on the polygon boundary, and polygon–polygon via `sat_gap_query()` run both ways, keeping the smaller gap.
2. Compute the closing speed along that normal, `v_closing = -rel_vel · n̂`. If the bodies are separating (`v_closing <= 0`), there is nothing to anticipate and no contact is produced.
3. Predict the gap at the end of the substep: `spec_depth = gap - v_closing · dt`. If it is still non-negative the bodies cannot meet this substep, so again no contact.
4. Otherwise emit a single-point contact whose `penetration_depth` is that **negative** `spec_depth`.

A negative depth is what makes this work: the position solver treats it as "do not approach closer than this", removing exactly the normal velocity that would have caused the overpenetration, rather than pushing the bodies apart. This is why `is_valid()` tests `penetration_depth` for finiteness rather than positivity.

Edges are excluded (`A->is_edge() || B->is_edge()` returns early). A bare segment has no skin, so the distance-minus-radii math degenerates, and edges are static level geometry where discrete contacts suffice.

Speculative contacts reduce tunneling but do not eliminate it, since they still sample velocity once per substep. Time-of-impact sweeps remain future work (see item 3 in [ToDo.md](ToDo.md)).

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

**2. Restitution target** — captured once per substep as `vn_pre` (relative normal velocity before the velocity sweeps). Bounce only if that approach speed exceeds a small slop:

$$e = \min(e_A, e_B), \qquad v_{n,\text{target}} = \begin{cases} -e \cdot v_{n,\text{pre}} & v_{n,\text{pre}} < -v_{\text{slop}} \\ 0 & \text{otherwise} \end{cases}$$

**3. Compute and apply normal impulse (with accumulation):**

The solver uses **accumulated impulses** per contact point (`jn_accumulated[i]`), clamped to remain non-negative (no tensile forces). Negative $\Delta j_n$ is applied too, so excess impulse from an earlier sweep can be released:

$$j_{n,\text{fresh}} = \frac{-(v_n - v_{n,\text{target}})}{K_n}$$
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

The Coulomb budget uses the **sum** of normal impulses on the manifold for this substep ($j_{n,\text{sum}} = \sum_i j_{n,\text{accumulated}}[i]$), computed once after the normal loop:

$$j_t = \begin{cases} j_t & |j_t| \leq \mu_s \cdot j_{n,\text{sum}} \quad \text{(static friction — no slip)} \\ \text{sign}(j_t) \cdot \mu_k \cdot j_{n,\text{sum}} & |j_t| > \mu_s \cdot j_{n,\text{sum}} \quad \text{(kinetic friction — sliding)} \end{cases}$$

Unset friction in YAML defaults to `0`, and pair coefficients use $\min(\mu_A,\mu_B)$ — so a body with no friction makes the contact frictionless.

**4. Apply tangential impulse** to both linear and angular velocities along `t`.

---

## Key Notes

**Normal convention.** The contact normal always points from `entity1` to `entity2`. Corrections push `entity1` in the `−n` direction and `entity2` in the `+n` direction.

**Warm starting.** Cached impulses from the previous frame are stored as `jn_warm` / `jt_warm`. On the first substep only, `FxSolver::warm_start()` applies those guesses and seeds `jn_accumulated` / `jt_accumulated`. Later substeps start accumulation from zero so warm impulses are not double-counted.

**Order of response.** Position correction runs before velocity derivation; velocity impulses run after. This means the restitution and friction impulses work on the velocities that already reflect all positional fixes from constraints and penetration resolution.

**`prev_pose` adjustment.** Both `pose` and `prev_pose` are shifted by the same positional correction so that velocity derivation (`v = Δx/h`) sees a clean, pre-correction baseline.

**Material mixing.** Restitution takes `max(a, b)` and friction takes `min(a, b)`, and the asymmetry is deliberate. A bouncy body should bounce off whatever it hits — under `min` a rubber ball landing on a dead crate inherited the crate's zero and simply stopped, so "make this ball bouncy" meant nothing unless every surface agreed. The cost is that a dead body cannot stay dead against a bouncy floor: to keep something inert, give the surfaces it lands on a low elasticity too. Friction goes the other way so the slipperiest surface wins, and ice stays slippery whatever slides on it. Note the restitution speed floor means resting contacts are unaffected by either rule.

**Static/kinematic bodies.** Entities with `inv_mass = 0` receive zero correction from all impulse calculations. Their contribution to the effective mass denominator is also zero, meaning the full correction is applied to the dynamic body only.

**Multi-point contacts.** When `count = 2` (e.g. a box face resting on a flat surface), the penetration correction is split evenly across both contact points, preventing over-correction.
