# Collision Resolution

Fx2D handles collisions in two distinct phases per substep: **detection** (broad + narrow phase geometry tests) and **response** (positional correction followed by velocity-level impulses). Both phases are implemented in `FxSolver` (`src/Collisions.cpp`, declared in `include/Fx2D/Solver.h`).

---

## The `FxContact` Struct

A successful collision check produces an `FxContact`:

| Field | Type | Description |
|---|---|---|
| `entity1`, `entity2` | `FxEntity*` | The two colliding bodies, **borrowed not owned** — see below |
| `normal` | `FxVec2f` | Unit contact normal, pointing **entity1 → entity2** |
| `position` | `FxVec2fArray` | World-space contact point(s) — up to 2 |
| `count` | `size_t` | Number of active contact points (0, 1, or 2) |
| `penetration_depth` | `float` | Signed overlap along `normal` (positive = penetrating, negative = a speculative gap not yet closed) |
| `jn_accumulated`, `jt_accumulated` | `float[2]` | Normal / tangent impulse actually applied this substep, per contact point |
| `jn_warm`, `jt_warm` | `float[2]` | Previous substep's impulses, used as the warm-start guess |
| `vn_pre` | `float[2]` | Closing speed captured at substep start, fixing the restitution target for every sweep |

The per-substep solver scratch — lever arms `rA`/`rB`, their cross products with the contact
basis, the effective masses `K_n`/`K_t`, the inverse masses `wA`/`wB`/`IA`/`IB`, the restitution
target `vn_pre`, and the mixed material constants — lives in a parallel `FxContactSolverData`
array rather than on the contact. It is rebuilt every substep by `init_velocity_pass` and never
read outside the substep, whereas the contact itself is copied into the step buffer once per
contact per substep; carrying the scratch along made that copy twice the size it needed to be.
Keeping it in its own array halved `FxContact` to 120 bytes and is also the column layout a
batched solve would gather.

The remaining fields are solver bookkeeping. They are visible on the contacts `FxScene::contacts()` hands back, but they describe how the solver reached the answer rather than the answer itself, and nothing outside `FxScene::step()` should read them:

| Field | Type | Description |
|---|---|---|
| `body1`, `body2` | `int32_t` | Packed registry indices of the two bodies, used to reach the solver velocity columns without dereferencing a `shared_ptr` |
| `restitution`, `mu_static`, `mu_kinetic` | `float` | Pair material constants, mixed once per substep so the sweeps never touch an entity to read them |
| `pair_key` | `uint64_t` | The two entity ids packed into one key, identifying the pair |
| `cache_slot` | `uint32_t` | Index of this pair's slot in the scene's warm-start cache |

`is_valid(full_check)` returns `true` when the contact was constructed valid and — with `full_check`, the default — both entities are non-null, `count != 0`, `penetration_depth` is finite, and `normal` is non-degenerate (`norm() > 1e-3`). Note the depth test is finiteness, not positivity: speculative contacts carry a **negative** depth (a gap that will close within the substep) and are still valid.

Contacts are no longer discarded once solved. `FxScene` retains them for the duration of the step and exposes them, along with begin/end contact events and sensor overlaps — see [contacts_and_events.md](contacts_and_events.md).

**Entity handles are borrowed.** `entity1` and `entity2` are raw pointers, and so are the ones
on `FxContactEvent`. They were `shared_ptr`, which cost an atomic increment and decrement every
time a contact was copied — once per contact per substep — to re-establish ownership of a body
the scene already owns. Lifetime is guaranteed instead by `FxScene` pinning a `shared_ptr` to
every entity its contact buffers name, rebuilt once per step, so a body deleted between steps
still survives long enough for its end-contact event to name it. The practical rule for callers
is unchanged and now explicit: **contacts and events are valid until the next `step()`** — read
what you need inside the frame, do not store them.

---

## Detection Pipeline

### Broad Phase

The candidate pairs come from a SAH-guided dynamic AABB tree (`include/Fx2D/AABBTree.h`), driven by `FxEntityRegistry` (`include/Fx2D/Registry.h`) in two halves:

- **`sync_broad_phase(dt, sweep_all_movers)`** runs every substep. Bodies have just moved, so each proxy is brought up to date from `FxEntity::bounding_box()` — which `FxEntity::step()` refreshed during integration. The tree stores every box **fattened by 20% of its extent**, so a proxy only has to be reinserted when its body leaves that fat box. The function returns whether any leaf was inserted, removed or reinserted.

  On the first substep the box is also **swept along the body's velocity for the whole step**, so a body moving at constant velocity stays inside the box the tree already holds and needs no reinsertion for the rest of the step. That inflates the pair list slightly, which the tight-AABB filter below rejects cheaply. This was tried once before and reverted for costing more than it saved; it only became worth it after the narrow phase got roughly three times cheaper. See [ToDo.md](ToDo.md).
- **`collect_broad_phase_pairs(pairs)`** walks the tree for overlapping proxies and translates them into packed entity indices, applying the registry-level filters (collision-exclusion pairs, negative collision groups, missing geometry).

`FxScene::step()` runs the walk only when the sync reports the tree changed. If no leaf moved, the tree is the same tree, so the walk would rebuild the identical list in the identical order — skipping it changes nothing and saves a tree descent. In a settling or piling scene about 90% of substeps skip it.

Two filters that used to live in the query now run per substep in the narrow-phase loop, because the pair list outlives the substep that built it and both answers change as the substeps run:

- **Sleeping pairs.** A pair with both bodies asleep is skipped. Filtering this at query time would have stopped a wake from propagating: a body woken partway through the step would have had its pairs dropped from the list *before* it woke.
- **`FxSolver::aabb_overlap_check(entity1, entity2)`**, which tests the two current tight AABBs directly. It is the cheap pre-filter; pairs that fail are skipped without running any geometry. CCD pairs are exempt, since a speculative contact is generated precisely when the bodies are still apart.

`collision_check()` runs the same test internally, so a contact and a broad-phase rejection can never disagree.

### Narrow Phase

`FxSolver::collision_check(entity1, entity2)` dispatches on the shape types of the two entities.

Polygon-vs-polygon is the one case that has to be tested **both ways** — SAT from A's faces and
from B's — because the smaller penetration is the correct one. Only the winner's contact points
are ever used, so the two halves are separate: `sat_query()` runs for each direction, and
`polygon_contact_from_sat()` clips just once, for whichever direction won. A separating axis from
either side ends the test immediately, without starting the other direction. Everything else —
circles, capsules, chains — goes through `compute_contact_one_way()` as before.

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

A chain resolves to the deepest contact any single segment makes, and three rules make that
reliable where naive per-segment dispatch is not:

- **One-sided.** Front is the left of each segment's authored direction, so a polyline written
  left to right supports bodies above it. A zero-thickness segment has no interior; without this,
  a body slipping behind one is pushed further through by a flipped normal.
- **Ghost vertices.** Round bodies are solved with neighbour knowledge: a contact at a segment
  joint must carry a normal inside the arc the two adjacent faces admit (a real corner arc only
  exists at convex joints), otherwise it is clamped to the nearer face normal. In isolation, an
  endpoint contact's normal is whatever direction the vertex makes with the body centre, which
  at a joint can point along the surface and eject the body through it.
- **The chain orients its own normals.** The usual centre-to-centre normal fixup is skipped:
  a chain's entity pose is the polyline origin, nowhere near the contact, so that flip is
  arbitrary and inverted correct face normals on steep terrain.

Chain–chain and chain–edge pairs produce nothing, neither side having volume to resolve, and
chains are excluded from speculative contacts, so a fast enough body still passes through one.

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
FxSolver::init_velocity_pass(FxContact&, FxContactSolverData&, const FxSolverBodies&);
FxSolver::warm_start(FxContact&, const FxContactSolverData&, FxSolverBodies&);

// The scalar sweep. Still the reference definition of the semantics, and still used by
// anything that solves one contact at a time.
FxSolver::resolve_velocities(FxContact&, const FxContactSolverData&, FxSolverBodies&);

// What FxScene::step actually calls: a whole colour at once. See "Batched solve" below.
FxSolver::batch_append(FxContactBatch&, const FxContact&, const FxContactSolverData&, uint32_t);
FxSolver::resolve_velocities_batched(FxContactBatch&, size_t begin, size_t end,
                                     FxSolverBodies&, int slots);
FxSolver::batch_write_back(const FxContactBatch&, std::vector<FxContact>&);
```

Called **after** velocity derivation. `init_velocity_pass` runs once per contact per substep and fills everything the sweeps need — lever arms, effective masses, the restitution target, and the mixed material constants. `resolve_velocities` then runs `velocity_passes` times over every contact (default 4), because a single sweep per contact leaves velocity residuals in a stack.

### The solver velocity columns

These three take an `FxSolverBodies` (`include/Fx2D/Solver.h`) rather than reading velocity off the entities. It is a structure of arrays — `vx`, `vy`, `w`, `inv_m`, `inv_i` — indexed by the packed registry index each contact carries in `body1` / `body2`. `FxScene::step()` gathers it once per substep after velocity derivation and scatters it back after the sweeps.

The reason is cost, not style: the sweeps are the hottest phase of the step, and reaching velocity through a `shared_ptr<FxEntity>` is a pointer chase per body per contact per sweep. A sleeping or immovable body gathers a zero inverse mass, which is what makes it immovable in the sweeps. The layout is also what the colored batch solve in [simd_plan.md](simd_plan.md) is built on.

### Normal Impulse (Restitution)

**1. Compute relative velocity at the contact point:**

$$\mathbf{v}_\text{rel} = (\mathbf{v}_B + \omega_B \times \mathbf{r}_B) - (\mathbf{v}_A + \omega_A \times \mathbf{r}_A)$$

$$v_n = \mathbf{v}_\text{rel} \cdot \mathbf{n}$$

**2. Restitution target** — captured once per substep as `vn_pre` (relative normal velocity before the velocity sweeps). Bounce only if that approach speed exceeds a small slop:

$$e = \max(e_A, e_B), \qquad v_{n,\text{target}} = \begin{cases} -e \cdot v_{n,\text{pre}} & v_{n,\text{pre}} < -v_{\text{slop}} \\ 0 & \text{otherwise} \end{cases}$$

Restitution mixes by `max` and friction by `min`; see **Material mixing** under Key Notes for why the two go opposite ways.

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

### Batched solve

`FxScene::step()` does not sweep contacts one at a time. `FxContactGraph` (`src/ContactGraph.cpp`)
colours the broad-phase pair list so that no two pairs in a colour touch the same movable body;
each contact inherits its pair's colour; and the contacts of one colour are transposed into
columns (`FxContactBatch`) and solved together. Because a colour's contacts write to disjoint
bodies, solving them in any order — or all at once — gives the same answer, and the impulses can
be scattered back without a conflict.

Immovable bodies are excluded from the colouring conflict test: a body with zero inverse mass and
inertia is never modified, so any number of contacts in one colour may share it. That matters
more than anything else in the algorithm, because in a pile or a stack the ground is a party to a
large share of all contacts.

Two properties of the arithmetic make the columns vectorisable:

- **Every contact runs two slots**, whatever its manifold holds, with the unused slot zeroed so
  it reads as inactive. The scalar kernel's `i < count` bound and `(i + iter) % count` index would
  otherwise differ from lane to lane; two fixed slots visited as `(slot + iter) & 1` give the
  identical order.
- **Every branch is a select.** The effective-mass test and the friction cone become blends, with
  the divisor forced to 1 where inactive so a masked lane cannot produce an infinity.

Each colour is further split by manifold size, and the kernel is instantiated for one slot and
for two. Running one-point contacts through the two-slot kernel is correct but doubles their
arithmetic, and a scene of circles is entirely one-point contacts.

The **grouping changes the order contacts are solved in**, which a Gauss-Seidel solver is
sensitive to — impulses propagate through a stack differently. It is a reordering, not an
approximation, and the adversarial suite passed unchanged through it; but it is why the golden
tables were re-baselined when this landed.

## Key Notes

**Normal convention.** The contact normal always points from `entity1` to `entity2`. Corrections push `entity1` in the `−n` direction and `entity2` in the `+n` direction.

**Warm starting.** Cached impulses from the previous frame are stored as `jn_warm` / `jt_warm`. On the first substep only, `FxSolver::warm_start()` applies those guesses and seeds `jn_accumulated` / `jt_accumulated`. Later substeps start accumulation from zero so warm impulses are not double-counted.

**Order of response.** Position correction runs before velocity derivation; velocity impulses run after. This means the restitution and friction impulses work on the velocities that already reflect all positional fixes from constraints and penetration resolution.

**`prev_pose` adjustment.** Both `pose` and `prev_pose` are shifted by the same positional correction so that velocity derivation (`v = Δx/h`) sees a clean, pre-correction baseline.

**Material mixing.** Restitution takes `max(a, b)` and friction takes `min(a, b)`, and the asymmetry is deliberate. A bouncy body should bounce off whatever it hits — under `min` a rubber ball landing on a dead crate inherited the crate's zero and simply stopped, so "make this ball bouncy" meant nothing unless every surface agreed. The cost is that a dead body cannot stay dead against a bouncy floor: to keep something inert, give the surfaces it lands on a low elasticity too. Friction goes the other way so the slipperiest surface wins, and ice stays slippery whatever slides on it. Note the restitution speed floor means resting contacts are unaffected by either rule.

**Static/kinematic bodies.** Entities with `inv_mass = 0` receive zero correction from all impulse calculations. Their contribution to the effective mass denominator is also zero, meaning the full correction is applied to the dynamic body only.

**Multi-point contacts.** When `count = 2` (e.g. a box face resting on a flat surface), the penetration correction is split evenly across both contact points, preventing over-correction.
