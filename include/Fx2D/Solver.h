#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

#include "Fx2D/Math.h"

// Forward declaration
class FxEntity;

// Result of a single-sided SAT overlap query (A's edges tested against B).
// gap < 0: bodies overlapping, |gap| = penetration depth on that axis.
// has_sep: true if a separating axis was found (no overlap from A's side).
struct FxSatResult {
    FxVec2f normal = {1.0f, 0.0f}; // edge normal of the min-penetration axis
    float gap = -FxInfinityf; // maximum B_min_val seen (signed separation)
    bool has_sep = false; // true if any edge gave gap > 0
    size_t ref_edge_index = 0; // index of that edge in A's vertex list
    FxVec2f ref_edge_dir = {0.0f, 0.0f}; // normalized direction along that edge
    size_t pen_vertex_index = 0; // index of B's deepest vertex on that edge
};

// Solver-local body state, structure of arrays, one entry per registry packed index. Gathered
// once per substep so the sweeps address bodies by index rather than chasing shared_ptrs, and
// the layout the batched solve reads. See docs/simd_plan.md.
struct FxSolverBodies {
    std::vector<float> vx, vy, w; // linear and angular velocity
    std::vector<float> inv_m, inv_i; // effective inverse mass and inertia (0 when immovable)

    void resize(std::size_t n) {
        vx.resize(n);
        vy.resize(n);
        w.resize(n);
        inv_m.resize(n);
        inv_i.resize(n);
    }

    std::size_t size() const { return vx.size(); }

    // Velocity of the material point at body-relative offset r. Same expression as
    // FxEntity::velocity_at_local_point, kept identical so the two cannot drift apart.
    FxVec2f velocity_at(int32_t body, const FxVec2f& r) const {
        const std::size_t i = static_cast<std::size_t>(body);
        return FxVec2f(vx[i] - w[i] * r.y(), vy[i] + w[i] * r.x());
    }
};

// Per-substep sweep scratch for one contact: lever arms, their cross products with the contact
// basis, the effective masses, the restitution target and the mixed material constants. Kept
// beside the contact, not inside it, so copying a contact does not carry it.
struct FxContactSolverData {
    FxVec2f rA[2]{{0.0f, 0.0f}, {0.0f, 0.0f}};
    FxVec2f rB[2]{{0.0f, 0.0f}, {0.0f, 0.0f}};
    float ra_n[2] = {0.0f, 0.0f}, rb_n[2] = {0.0f, 0.0f};
    float ra_t[2] = {0.0f, 0.0f}, rb_t[2] = {0.0f, 0.0f};
    float K_n[2] = {0.0f, 0.0f}, K_t[2] = {0.0f, 0.0f};
    float wA = 0.0f, wB = 0.0f, IA = 0.0f, IB = 0.0f;

    // Closing speed at substep start — fixes the restitution target for every sweep.
    float vn_pre[2] = {0.0f, 0.0f};

    // Pair material constants, mixed once per substep so the sweeps never touch an entity to
    // find out how bouncy or how rough the pair is.
    float restitution = 0.0f;
    float mu_static = 0.0f;
    float mu_kinetic = 0.0f;
};

struct FxContact {
    // Sentinel for cache_slot below: this contact has not been registered with a scene.
    static constexpr uint32_t kNoCacheSlot = 0xffffffffu;

  private:
    bool m_is_valid = false;

  public:
    size_t count = 0; // True if contact is valid
    FxVec2f position[2] = {{0.0f, 0.0f}, {0.0f, 0.0f}}; // up to 2 contact points in world
                                                        // coordinates
    FxVec2f normal{0.0f, 0.0f}; // Contact normal (unit vector)
    float penetration_depth = FxInfinityf; // Penetration depth (positive if overlapping)

    // The two colliding bodies, borrowed not owned: FxScene pins a shared_ptr to every entity
    // its contact buffers name, so one deleted between steps still survives to be reported.
    // Valid until the next step(); do not keep a contact past the frame that produced it.
    FxEntity* entity1 = nullptr; // First entity in collision
    FxEntity* entity2 = nullptr; // Second entity in collision

    // Impulse applied this substep (may be released; never exceeds what was applied).
    float jn_accumulated[2] = {0.0f, 0.0f};
    float jt_accumulated[2] = {0.0f, 0.0f};

    // Previous substep impulse used as the warm-start guess.
    float jn_warm[2] = {0.0f, 0.0f};
    float jt_warm[2] = {0.0f, 0.0f};

    // Packed indices of the two bodies into FxSolverBodies, filled when the contact is detected.
    // The shared_ptrs above stay for the public contact buffer; the solver uses these.
    int32_t body1 = -1;
    int32_t body2 = -1;

    // Solver-internal pair bookkeeping: the key identifies the entity pair, the slot is where
    // its warm-start impulses and step-buffer position live. Exposed through contacts() only
    // because the whole struct is; nothing outside the solver should read them.
    uint64_t pair_key = 0;
    uint32_t cache_slot = kNoCacheSlot;

    // Constructor overloads
    FxContact() = default;
    FxContact(bool valid) : m_is_valid(valid) {}

    // method to check validity
    bool is_valid(bool full_check = true) const {
        return m_is_valid &&
               (!full_check || (entity1 != nullptr && entity2 != nullptr && count != 0 &&
                                std::isfinite(penetration_depth) && normal.norm() > 1e-3f));
    }
    void set_valid(bool valid) { m_is_valid = valid; }
};

// Position-based constraint base class for XPBD solver
class FxConstraint {
    // Joints rename the constraints they own; see FxJoint::namespace_constraints.
    friend class FxJoint;

  protected:
    std::string m_name; // "id1_id2_constraint-name"
    double compliance = 1e-7; // XPBD alpha = compliance / dt^2
    // When set, prev_pose moves with pose so the correction registers no velocity. Velocity is
    // derived as (pose - prev_pose)/h, so a constraint that only moves pose is a spring that
    // converts every pull into momentum -- right for a joint, wrong for dragging a body.
    bool carries_velocity = true;
    std::shared_ptr<FxEntity> entity1;
    std::shared_ptr<FxEntity> entity2;

  public:
    // bool entities_collide = false;     // Whether connected entities should collide
    // Set stiffness (converts to compliance internally)
    void set_stiffness(double k) {
        if (k <= 0.0) return;
        compliance = 1.0 / k;
    }
    void setCompliance(double c) { compliance = std::max(0.0, c); }
    // Evaluate C and gradients; set active=false to skip
    virtual void evaluate(float& C, FxVec2f& g1, FxVec2f& g2, float& gth1, float& gth2,
                          bool& active) const = 0;
    // One-iteration XPBD/PBD correction (no lambda term in numerator)
    void resolve(double dt);
    // Accessor method for name (required by FxNamedRegistry)
    const std::string& get_name() const { return m_name; }

    // Entity accessor methods (read-only)
    const std::shared_ptr<FxEntity>& get_entity1() const { return entity1; }
    const std::shared_ptr<FxEntity>& get_entity2() const { return entity2; }

    // Entity name accessor methods
    std::string get_entity1_name() const;
    std::string get_entity2_name() const;

    virtual ~FxConstraint() = default;
};

// Angular limit constraint that restricts the relative angle between two entities
// to be within a specified range [lower, upper]. The constraint is only active when violated
class FxAngularLimitConstraint : public FxConstraint {
  public:
    float lower_limit = 0.0f; // Lower angle limit (degrees)
    float upper_limit = FxPif; // Upper angle limit (degrees)
    float slop = 0.01f; // Tolerance zone around limits
    bool enabled = true; // Whether this constraint is active
    FxAngularLimitConstraint(const std::shared_ptr<FxEntity>& e1,
                             const std::shared_ptr<FxEntity>& e2);
    void evaluate(float& C, FxVec2f& g1, FxVec2f& g2, float& gth1, float& gth2,
                  bool& active) const override;
};

// Constraint that locks relative angle between two entities to a target value
class FxAngleLockConstraint : public FxConstraint {
  public:
    float target = 0.0f; // Target relative angle
    bool enabled = true; // Whether this constraint is active
    FxAngleLockConstraint(const std::shared_ptr<FxEntity>& e1, const std::shared_ptr<FxEntity>& e2,
                          float tgt = 0.0f);
    void evaluate(float& C, FxVec2f& g1, FxVec2f& g2, float& gth1, float& gth2,
                  bool& active) const override;
};

// Holds a body's anchor point at a world target, softly. The second body is deliberately null:
// the target is a point in the world, not another entity, which is what a dragged cursor is.
class FxMouseConstraint : public FxConstraint {
  private:
    FxVec2f m_local_anchor; // grab point, in entity1's frame
    FxVec2f m_target{0.0f, 0.0f}; // where that point is being pulled to, in world space

  public:
    bool enabled = true;
    FxMouseConstraint(const std::shared_ptr<FxEntity>& e1, const FxVec2f& anchor,
                      bool anchor_is_local = true);
    void set_target(const FxVec2f& world_point) { m_target = world_point; }
    const FxVec2f& target() const { return m_target; }
    const FxVec2f& local_anchor() const { return m_local_anchor; }
    void evaluate(float& C, FxVec2f& g1, FxVec2f& g2, float& gth1, float& gth2,
                  bool& active) const override;
};

// Constraint that projects the separation between two anchor points onto a specified world axis
class FxAnchorConstraint : public FxConstraint {
  private:
    FxVec2f m_anchor1; // Local anchor point on entity1
    FxVec2f m_anchor2; // Local anchor point on entity2
  public:
    bool enabled = true; // Whether this constraint is active
    FxAnchorConstraint(const std::shared_ptr<FxEntity>& e1, const std::shared_ptr<FxEntity>& e2,
                       const FxVec2f& anchor, bool anchor_is_local = true);
    void evaluate(float& C, FxVec2f& g1, FxVec2f& g2, float& gth1, float& gth2,
                  bool& active) const override;
};

// Linear limit constraint that restricts the projection of separation between two entities onto a
// specified axis
class FxSeparationConstraint : public FxConstraint {
  private:
    FxVec2f m_axis; // Axis direction (normalized)
    bool m_axis_is_local; // Whether axis is local to entity1 or in world coordinates
    float m_initial_projection;

  public:
    float lower_limit = 0; // Lower limit for projection
    float upper_limit = 10; // Upper limit for projection
    float slop = 0.0001f; // Tolerance zone around limits
    bool enabled = true; // Whether this constraint is active
    FxSeparationConstraint(const std::shared_ptr<FxEntity>& e1, const std::shared_ptr<FxEntity>& e2,
                           const FxVec2f& axis, bool axis_is_local = true);
    void evaluate(float& C, FxVec2f& g1, FxVec2f& g2, float& gth1, float& gth2,
                  bool& active) const override;
};

// Constraint that forces motion along a specified axis
class FxMotionAlongAxisConstraint : public FxConstraint {
  private:
    FxVec2f m_axis; // Axis direction (normalized)
    bool m_axis_is_local; // Whether axis is local to entity1 or in world coordinates
    float m_initial_projection; // Initial perpendicular distance projection
  public:
    bool enabled = true; // Whether this constraint is active
    FxMotionAlongAxisConstraint(const std::shared_ptr<FxEntity>& e1,
                                const std::shared_ptr<FxEntity>& e2, const FxVec2f& axis,
                                bool axis_is_local = true);
    void evaluate(float& C, FxVec2f& g1, FxVec2f& g2, float& gth1, float& gth2,
                  bool& active) const override;
};

// Partitions the broad-phase pair list into colors, where no two pairs in a color touch the
// same movable body -- so a color's contacts write to disjoint bodies and can be solved
// together. Greedy in pair order, so deterministic. See docs/collision_resolution.md.
class FxContactGraph {
  public:
    // Colors are capped so the per-color bookkeeping stays a fixed, small cost. Contacts that
    // find no free color land in an overflow group, which is solved one contact at a time.
    static constexpr uint32_t kMaxColors = 12;

    // Color `pairs`, leaving the list itself alone -- reordering it costs the narrow phase its
    // locality. `entities` is the registry packed storage, indexed by the pair entries.
    // Buffers are reused, so this allocates nothing after the first call.
    void color_pairs(const std::vector<std::pair<size_t, size_t>>& pairs,
                     const std::vector<std::shared_ptr<FxEntity>>& entities);

    // The color assigned to pair i, as an index the contact carries forward. kMaxColors means
    // the pair overflowed and must be solved on its own.
    uint32_t color_of_pair(size_t i) const { return m_color_of[i]; }

    // Turn per-contact colors into a sweep order: `order` comes back holding contact indices
    // grouped by color. Cheap -- it sorts four-byte indices, not contacts.
    void group_contacts(const std::vector<uint32_t>& contact_colors, std::vector<uint32_t>& order);

    // Group g spans [group_start(g), group_start(g + 1)) in that order.
    size_t group_count() const { return m_group_starts.empty() ? 0 : m_group_starts.size() - 1; }
    uint32_t group_start(size_t g) const { return m_group_starts[g]; }

    // True when group g holds the contacts whose pairs did not fit a color. They may share
    // bodies with each other and must be solved sequentially, never batched.
    bool is_overflow_group(size_t g) const { return g == m_overflow_group; }

    // Pairs placed into a color, and those that overflowed. Diagnostics only.
    size_t colored_count() const { return m_colored_count; }
    size_t overflow_count() const { return m_overflow_count; }

  private:
    std::vector<uint32_t> m_group_starts; // group_count() + 1 entries
    std::vector<uint32_t> m_color_of; // per pair: color index, or kMaxColors for overflow
    std::vector<uint64_t> m_used; // kMaxColors bitsets, one bit per body
    std::vector<uint32_t> m_counts; // pairs per group, then reused as write cursors
    std::vector<uint8_t> m_movable; // per body: can an impulse move it at all
    std::vector<uint32_t> m_group_of_color; // color -> sweep group, or kUncolored if unused
    std::vector<uint32_t> m_group_counts; // counting-sort cursors for group_contacts
    size_t m_overflow_group = static_cast<size_t>(-1); // sweep group holding overflow, if any
    size_t m_colored_count = 0;
    size_t m_overflow_count = 0;
    bool m_has_overflow = false;
};

// One color's contacts transposed into columns, built once per substep and swept
// velocity_passes times. Always two slots per contact: a one-point manifold leaves slot 1 with
// zero effective mass, read as inactive, so no lane has to branch on manifold size.
struct FxContactBatch {
    // Per contact.
    std::vector<int32_t> ia, ib;
    std::vector<float> nx, ny, tx, ty;
    std::vector<float> wA, wB, IA, IB;
    std::vector<float> restitution, mu_s, mu_k;
    std::vector<uint32_t> contact_index; // where to write the impulses back
    std::vector<float> jn_sum;

    // The two bodies' velocities, gathered in and scattered out once per colour per pass. Held
    // in columns so everything between the gather and the scatter is contiguous.
    std::vector<float> vax, vay, wav, vbx, vby, wbv;

    // Per contact, per slot.
    std::vector<float> rAx[2], rAy[2], rBx[2], rBy[2];
    std::vector<float> ra_n[2], rb_n[2], ra_t[2], rb_t[2];
    std::vector<float> K_n[2], K_t[2], vn_pre[2];
    std::vector<float> jn[2], jt[2];

    std::size_t size() const { return ia.size(); }

    // Every column, visited once. The single place that knows the full list, so adding a column
    // cannot leave one uncleared and carrying last substep's data.
    template<typename Fn>
    void for_each_column(Fn fn) {
        fn(ia);
        fn(ib);
        fn(nx);
        fn(ny);
        fn(tx);
        fn(ty);
        fn(wA);
        fn(wB);
        fn(IA);
        fn(IB);
        fn(restitution);
        fn(mu_s);
        fn(mu_k);
        fn(contact_index);
        fn(jn_sum);
        fn(vax);
        fn(vay);
        fn(wav);
        fn(vbx);
        fn(vby);
        fn(wbv);
        for (int s = 0; s < 2; ++s) {
            fn(rAx[s]);
            fn(rAy[s]);
            fn(rBx[s]);
            fn(rBy[s]);
            fn(ra_n[s]);
            fn(rb_n[s]);
            fn(ra_t[s]);
            fn(rb_t[s]);
            fn(K_n[s]);
            fn(K_t[s]);
            fn(vn_pre[s]);
            fn(jn[s]);
            fn(jt[s]);
        }
    }

    void clear() {
        for_each_column([](auto& column) { column.clear(); });
    }

    // Grow the velocity columns to match the contact columns after a build.
    void size_velocity_columns() {
        vax.resize(ia.size());
        vay.resize(ia.size());
        wav.resize(ia.size());
        vbx.resize(ia.size());
        vby.resize(ia.size());
        wbv.resize(ia.size());
        jn_sum.resize(ia.size());
    }
};

namespace FxSolver {
// True when a contact is one the velocity solver will actually act on. Shared so the contact
// graph colors exactly the set the sweeps visit -- coloring a contact the solver then skips
// would waste a color slot and could push a real contact into the overflow group.
inline bool contact_is_solvable(const FxContact& contact) {
    if (!contact.is_valid() || contact.penetration_depth <= 0.0f) return false;
    if (!contact.entity1 || !contact.entity2) return false;
    return contact.body1 >= 0 && contact.body2 >= 0;
}

// AABB overlap check methods
bool aabb_overlap_check(const FxEntity& entity1, const FxEntity& entity2);
bool aabb_overlap_check(const std::shared_ptr<FxEntity>& entity1,
                        const std::shared_ptr<FxEntity>& entity2);

// Main collision detection method using SAT
const FxContact collision_check(const FxEntity& entity1, const FxEntity& entity2);
const FxContact collision_check(const std::shared_ptr<FxEntity>& entity1,
                                const std::shared_ptr<FxEntity>& entity2);

// Speculative contact for CCD bodies: generates a pre-contact when bodies are separated but
// approaching fast enough to close the gap within this substep.
FxContact speculative_contact_check(const std::shared_ptr<FxEntity>& entity1,
                                    const std::shared_ptr<FxEntity>& entity2, float substep_dt);

// Main collision resolution method
void resolve_penetration(const FxContact& contact, double dt = 0.016f);
// Capture closing speeds once per substep before impulses (shared restitution target), and
// resolve the per-substep constants the sweeps need. Reads pose and material properties from the
// entities, velocity from the columns.
void init_velocity_pass(FxContact& contact, FxContactSolverData& data,
                        const FxSolverBodies& bodies);
// Velocity-level solver: restitution and dynamic friction impulses. Columns only.
// Append one contact to the batch. Caller guarantees contact_is_solvable(contact).
void batch_append(FxContactBatch& batch, const FxContact& contact, const FxContactSolverData& data,
                  uint32_t contact_index);
// Solve batch entries [begin, end) -- one colour and one manifold size, so no two entries
// share a movable body and every lane runs the same number of contact points.
void resolve_velocities_batched(FxContactBatch& batch, std::size_t begin, std::size_t end,
                                FxSolverBodies& bodies, int slots);
// Copy the accumulated impulses back onto the contacts, for the warm-start cache.
void batch_write_back(const FxContactBatch& batch, std::vector<FxContact>& contacts);
// Re-apply cached impulses from the previous solve before computing new ones. Columns only.
void warm_start(FxContact& contact, const FxContactSolverData& data, FxSolverBodies& bodies);

} // namespace FxSolver