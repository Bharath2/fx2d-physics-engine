#pragma once

#include <algorithm>
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

struct FxContact {
  private:
    bool m_is_valid = false;

  public:
    size_t count = 0; // True if contact is valid
    FxVec2fArray position{{0.0f, 0.0f}, {0.0f, 0.0f}}; // upto to 2 Contact points in world
                                                       // coordinates
    FxVec2f normal{0.0f, 0.0f}; // Contact normal (unit vector)
    float penetration_depth = FxInfinityf; // Penetration depth (positive if overlapping)

    std::shared_ptr<FxEntity> entity1 = nullptr; // First entity in collision
    std::shared_ptr<FxEntity> entity2 = nullptr; // Second entity in collision

    // Impulse applied this substep (may be released; never exceeds what was applied).
    float jn_accumulated[2] = {0.0f, 0.0f};
    float jt_accumulated[2] = {0.0f, 0.0f};

    // Previous substep impulse used as the warm-start guess.
    float jn_warm[2] = {0.0f, 0.0f};
    float jt_warm[2] = {0.0f, 0.0f};

    // Closing speed at substep start — fixes restitution target for every sweep.
    float vn_pre[2] = {0.0f, 0.0f};

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

namespace FxSolver {
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
// Capture closing speeds once per substep before impulses (shared restitution target).
void init_velocity_pass(FxContact& contact);
// Velocity-level solver: restitution and dynamic friction impulses
void resolve_velocities(FxContact& contact);
// Re-apply cached impulses from the previous solve before computing new ones.
void warm_start(FxContact& contact);

} // namespace FxSolver