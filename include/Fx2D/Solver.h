#pragma once

#include <vector>
#include <memory>
#include <algorithm>

#include "Fx2D/Math.h"

// Forward declaration
class FxEntity;

struct FxContact {
  private:
    bool m_is_valid = false;  
  public:
    size_t count = 0;                                             // True if contact is valid
    FxArray<FxVec2f> position {{0.0f, 0.0f}, {0.0f, 0.0f}};      // upto to 2 Contact points in world coordinates
    FxVec2f normal {0.0f, 0.0f};              // Contact normal (unit vector)
    float penetration_depth = FxInfinityf;    // Penetration depth (positive if overlapping)

    std::shared_ptr<FxEntity> entity1 = nullptr;  // First entity in collision
    std::shared_ptr<FxEntity> entity2 = nullptr;  // Second entity in collision
    
    // Constructor overloads
    FxContact() = default;
    FxContact(bool valid): m_is_valid(valid) {}
    
    // method to check validity
    bool is_valid(bool full_check = true) const { 
        return m_is_valid && (!full_check || (entity1 != nullptr && entity2 != nullptr && 
               count != 0 && std::isfinite(penetration_depth) && normal.norm() > 1e-3f));
    }
    void set_valid(bool valid) { m_is_valid = valid; }
};


// Position-based constraint base class for XPBD solver
class FxConstraint {
  public:
    std::string name;                 // "id1_id2_constraint-name"
    float compliance = 1e-7f;          // XPBD alpha = compliance / dt^2
    // bool entities_collide = false;     // Whether connected entities should collide
    std::shared_ptr<FxEntity> entity1;
    std::shared_ptr<FxEntity> entity2;
    // Set stiffness (converts to compliance internally)
    void set_stiffness(float k) {
        if (k <= 0.0f) return;
        compliance = 1.0f / k;
    }
    void setCompliance(float c) { 
        compliance = std::max(0.0f, c); 
    }
    // Evaluate C and gradients; set active=false to skip
    virtual void evaluate(float& C, FxVec2f& g1, FxVec2f& g2,
                          float& gth1, float& gth2, bool& active) const = 0;
    // One-iteration XPBD/PBD correction (no lambda term in numerator)
    void resolve(float dt);
    virtual ~FxConstraint() = default;
};


// Angular limit constraint that restricts the relative angle between two entities
// to be within a specified range [lower, upper]. The constraint is only active when violated
class FxAngularLimitConstraint : public FxConstraint {
public:
    float lower_limit = 0.0f;    // Lower angle limit (degrees)
    float upper_limit = FxPif;    // Upper angle limit (degrees)
    float slop  = 0.01f;    // Tolerance zone around limits
    bool enabled = true;  // Whether this constraint is active
    FxAngularLimitConstraint(std::shared_ptr<FxEntity> e1, std::shared_ptr<FxEntity> e2);
    void evaluate(float& C, FxVec2f& g1, FxVec2f& g2,
                  float& gth1, float& gth2, bool& active) const override;
};

// Constraint that locks relative angle between two entities to a target value
class FxAngleLockConstraint : public FxConstraint {
public:
    float target = 0.0f;   // Target relative angle
    bool enabled = true;  // Whether this constraint is active
    FxAngleLockConstraint(std::shared_ptr<FxEntity> e1, std::shared_ptr<FxEntity> e2, float tgt = 0.0f);
    void evaluate(float& C, FxVec2f& g1, FxVec2f& g2,
                  float& gth1, float& gth2, bool& active) const override;
};

// Constraint that projects the separation between two anchor points onto a specified world axis
class FxAnchorConstraint : public FxConstraint {
private:
    FxVec2f m_anchor1;     // Local anchor point on entity1
    FxVec2f m_anchor2;     // Local anchor point on entity2
public:
    bool enabled = true; // Whether this constraint is active
    FxAnchorConstraint(std::shared_ptr<FxEntity> e1, std::shared_ptr<FxEntity> e2,
                            const FxVec2f& anchor, bool anchor_is_local = true);
    void evaluate(float& C, FxVec2f& g1, FxVec2f& g2,
                  float& gth1, float& gth2, bool& active) const override;
};

// Linear limit constraint that restricts the projection of separation between two entities onto a specified axis
class FxSeparationConstraint : public FxConstraint {
private:
    FxVec2f m_axis;         // Axis direction (normalized)
    bool m_axis_is_local;   // Whether axis is local to entity1 or in world coordinates
public:
    float lower_limit = 0;        // Lower limit for projection
    float upper_limit = 10;       // Upper limit for projection
    float slop = 0.0001f;   // Tolerance zone around limits
    bool enabled = true;    // Whether this constraint is active
    FxSeparationConstraint(std::shared_ptr<FxEntity> e1, std::shared_ptr<FxEntity> e2,
                           const FxVec2f& axis, bool axis_is_local = true);
    void evaluate(float& C, FxVec2f& g1, FxVec2f& g2,
                  float& gth1, float& gth2, bool& active) const override;
};

// Constraint that forces motion along a specified axis
class FxMotionAlongAxisConstraint : public FxConstraint {
private:
    FxVec2f m_axis;         // Axis direction (normalized)
    bool m_axis_is_local;   // Whether axis is local to entity1 or in world coordinates
    float m_initial_projection; // Initial perpendicular distance projection
public:
    bool enabled = true;    // Whether this constraint is active
    FxMotionAlongAxisConstraint(std::shared_ptr<FxEntity> e1, std::shared_ptr<FxEntity> e2,
                               const FxVec2f& axis, bool axis_is_local = true);
    void evaluate(float& C, FxVec2f& g1, FxVec2f& g2,
                  float& gth1, float& gth2, bool& active) const override;
};


namespace FxSolver {
    // AABB overlap check methods
    bool aabb_overlap_check(const FxEntity& entity1, const FxEntity& entity2);
    bool aabb_overlap_check(const std::shared_ptr<FxEntity>& entity1, const std::shared_ptr<FxEntity>& entity2);

    // Main collision detection method using SAT
    const FxContact collision_check(const FxEntity& entity1, const FxEntity& entity2);
    const FxContact collision_check(const std::shared_ptr<FxEntity>& entity1, const std::shared_ptr<FxEntity>& entity2);

    // Main collision resolution method 
    void resolve_penetration(const FxContact& contact, float dt = 0.016f);
    // Velocity-level solver: restitution and dynamic friction impulses
    void resolve_velocities(const FxContact& contact);

} // namespace FxSolver