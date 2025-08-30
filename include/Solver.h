#pragma once

#include <vector>
#include <memory>
#include <algorithm>

#include "MathTypes.h"

// Forward declaration
class FxEntity;

struct FxContact {
    bool is_valid = false;                    // True if contact is valid
    FxVec2f position {0.0f, 0.0f};            // Contact position in world coordinates
    FxVec2f normal {0.0f, 0.0f};             // Contact normal (unit vector)
    float penetration_depth = FxInfinityf;    // Penetration depth (positive if overlapping)
    
    // Entity pointers (null if invalid contact)
    std::shared_ptr<FxEntity> entity1 = nullptr;  // First entity in collision
    std::shared_ptr<FxEntity> entity2 = nullptr;  // Second entity in collision
    
    // Constructor overloads
    FxContact() = default;
    FxContact(bool valid)
        : is_valid(valid), position(0.0f, 0.0f), normal(0.0f, 0.0f), penetration_depth(FxInfinityf) {}
    FxContact(bool valid, const FxVec2f& pos, const FxVec2f& norm, float depth)
        : is_valid(valid), position(pos), normal(norm), penetration_depth(depth) {}
    FxContact(bool valid, const FxVec2f& pos, const FxVec2f& norm, float depth, 
              std::shared_ptr<FxEntity> ent1, std::shared_ptr<FxEntity> ent2)
        : is_valid(valid), position(pos), normal(norm), penetration_depth(depth), entity1(ent1), entity2(ent2) {}
};

// Position-based constraint base class for XPBD solver
class FxConstraint {
public:
    float compliance = 0.0f;  // Inverse of stiffness (alpha in XPBD)
    std::shared_ptr<FxEntity> entity1; // First constrained entity
    std::shared_ptr<FxEntity> entity2; // Second constrained entity

    // Virtual functions that derived constraints must implement
    virtual float evaluate() const = 0;  // Constraint function C(x)
    virtual FxVec2f gradientEntity1() const = 0;  // Gradient w.r.t. entity1 position
    virtual FxVec2f gradientEntity2() const = 0;  // Gradient w.r.t. entity2 position
    virtual float gradientThetaEntity1() const = 0;  // Gradient w.r.t. entity1 rotation
    virtual float gradientThetaEntity2() const = 0;  // Gradient w.r.t. entity2 rotation

    // Set stiffness (converts to compliance internally)
    void set_stiffness(float k) {
        if (k <= 0.0f) return;
        compliance = 1.0f / k;
    }

    // Solve one iteration of the constraint
    void resolve(float dt) {
        if (!entity1 || !entity2) return;

        float C = evaluate();
        FxVec2f grad1 = gradientEntity1();
        FxVec2f grad2 = gradientEntity2();
        float gradTheta1 = gradientThetaEntity1();
        float gradTheta2 = gradientThetaEntity2();

        // Calculate denominator (includes compliance term and angular terms)
        float denominator = entity1->inv_mass() * grad1.dot(grad1) + 
                            entity2->inv_mass() * grad2.dot(grad2) + 
                            entity1->inv_inertia() * gradTheta1 * gradTheta1 +
                            entity2->inv_inertia() * gradTheta2 * gradTheta2 +
                            compliance/(dt*dt);
        
        if (denominator == 0.0f) return;

        // Calculate position corrections directly
        float delta_x = -C/denominator;

        // Apply position corrections
        entity1->pose.xy() += entity1->inv_mass() * delta_x * grad1;
        entity2->pose.xy() += entity2->inv_mass() * delta_x * grad2;
        
        // Apply orientation corrections
        entity1->pose.theta() += entity1->inv_inertia() * delta_x * gradTheta1;
        entity2->pose.theta() += entity2->inv_inertia() * delta_x * gradTheta2;
    }

    virtual ~FxConstraint() = default;
};

// Revolute joint constraint - constrains two bodies to rotate around a fixed point
class FxRevoluteJointConstraint : public FxConstraint {
private:
    FxVec2f anchor_local1;  // Anchor point in entity1's local space
    FxVec2f anchor_local2;  // Anchor point in entity2's local space

public:
    FxRevoluteJointConstraint(std::shared_ptr<FxEntity> ent1, std::shared_ptr<FxEntity> ent2,
                              const FxVec2f& anchor1, const FxVec2f& anchor2)
        : anchor_local1(anchor1), anchor_local2(anchor2) {
        entity1 = ent1;
        entity2 = ent2;
    }

    // Get world positions of anchor points
    FxVec2f getWorldAnchor1() const {
        float cos_theta = std::cos(entity1->pose.theta());
        float sin_theta = std::sin(entity1->pose.theta());
        return entity1->pose.xy() + FxVec2f(
            cos_theta * anchor_local1.x() - sin_theta * anchor_local1.y(),
            sin_theta * anchor_local1.x() + cos_theta * anchor_local1.y()
        );
    }

    FxVec2f getWorldAnchor2() const {
        float cos_theta = std::cos(entity2->pose.theta());
        float sin_theta = std::sin(entity2->pose.theta());
        return entity2->pose.xy() + FxVec2f(
            cos_theta * anchor_local2.x() - sin_theta * anchor_local2.y(),
            sin_theta * anchor_local2.x() + cos_theta * anchor_local2.y()
        );
    }

    // Constraint function: distance between world anchor points should be zero
    float evaluate() const override {
        FxVec2f diff = getWorldAnchor1() - getWorldAnchor2();
        return diff.magnitude();
    }

    // Gradient w.r.t. entity1 position
    FxVec2f gradientEntity1() const override {
        FxVec2f diff = getWorldAnchor1() - getWorldAnchor2();
        float mag = diff.magnitude();
        return (mag > 1e-8f) ? diff / mag : FxVec2f(0.0f, 0.0f);
    }

    // Gradient w.r.t. entity2 position  
    FxVec2f gradientEntity2() const override {
        return -gradientEntity1();
    }

    // Gradient w.r.t. entity1 rotation
    float gradientThetaEntity1() const override {
        FxVec2f diff = getWorldAnchor1() - getWorldAnchor2();
        float mag = diff.magnitude();
        if (mag <= 1e-8f) return 0.0f;
        
        // Derivative of world anchor1 w.r.t. theta1
        FxVec2f anchor_perp(-anchor_local1.y(), anchor_local1.x());
        float cos_theta = std::cos(entity1->pose.theta());
        float sin_theta = std::sin(entity1->pose.theta());
        FxVec2f d_anchor1_dtheta(
            -sin_theta * anchor_local1.x() - cos_theta * anchor_local1.y(),
            cos_theta * anchor_local1.x() - sin_theta * anchor_local1.y()
        );
        
        return diff.dot(d_anchor1_dtheta) / mag;
    }

    // Gradient w.r.t. entity2 rotation
    float gradientThetaEntity2() const override {
        FxVec2f diff = getWorldAnchor1() - getWorldAnchor2();
        float mag = diff.magnitude();
        if (mag <= 1e-8f) return 0.0f;
        
        // Derivative of world anchor2 w.r.t. theta2
        float cos_theta = std::cos(entity2->pose.theta());
        float sin_theta = std::sin(entity2->pose.theta());
        FxVec2f d_anchor2_dtheta(
            -sin_theta * anchor_local2.x() - cos_theta * anchor_local2.y(),
            cos_theta * anchor_local2.x() - sin_theta * anchor_local2.y()
        );
        
        return -diff.dot(d_anchor2_dtheta) / mag;
    }
};

namespace FxSolver {
    // AABB overlap check methods
    bool aabb_overlap_check(const FxEntity& entity1, const FxEntity& entity2);
    bool aabb_overlap_check(const std::shared_ptr<FxEntity>& entity1, const std::shared_ptr<FxEntity>& entity2);

    // Main collision detection method using SAT
    const FxContact collision_check(const FxEntity& entity1, const FxEntity& entity2);
    const FxContact collision_check(const std::shared_ptr<FxEntity>& entity1, const std::shared_ptr<FxEntity>& entity2);

    // XPBD Constraint Solver
    void resolve_constraints_xpbd(FxConstraint& contraints, float dt = 0.016f);
    // Main collision resolution method 
    void resolve_penetration(const FxContact& contact);
    // Velocity-level solver: restitution and dynamic friction impulses
    void resolve_velocities(const FxContact& contacts);

} // namespace FxSolver