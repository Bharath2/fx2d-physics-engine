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

    // Set stiffness (converts to compliance internally)
    void set_stiffness(float k) {
        if (k <= 0.0f) return;
        compliance = 1.0f / k;
    }

    // // Solve one iteration of the constraint
    // void resolve(float dt) {
    //     if (!entity1 || !entity2) return;

    //     // float C = evaluate();
    //     // FxVec2f grad1 = gradientEntity1();
    //     // FxVec2f grad2 = gradientEntity2();
    //     // float gradTheta1 = gradientThetaEntity1();
    //     // float gradTheta2 = gradientThetaEntity2();

    //     // // Calculate denominator (includes compliance term and angular terms)
    //     // float denominator = entity1->inv_mass() * grad1.dot(grad1) + 
    //     //                     entity2->inv_mass() * grad2.dot(grad2) + 
    //     //                     entity1->inv_inertia() * gradTheta1 * gradTheta1 +
    //     //                     entity2->inv_inertia() * gradTheta2 * gradTheta2 +
    //     //                     compliance/(dt*dt);
        
    //     // if (denominator == 0.0f) return;

    //     // // Calculate position corrections directly
    //     // float delta_x = -C/denominator;

    //     // // Apply position corrections
    //     // entity1->pose.xy() += entity1->inv_mass() * delta_x * grad1;
    //     // entity2->pose.xy() += entity2->inv_mass() * delta_x * grad2;
        
    //     // // Apply orientation corrections
    //     // entity1->pose.theta() += entity1->inv_inertia() * delta_x * gradTheta1;
    //     // entity2->pose.theta() += entity2->inv_inertia() * delta_x * gradTheta2;
    // }

    // virtual ~Constraint() = default;
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