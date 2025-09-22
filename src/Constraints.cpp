#include "Fx2D/Solver.h"
#include "Fx2D/Entity.h"
#include <iostream>
#include <unordered_set>
#include <algorithm>
#include <cmath>

// FxConstraint implementation
void FxConstraint::resolve(float dt) {
    if (!entity1 || !entity2) return;

    float C = 0;            // Constraint violation value
    auto g1 = FxVec2f(0.0f, 0.0f); 
    auto g2 = FxVec2f(0.0f, 0.0f);   // Position gradients for entity1 and entity2
    float gth1 = 0, gth2 = 0;        // Angular gradients for entity1 and entity2
    bool active = false;             // Whether the constraint is active/should be solved

    // Evaluate constraint function and gradients
    evaluate(C, g1, g2, gth1, gth2, active);
    if (!active) return;

    // Get inverse mass and inertia properties
    const float w1 = entity1->inv_mass();
    const float w2 = entity2->inv_mass();
    const float I1 = entity1->inv_inertia();
    const float I2 = entity2->inv_inertia();

    // Calculate compliance term (alpha = compliance / dt^2)
    const float alpha = (compliance > 0.0f) ? (compliance / (dt * dt)) : 0.0f;
    // Calculate denominator for XPBD solver (includes compliance for softness)
    float denom = w1 * g1.dot(g1) + w2 * g2.dot(g2)
                + I1 * gth1 * gth1 + I2 * gth2 * gth2 + alpha;
    if (denom <= 1e-12f) return;
    // Calculate Lagrange multiplier delta
    const float dLambda = -C / denom;

    // Apply corrections directly to entity poses
    entity1->pose.xy()    += w1 * dLambda * g1;
    entity2->pose.xy()    += w2 * dLambda * g2;
    entity1->pose.theta() += I1 * dLambda * gth1;
    entity2->pose.theta() += I2 * dLambda * gth2;
}

// FxAngleLockConstraint constructors
FxAngleLockConstraint::FxAngleLockConstraint(const std::shared_ptr<FxEntity>& e1, const std::shared_ptr<FxEntity>& e2, float tgt) {
    entity1 = e1; entity2 = e2; target = tgt;
    name = e1->get_name() + "_" + e2->get_name() + "_AngleLock";
}

// FxAngleLockConstraint implementation
void FxAngleLockConstraint::evaluate(float& C, FxVec2f& g1, FxVec2f& g2,
                                      float& gth1, float& gth2, bool& active) const {
    if (!enabled) { return; }
    // Calculate relative angle error
    C = FxAngleWrap(entity2->pose.theta() - entity1->pose.theta() - target);
    // Only angular gradients are non-zero
    g1 = FxVec2f(0.0f, 0.0f); g2 = FxVec2f(0.0f, 0.0f);
    gth1 = -1.0f; gth2 = 1.0f; active = true;
}

// FxAngularLimitConstraint constructors
FxAngularLimitConstraint::FxAngularLimitConstraint(const std::shared_ptr<FxEntity>& e1, const std::shared_ptr<FxEntity>& e2) {
    entity1 = e1; entity2 = e2;
    name = e1->get_name() + "_" + e2->get_name() + "_AngleLmt";
}

// FxAngularLimitConstraint implementation
void FxAngularLimitConstraint::evaluate(float& C, FxVec2f& g1, FxVec2f& g2,
                                         float& gth1, float& gth2, bool& active) const {
    // Early exit if constraint is disabled
    if (!enabled) { return; }
    // std::cout<<name<<std::endl;
    // Calculate relative angle between entities (wrapped to [-π, π])
    const float rel = FxAngleWrap(entity2->pose.theta() - entity1->pose.theta());
    // Check if relative angle violates lower or upper bounds
    const bool lowHit = rel < (lower_limit - slop);
    const bool upHit  = rel > (upper_limit + slop);
    // Exit if angle is within acceptable range
    if (!lowHit && !upHit) { return; }
    // Determine which bound was violated and use as constraint target
    const float bound = lowHit ? (lower_limit - slop) : (upper_limit + slop);
    // Set constraint violation value and gradients
    C = rel - bound;
    g1 = FxVec2f(0.0f, 0.0f); g2 = FxVec2f(0.0f, 0.0f);
    gth1 = 1.0f; gth2 = -1.0f; active = true;
}

// FxAnchorConstraint constructors
FxAnchorConstraint::FxAnchorConstraint(const std::shared_ptr<FxEntity>& e1, const std::shared_ptr<FxEntity>& e2,
                                        const FxVec2f& anchor, bool anchor_is_local) {
    entity1 = e1; entity2 = e2;
    if(!anchor_is_local){
        m_anchor1 = e1->to_entity_frame(anchor);
        m_anchor2 = e2->to_entity_frame(anchor);
    } else {
        m_anchor1 = anchor;
        const auto anc2 = e1->to_world_frame(anchor); 
        m_anchor2 = e2->to_entity_frame(anc2);
    }
    name = e1->get_name() + "_" + e2->get_name() + "_Anchor";
}

// FxAnchorConstraint implementation
void FxAnchorConstraint::evaluate(float& C, FxVec2f& g1, FxVec2f& g2,
                                  float& gth1, float& gth2, bool& active) const {
    if (!enabled) { return; }
    // Transform local anchor points to world coordinates using each entity's current pose
    const FxVec2f a1 = entity1->to_world_frame(m_anchor1);
    const FxVec2f a2 = entity2->to_world_frame(m_anchor2);
    // Calculate the separation vector between the two world anchor points
    const FxVec2f d  = a1 - a2;
    // Constraint violation C: projection of separation onto the constraint direction
    // C = 0 means anchors are aligned along the constraint axis
    FxVec2f m_dirWorld = d.normalized();
    C = m_dirWorld.dot(d);
    // Linear gradients: how constraint changes with respect to entity positions
    // Positive movement of entity1 in direction increases C and same for entity2
    g1 = m_dirWorld;  g2 = -m_dirWorld;
    // Angular gradients: how constraint changes with respect to entity rotations
    // Calculate how anchor point moves perpendicular to radius when entity rotates
    const FxVec2f da1 = m_anchor1.rotate_rad(entity1->pose.theta()).perp();
    const FxVec2f da2 = m_anchor2.rotate_rad(entity2->pose.theta()).perp();
    // Project the angular motion onto the constraint direction
    gth1 = m_dirWorld.dot(da1);   // Effect of entity1 rotation on constraint
    gth2 = -m_dirWorld.dot(da2);  // Effect of entity2 rotation on constraint (opposite)
    active = true;
}

// FxSeparationConstraint constructors
FxSeparationConstraint::FxSeparationConstraint(const std::shared_ptr<FxEntity>& e1, const std::shared_ptr<FxEntity>& e2,
                                                  const FxVec2f& axis, bool axis_is_local) {
    entity1 = e1; entity2 = e2;
    m_axis = axis.normalized();
    m_axis_is_local = axis_is_local;
    // Store the initial distance projection
    FxVec2f axw = m_axis;
    if (m_axis_is_local) {
        axw = m_axis.rotate_rad(entity1->pose.theta());
    }
    const FxVec2f a1 = entity1->pose.xy();
    const FxVec2f a2 = entity2->pose.xy();
    const FxVec2f d = a2 - a1;
    m_initial_projection = axw.dot(d);
    name = e1->get_name() + "_" + e2->get_name() + "_LinearLmt";
}

// FxSeparationConstraint implementation
void FxSeparationConstraint::evaluate(float& C, FxVec2f& g1, FxVec2f& g2,
                                        float& gth1, float& gth2, bool& active) const {
            // std::cout << "FxSeparationConstraint initialized: " << name << std::endl;
    if (!enabled) { return; }
    // Transform axis to world coordinates based on is_local flag
    FxVec2f axw = m_axis;
    if (m_axis_is_local) { // If axis is local to entity1, rotate it by entity1's current orientation
        axw = (axw.rotate_rad(entity1->pose.theta()));
    }
    // Transform anchor points to world coordinates using existing methods
    const FxVec2f a1 = entity1->pose.xy();
    const FxVec2f a2 = entity2->pose.xy();
    const auto d = a2 - a1;
    // Calculate projection of separation vector onto world axis
    const float current_projection = axw.dot(d);
    const float s = current_projection - m_initial_projection;
    // Check if projection violates lower or upper bounds
    const bool lowHit = s < (lower_limit - slop);
    const bool upHit  = s > (upper_limit + slop);
    // Exit if projection is within acceptable range
    if (!lowHit && !upHit) { return; }
    // Determine which bound was violated and calculate constraint violation
    const float bound = lowHit ? (lower_limit - slop) : (upper_limit + slop);
    C = s - bound;
    // Linear gradients
    g1 = -axw; g2 = axw;
    // Calculate angular gradient contributions
    if (m_axis_is_local){
        gth1 = -(axw.perp()).dot(d);
    }
    active = true; gth2 = 0.0f;
}


// FxMotionAlongAxisConstraint constructors
FxMotionAlongAxisConstraint::FxMotionAlongAxisConstraint(const std::shared_ptr<FxEntity>& e1, const std::shared_ptr<FxEntity>& e2,
                                                         const FxVec2f& axis, bool axis_is_local) {
    entity1 = e1; entity2 = e2;
    m_axis = axis.normalized();
    m_axis_is_local = axis_is_local;
    
    // Store the initial distance projection
    FxVec2f axw = m_axis;
    if (m_axis_is_local) {
        axw = m_axis.rotate_rad(entity1->pose.theta());
    }
    const FxVec2f daxw = axw.perp();
    const FxVec2f d = entity1->pose.xy() - entity2->pose.xy();
    m_initial_projection = daxw.dot(d);
    
    name = e1->get_name() + "_" + e2->get_name() + "_MotionAlongAxis";
}

// FxMotionAlongAxisConstraint implementation
void FxMotionAlongAxisConstraint::evaluate(float& C, FxVec2f& g1, FxVec2f& g2,
                                           float& gth1, float& gth2, bool& active) const {
    if (!enabled) { return; }
    // Transform axis to world coordinates based on is_local flag
    FxVec2f axw = m_axis;
    if (m_axis_is_local) {
        axw = m_axis.rotate_rad(entity1->pose.theta());
    }
    const FxVec2f daxw = axw.perp();
    // Current separation vector
    const FxVec2f d = (entity1->pose.xy() - entity2->pose.xy());
    // Constraint violation - current perpendicular distance should equal initial distance
    C = daxw.dot(d) - m_initial_projection;
    // Linear gradients
    g1 = daxw; g2 = -daxw;
    // Angular gradient contribution
    if (m_axis_is_local) {
        gth1 = -axw.dot(d);
    }
    active = true; gth2 = 0.0f;
}


namespace FxSolver {
    void resolve_penetration(const FxContact& contact, float dt) {
        // Early exits for invalid contacts
        if (!contact.is_valid()) return;
        if (!contact.entity1 || !contact.entity2) return;
        // Allow small penetration - only resolve if depth exceeds threshold
        const float penetration_tolerance = 2e-4f; // tolerance
        if (contact.penetration_depth <= penetration_tolerance) return;
      
        // Get entity references
        FxEntity& A = *contact.entity1;
        FxEntity& B = *contact.entity2;
        FxVec2f n = (contact.normal); 

        // Mass and inertia properties
        const float wA = A.inv_mass(), wB = B.inv_mass();
        const float IA = A.inv_inertia(), IB = B.inv_inertia();
        
        // Handle each contact point
        for (size_t i = 0; i < contact.count; i++) {
            // Contact point relative to each entity's center
            FxVec2f rA = contact.position[i] - A.pose.xy();
            FxVec2f rB = contact.position[i] - B.pose.xy();
            
            // --- Position Correction (Penetration Resolution) ---
            float ra_n = rA.cross(n), rb_n = rB.cross(n); 
            float K_n = wA + wB + IA * ra_n * ra_n + IB * rb_n * rb_n;
            float compliance = 1e-7f; // tweak: 0 = rigid, higher = softer
            K_n = K_n + compliance / (dt * dt);
            
            if (K_n > 1e-8f) { 
                float correction_depth = contact.penetration_depth/contact.count;
                float lambdaP = correction_depth / K_n; 
                FxVec2f dP = n * lambdaP; 
                A.pose.xy() -= wA * dP; 
                B.pose.xy() += wB * dP; 
                A.prev_pose.xy() -= wA * dP; 
                B.prev_pose.xy() += wB * dP; 
                
                // Apply angular corrections
                A.pose.theta() = FxAngleWrap(A.pose.theta() - IA * lambdaP * ra_n);
                B.pose.theta() = FxAngleWrap(B.pose.theta() + IB * lambdaP * rb_n);
                A.prev_pose.theta() = FxAngleWrap(A.prev_pose.theta() - IA * lambdaP * ra_n);
                B.prev_pose.theta() = FxAngleWrap(B.prev_pose.theta() + IB * lambdaP * rb_n);
            }
        }
    }

    // Post-constraint velocity impulses for restitution and dynamic friction
    void resolve_velocities(const FxContact& contact) {
        if (!contact.is_valid() || contact.penetration_depth <= 1e-4f) return;
        if (!contact.entity1 || !contact.entity2) return;

        // Get entity references
        FxEntity& A = *contact.entity1;
        FxEntity& B = *contact.entity2;
        FxVec2f n = (contact.normal); 
        FxVec2f t(-n.y(), n.x());  // fixed tangent (no normalize of vRel_t)

        // Mass, inertia and other properties
        const float wA = A.inv_mass(), wB = B.inv_mass();
        const float IA = A.inv_inertia(), IB = B.inv_inertia();
        const float e  = std::clamp(std::min(A.elasticity, B.elasticity), 0.0f, 1.0f);
        const float mu_s = std::clamp(std::max(A.static_friction,  B.static_friction), 0.0f, 10.0f);
        const float mu_k = std::clamp(std::max(A.dynamic_friction, B.dynamic_friction), 0.0f, 10.0f);
    
        // Accumulate normal impulses for shared friction cone
        float jn_sum = 0.0f;

        auto rA = contact.position - A.pose.get_xy();
        auto rB = contact.position - B.pose.get_xy();

        // Iteratively resolve normal impulses
        for (int iter = 0; iter < 2; iter++) {
            for (size_t i = 0; i < contact.count; i++) {
                size_t k = (i + iter)%contact.count;
                auto vA = A.velocity_at_local_point(rA[k]);
                auto vB = B.velocity_at_local_point(rB[k]);

                // Relative velocity and its normal component
                float vn = (vB - vA).dot(n); 
                
                // --- Velocity Correction (Normal Impulse) ---
                float ra_n = rA[k].cross(n), rb_n = rB[k].cross(n); 
                float K_n = wA + wB + IA * ra_n * ra_n + IB * rb_n * rb_n;
                
                float jn = 0.0f;
                float bias = (vn < -1e-3f) ? e : 0.0f;
                if (K_n > 1e-6f) {
                    jn = -(1.0f + bias) * vn / K_n;
                    if (jn > 0.0f) {
                        FxVec2f Pn = n * jn;
                        A.velocity.xy() -= wA * Pn;
                        B.velocity.xy() += wB * Pn;
                        A.velocity.theta() -= IA * jn * ra_n;
                        B.velocity.theta() += IB * jn * rb_n;
                        jn_sum += jn;
                    }
                }
            }
        }

        // Single pass friction resolution using accumulated normal impulses
        for (size_t i = 0; i < contact.count; i++) {
            FxVec2f vA = A.velocity_at_local_point(rA[i]);
            FxVec2f vB = B.velocity_at_local_point(rB[i]);
            float vt   = (vB - vA).dot(t);

            float ra_t = rA[i].cross(t), rb_t = rB[i].cross(t);
            float Kt   = wA + wB + IA*ra_t*ra_t + IB*rb_t*rb_t;
            if (Kt <= 1e-8f) continue;

            float jt = -vt / Kt;

            float max_static = mu_s * std::max(0.f, jn_sum);
            if (std::fabs(jt) > max_static) {
                jt = (jt >= 0.f ? 1.f : -1.f) * (mu_k * std::max(0.f, jn_sum));
            }

            FxVec2f Pt = t * jt;
            A.velocity.xy() -= wA * Pt;  
            B.velocity.xy() += wB * Pt;
            A.velocity.theta() -= IA * jt * ra_t;
            B.velocity.theta() += IB * jt * rb_t;
        }
    }
}