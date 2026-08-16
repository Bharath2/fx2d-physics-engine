#include "Fx2D/Entity.h"
#include "Fx2D/Solver.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_set>

// Minimum closing speed (m/s) to trigger restitution; prevents micro-bounce during stacking
static constexpr float kRestitutionSlop = 2e-2f;

// Entity name accessor methods for FxConstraint
std::string FxConstraint::get_entity1_name() const {
    return entity1 ? entity1->get_name() : "";
}

std::string FxConstraint::get_entity2_name() const {
    return entity2 ? entity2->get_name() : "";
}

// FxConstraint implementation
void FxConstraint::resolve(double dt) {
    if (!entity1 || !entity2) return;

    float C = 0; // Constraint violation value
    auto g1 = FxVec2f(0.0f, 0.0f);
    auto g2 = FxVec2f(0.0f, 0.0f); // Position gradients for entity1 and entity2
    float gth1 = 0, gth2 = 0; // Angular gradients for entity1 and entity2
    bool active = false; // Whether the constraint is active/should be solved

    // Evaluate constraint function and gradients
    evaluate(C, g1, g2, gth1, gth2, active);
    if (!active) return;

    // Get inverse mass and inertia properties
    const float w1 = entity1->inv_mass();
    const float w2 = entity2->inv_mass();
    const float I1 = entity1->inv_inertia();
    const float I2 = entity2->inv_inertia();

    // Calculate compliance term (alpha = compliance / dt^2)
    const float alpha = std::max(static_cast<float>(compliance / (dt * dt)), 0.0f);
    // Calculate denominator for XPBD solver (includes compliance for softness)
    float denom = w1 * g1.dot(g1) + w2 * g2.dot(g2) + I1 * gth1 * gth1 + I2 * gth2 * gth2 + alpha;
    if (denom <= 1e-12f) return;
    // Calculate Lagrange multiplier delta
    const float dLambda = -C / denom;

    // Mixed precision, not a direct float add: a correction below one ulp of the coordinate
    // would round away entirely.
    const FxVec2f dxy1 = w1 * dLambda * g1;
    const FxVec2f dxy2 = w2 * dLambda * g2;
    entity1->apply_pose_correction(FxVec3f{dxy1.x(), dxy1.y(), I1 * dLambda * gth1});
    entity2->apply_pose_correction(FxVec3f{dxy2.x(), dxy2.y(), I2 * dLambda * gth2});
}

// FxAngleLockConstraint constructors
FxAngleLockConstraint::FxAngleLockConstraint(const std::shared_ptr<FxEntity>& e1,
                                             const std::shared_ptr<FxEntity>& e2, float tgt) {
    entity1 = e1;
    entity2 = e2;
    target = tgt;
    m_name = e1->get_name() + "_" + e2->get_name() + "_AngleLock";
}

// FxAngleLockConstraint implementation
void FxAngleLockConstraint::evaluate(float& C, FxVec2f& g1, FxVec2f& g2, float& gth1, float& gth2,
                                     bool& active) const {
    if (!enabled) {
        return;
    }
    // Calculate relative angle error
    C = FxAngleWrap(entity2->pose.theta() - entity1->pose.theta() - target);
    // Only angular gradients are non-zero
    g1 = FxVec2f(0.0f, 0.0f);
    g2 = FxVec2f(0.0f, 0.0f);
    gth1 = -1.0f;
    gth2 = 1.0f;
    active = true;
}

// FxAngularLimitConstraint constructors
FxAngularLimitConstraint::FxAngularLimitConstraint(const std::shared_ptr<FxEntity>& e1,
                                                   const std::shared_ptr<FxEntity>& e2) {
    entity1 = e1;
    entity2 = e2;
    m_name = e1->get_name() + "_" + e2->get_name() + "_AngleLmt";
}

// FxAngularLimitConstraint implementation
void FxAngularLimitConstraint::evaluate(float& C, FxVec2f& g1, FxVec2f& g2, float& gth1,
                                        float& gth2, bool& active) const {
    // Early exit if constraint is disabled
    if (!enabled) {
        return;
    }
    // std::cout<<m_name<<std::endl;
    // Calculate relative angle between entities (wrapped to [-π, π])
    const float rel = FxAngleWrap(entity2->pose.theta() - entity1->pose.theta());
    // Check if relative angle violates lower or upper bounds
    const bool lowHit = rel < (lower_limit - slop);
    const bool upHit = rel > (upper_limit + slop);
    // Exit if angle is within acceptable range
    if (!lowHit && !upHit) {
        return;
    }
    // Determine which bound was violated and use as constraint target
    const float bound = lowHit ? (lower_limit - slop) : (upper_limit + slop);
    // Set constraint violation value and gradients
    C = rel - bound;
    g1 = FxVec2f(0.0f, 0.0f);
    g2 = FxVec2f(0.0f, 0.0f);
    gth1 = 1.0f;
    gth2 = -1.0f;
    active = true;
}

// FxAnchorConstraint constructors
FxAnchorConstraint::FxAnchorConstraint(const std::shared_ptr<FxEntity>& e1,
                                       const std::shared_ptr<FxEntity>& e2, const FxVec2f& anchor,
                                       bool anchor_is_local) {
    entity1 = e1;
    entity2 = e2;
    if (!anchor_is_local) {
        m_anchor1 = e1->to_entity_frame(anchor);
        m_anchor2 = e2->to_entity_frame(anchor);
    } else {
        m_anchor1 = anchor;
        const auto anc2 = e1->to_world_frame(anchor);
        m_anchor2 = e2->to_entity_frame(anc2);
    }
    m_name = e1->get_name() + "_" + e2->get_name() + "_Anchor";
}

// FxAnchorConstraint implementation
void FxAnchorConstraint::evaluate(float& C, FxVec2f& g1, FxVec2f& g2, float& gth1, float& gth2,
                                  bool& active) const {
    if (!enabled) {
        return;
    }
    // Transform local anchor points to world coordinates using each entity's current pose
    const FxVec2f a1 = entity1->to_world_frame(m_anchor1);
    const FxVec2f a2 = entity2->to_world_frame(m_anchor2);
    // Calculate the separation vector between the two world anchor points
    const FxVec2f d = a1 - a2;
    // Guard: if anchors coincide the constraint is already satisfied
    if (d.squaredNorm() < 1e-12f) {
        active = false;
        return;
    }
    // Constraint violation C: projection of separation onto the constraint direction
    // C = 0 means anchors are aligned along the constraint axis
    FxVec2f m_dirWorld = d.normalized();
    C = m_dirWorld.dot(d);
    // Linear gradients: how constraint changes with respect to entity positions
    // Positive movement of entity1 in direction increases C and same for entity2
    g1 = m_dirWorld;
    g2 = -m_dirWorld;
    // Angular gradients: how constraint changes with respect to entity rotations
    // Calculate how anchor point moves perpendicular to radius when entity rotates
    const FxVec2f da1 = m_anchor1.rotate_rad(entity1->pose.theta()).perp();
    const FxVec2f da2 = m_anchor2.rotate_rad(entity2->pose.theta()).perp();
    // Project the angular motion onto the constraint direction
    gth1 = m_dirWorld.dot(da1); // Effect of entity1 rotation on constraint
    gth2 = -m_dirWorld.dot(da2); // Effect of entity2 rotation on constraint (opposite)
    active = true;
}

// FxSeparationConstraint constructors
FxSeparationConstraint::FxSeparationConstraint(const std::shared_ptr<FxEntity>& e1,
                                               const std::shared_ptr<FxEntity>& e2,
                                               const FxVec2f& axis, bool axis_is_local) :
    m_axis(axis.normalized()), m_axis_is_local(axis_is_local) {
    entity1 = e1;
    entity2 = e2;
    // Store the initial distance projection
    FxVec2f axw = m_axis;
    if (m_axis_is_local) {
        axw = m_axis.rotate_rad(entity1->pose.theta());
    }
    const FxVec2f a1 = entity1->pose.xy();
    const FxVec2f a2 = entity2->pose.xy();
    const FxVec2f d = a2 - a1;
    m_initial_projection = axw.dot(d);
    m_name = e1->get_name() + "_" + e2->get_name() + "_LinearLmt";
}

// FxSeparationConstraint implementation
void FxSeparationConstraint::evaluate(float& C, FxVec2f& g1, FxVec2f& g2, float& gth1, float& gth2,
                                      bool& active) const {
    // std::cout << "FxSeparationConstraint initialized: " << m_name << std::endl;
    if (!enabled) {
        return;
    }
    // Transform axis to world coordinates based on is_local flag
    FxVec2f axw = m_axis;
    if (m_axis_is_local) { // If axis is local to entity1, rotate it by entity1's current
                           // orientation
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
    const bool upHit = s > (upper_limit + slop);
    // Exit if projection is within acceptable range
    if (!lowHit && !upHit) {
        return;
    }
    // Determine which bound was violated and calculate constraint violation
    const float bound = lowHit ? (lower_limit - slop) : (upper_limit + slop);
    C = s - bound;
    // Linear gradients
    g1 = -axw;
    g2 = axw;
    // Calculate angular gradient contributions
    if (m_axis_is_local) {
        gth1 = -(axw.perp()).dot(d);
    }
    active = true;
    gth2 = 0.0f;
}

// FxMotionAlongAxisConstraint constructors
FxMotionAlongAxisConstraint::FxMotionAlongAxisConstraint(const std::shared_ptr<FxEntity>& e1,
                                                         const std::shared_ptr<FxEntity>& e2,
                                                         const FxVec2f& axis, bool axis_is_local) :
    m_axis(axis.normalized()), m_axis_is_local(axis_is_local) {
    entity1 = e1;
    entity2 = e2;

    // Store the initial distance projection
    FxVec2f axw = m_axis;
    if (m_axis_is_local) {
        axw = m_axis.rotate_rad(entity1->pose.theta());
    }
    const FxVec2f daxw = axw.perp();
    const FxVec2f d = entity1->pose.xy() - entity2->pose.xy();
    m_initial_projection = daxw.dot(d);

    m_name = e1->get_name() + "_" + e2->get_name() + "_MotionAlongAxis";
}

// FxMotionAlongAxisConstraint implementation
void FxMotionAlongAxisConstraint::evaluate(float& C, FxVec2f& g1, FxVec2f& g2, float& gth1,
                                           float& gth2, bool& active) const {
    if (!enabled) {
        return;
    }
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
    g1 = daxw;
    g2 = -daxw;
    // Angular gradient contribution
    if (m_axis_is_local) {
        gth1 = -axw.dot(d);
    }
    active = true;
    gth2 = 0.0f;
}

namespace FxSolver {
// Sleeping bodies are immovable in contact solves (integration is paused).
static float eff_inv_mass(const FxEntity& e) {
    return e.is_sleeping() ? 0.0f : e.inv_mass();
}
static float eff_inv_inertia(const FxEntity& e) {
    return e.is_sleeping() ? 0.0f : e.inv_inertia();
}

void resolve_penetration(const FxContact& contact, double dt) {
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
    const float wA = eff_inv_mass(A), wB = eff_inv_mass(B);
    const float IA = eff_inv_inertia(A), IB = eff_inv_inertia(B);

    // Resolve each contact point individually
    for (size_t i = 0; i < contact.count; i++) {
        // Get contact point for this iteration
        FxVec2f contact_point = contact.position[i];

        // Contact point relative to each entity's center
        FxVec2f rA = contact_point - A.pose.xy();
        FxVec2f rB = contact_point - B.pose.xy();

        // --- Position Correction (Penetration Resolution) ---
        float ra_n = rA.cross(n), rb_n = rB.cross(n);
        float K_n = wA + wB + IA * ra_n * ra_n + IB * rb_n * rb_n;
        double compliance = 1e-8f; // tweak: 0 = rigid, higher = softer
        K_n = K_n + static_cast<float>(compliance / (dt * dt));

        if (K_n > 1e-8f) {
            float correction_depth = (contact.penetration_depth - penetration_tolerance) /
                                     static_cast<float>(contact.count);
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

void init_velocity_pass(FxContact& contact) {
    for (size_t i = 0; i < 2; ++i)
        contact.vn_pre[i] = 0.0f;
    if (!contact.is_valid() || contact.penetration_depth <= 0.0f) return;
    if (!contact.entity1 || !contact.entity2) return;

    FxEntity& A = *contact.entity1;
    FxEntity& B = *contact.entity2;
    const FxVec2f n = contact.normal;
    const FxVec2f t(-n.y(), n.x());

    contact.wA = eff_inv_mass(A);
    contact.wB = eff_inv_mass(B);
    contact.IA = eff_inv_inertia(A);
    contact.IB = eff_inv_inertia(B);

    const FxVec2f pA = A.pose.get_xy();
    const FxVec2f pB = B.pose.get_xy();
    for (size_t i = 0; i < contact.count; ++i) {
        const FxVec2f p = contact.position[i];
        contact.rA[i] = p - pA;
        contact.rB[i] = p - pB;
        contact.ra_n[i] = contact.rA[i].cross(n);
        contact.rb_n[i] = contact.rB[i].cross(n);
        contact.ra_t[i] = contact.rA[i].cross(t);
        contact.rb_t[i] = contact.rB[i].cross(t);
        contact.K_n[i] = contact.wA + contact.wB + contact.IA * contact.ra_n[i] * contact.ra_n[i] +
                         contact.IB * contact.rb_n[i] * contact.rb_n[i];
        contact.K_t[i] = contact.wA + contact.wB + contact.IA * contact.ra_t[i] * contact.ra_t[i] +
                         contact.IB * contact.rb_t[i] * contact.rb_t[i];

        const FxVec2f vA = A.velocity_at_local_point(contact.rA[i]);
        const FxVec2f vB = B.velocity_at_local_point(contact.rB[i]);
        contact.vn_pre[i] = (vB - vA).dot(n);
    }
}

void warm_start(FxContact& contact) {
    if (!contact.is_valid() || contact.penetration_depth <= 0.0f) return;
    if (!contact.entity1 || !contact.entity2) return;

    FxEntity& A = *contact.entity1;
    FxEntity& B = *contact.entity2;
    FxVec2f n = contact.normal;
    FxVec2f t(-n.y(), n.x());

    for (size_t i = 0; i < contact.count; ++i) {
        const float jn = contact.jn_warm[i];
        const float jt = contact.jt_warm[i];
        FxVec2f impulse = n * jn + t * jt;
        A.velocity.xy() -= contact.wA * impulse;
        B.velocity.xy() += contact.wB * impulse;
        A.velocity.theta() -= contact.IA * (jn * contact.ra_n[i] + jt * contact.ra_t[i]);
        B.velocity.theta() += contact.IB * (jn * contact.rb_n[i] + jt * contact.rb_t[i]);
        // Book the warm-start as applied so excess can be released later.
        contact.jn_accumulated[i] = jn;
        contact.jt_accumulated[i] = jt;
    }
}

// Post-constraint velocity impulses for restitution and dynamic friction
void resolve_velocities(FxContact& contact) {
    if (!contact.is_valid() || contact.penetration_depth <= 0.0f) return;
    if (!contact.entity1 || !contact.entity2) return;

    // Get entity references
    FxEntity& A = *contact.entity1;
    FxEntity& B = *contact.entity2;
    FxVec2f n = (contact.normal);
    FxVec2f t(-n.y(), n.x()); // fixed tangent (no normalize of vRel_t)

    // Restitution takes max so the liveliest surface sets the bounce and a bouncy body bounces
    // off anything; friction takes min so the slipperiest surface wins and ice stays slippery.
    const float e = std::clamp(std::max(A.elasticity, B.elasticity), 0.0f, 1.0f);
    const float mu_s = std::clamp(std::min(A.static_friction, B.static_friction), 0.0f, 10.0f);
    const float mu_k = std::clamp(std::min(A.dynamic_friction, B.dynamic_friction), 0.0f, 10.0f);

    // Lever arms and effective masses come from the per-substep cache in init_velocity_pass:
    // this function sweeps every contact several times per substep, and none of them change
    // between sweeps.
    const float wA = contact.wA, wB = contact.wB, IA = contact.IA, IB = contact.IB;

    // Iteratively resolve normal impulses
    for (int iter = 0; iter < 2; iter++) {
        for (size_t i = 0; i < contact.count; i++) {
            size_t k = (i + iter) % contact.count;
            auto vA = A.velocity_at_local_point(contact.rA[k]);
            auto vB = B.velocity_at_local_point(contact.rB[k]);

            // Relative velocity and its normal component
            float vn = (vB - vA).dot(n);

            // --- Velocity Correction (Normal Impulse) ---
            const float ra_n = contact.ra_n[k], rb_n = contact.rb_n[k];
            const float K_n = contact.K_n[k];

            // Fixed restitution target from substep start (later sweeps must not cancel bounce).
            const float vn_target =
                (contact.vn_pre[k] < -kRestitutionSlop) ? -e * contact.vn_pre[k] : 0.0f;
            if (K_n > 1e-6f) {
                float fresh_jn = -(vn - vn_target) / K_n;
                float old_jn = contact.jn_accumulated[k];
                float new_jn = std::max(0.0f, old_jn + fresh_jn);
                float delta_jn = new_jn - old_jn;
                contact.jn_accumulated[k] = new_jn;

                // Negative delta releases excess impulse already applied this substep.
                if (delta_jn != 0.0f) {
                    FxVec2f Pn = n * delta_jn;
                    A.velocity.xy() -= wA * Pn;
                    B.velocity.xy() += wB * Pn;
                    A.velocity.theta() -= IA * delta_jn * ra_n;
                    B.velocity.theta() += IB * delta_jn * rb_n;
                }
            }
        }
    }

    // Friction cone budget = normal impulse this substep (sum once, not per iter).
    float jn_sum = 0.0f;
    for (size_t i = 0; i < contact.count; i++)
        jn_sum += contact.jn_accumulated[i];

    // Single pass friction resolution using accumulated normal impulses
    for (size_t i = 0; i < contact.count; i++) {
        FxVec2f vA = A.velocity_at_local_point(contact.rA[i]);
        FxVec2f vB = B.velocity_at_local_point(contact.rB[i]);
        float vt = (vB - vA).dot(t);

        const float ra_t = contact.ra_t[i], rb_t = contact.rb_t[i];
        const float Kt = contact.K_t[i];
        if (Kt <= 1e-8f) continue;

        float fresh_jt = -vt / Kt;
        float old_jt = contact.jt_accumulated[i];
        float new_jt = old_jt + fresh_jt;
        float max_static = mu_s * std::max(0.f, jn_sum);
        if (std::fabs(new_jt) > max_static) {
            float max_dynamic = mu_k * std::max(0.f, jn_sum);
            new_jt = (new_jt >= 0.f ? 1.f : -1.f) * max_dynamic;
        }

        contact.jt_accumulated[i] = new_jt;
        float delta_jt = new_jt - old_jt;

        FxVec2f Pt = t * delta_jt;
        A.velocity.xy() -= wA * Pt;
        B.velocity.xy() += wB * Pt;
        A.velocity.theta() -= IA * delta_jt * ra_t;
        B.velocity.theta() += IB * delta_jt * rb_t;
    }
}
} // namespace FxSolver