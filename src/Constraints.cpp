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
    // entity2 may be null: a world-anchored constraint has only one body to move, and an
    // absent second body behaves exactly like an immovable one.
    if (!entity1) return;

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
    const float w2 = entity2 ? entity2->inv_mass() : 0.0f;
    const float I1 = entity1->inv_inertia();
    const float I2 = entity2 ? entity2->inv_inertia() : 0.0f;

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
    const FxVec3f landed1 =
        entity1->apply_pose_correction(FxVec3f{dxy1.x(), dxy1.y(), I1 * dLambda * gth1});
    FxVec3f landed2{0.0f, 0.0f, 0.0f};
    if (entity2)
        landed2 = entity2->apply_pose_correction(FxVec3f{dxy2.x(), dxy2.y(), I2 * dLambda * gth2});

    if (carries_velocity) return;
    // Drag the reference pose along so the move produces no velocity, exactly as penetration
    // recovery does. Uses the delta that actually landed, not the one asked for.
    auto follow = [](FxEntity& e, const FxVec3f& d) {
        e.prev_pose.x() += d.x();
        e.prev_pose.y() += d.y();
        e.prev_pose.theta() = FxAngleWrap(e.prev_pose.theta() + d.theta());
    };
    follow(*entity1, landed1);
    if (entity2) follow(*entity2, landed2);
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
FxMouseConstraint::FxMouseConstraint(const std::shared_ptr<FxEntity>& e1, const FxVec2f& anchor,
                                     bool anchor_is_local) {
    entity1 = e1;
    entity2 = nullptr; // the target is a point in the world, not a body
    m_local_anchor = anchor_is_local ? anchor : e1->to_entity_frame(anchor);
    m_target = e1->to_world_frame(m_local_anchor);
    m_name = "MouseConstraint";
    compliance = 1e-4; // soft by default; a stiff drag through a heavy body explodes
    carries_velocity = false; // a dragged body should follow the cursor, not be launched by it
}

void FxMouseConstraint::evaluate(float& C, FxVec2f& g1, FxVec2f& g2, float& gth1, float& gth2,
                                 bool& active) const {
    g2 = FxVec2f(0.0f, 0.0f);
    gth2 = 0.0f;
    if (!enabled || !entity1) {
        active = false;
        return;
    }

    const FxVec2f grabbed = entity1->to_world_frame(m_local_anchor);
    const FxVec2f delta = grabbed - m_target;
    const float distance = delta.norm();
    if (distance < 1e-6f) {
        active = false;
        return;
    }

    // Pull the grabbed point straight at the target; the lever arm turns the body with it.
    C = distance;
    g1 = delta / distance;
    const FxVec2f r = grabbed - entity1->pose.get_xy();
    gth1 = r.cross(g1);
    active = true;
}

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

void init_velocity_pass(FxContact& contact, FxContactSolverData& data,
                        const FxSolverBodies& bodies) {
    for (size_t i = 0; i < 2; ++i)
        data.vn_pre[i] = 0.0f;
    if (!contact_is_solvable(contact)) return;

    FxEntity& A = *contact.entity1;
    FxEntity& B = *contact.entity2;
    const FxVec2f n = contact.normal;
    const FxVec2f t(-n.y(), n.x());

    const size_t ia = static_cast<size_t>(contact.body1);
    const size_t ib = static_cast<size_t>(contact.body2);
    data.wA = bodies.inv_m[ia];
    data.wB = bodies.inv_m[ib];
    data.IA = bodies.inv_i[ia];
    data.IB = bodies.inv_i[ib];

    // Restitution mixes by max so a bouncy body bounces off anything; friction by min so the
    // slipperiest surface wins. Resolved once per substep so the sweeps never touch an entity.
    data.restitution = std::clamp(std::max(A.elasticity, B.elasticity), 0.0f, 1.0f);
    data.mu_static = std::clamp(std::min(A.static_friction, B.static_friction), 0.0f, 10.0f);
    data.mu_kinetic = std::clamp(std::min(A.dynamic_friction, B.dynamic_friction), 0.0f, 10.0f);

    // Slots past the manifold are zeroed, not left stale: the batched solve reads a zero
    // effective mass as inactive, and this record is reused across substeps, so an old second
    // point would otherwise still look live.
    for (size_t i = contact.count; i < 2; ++i) {
        data.K_n[i] = 0.0f;
        data.K_t[i] = 0.0f;
        data.rA[i] = {0.0f, 0.0f};
        data.rB[i] = {0.0f, 0.0f};
        data.ra_n[i] = data.rb_n[i] = data.ra_t[i] = data.rb_t[i] = 0.0f;
    }

    const FxVec2f pA = A.pose.get_xy();
    const FxVec2f pB = B.pose.get_xy();
    for (size_t i = 0; i < contact.count; ++i) {
        const FxVec2f p = contact.position[i];
        data.rA[i] = p - pA;
        data.rB[i] = p - pB;
        data.ra_n[i] = data.rA[i].cross(n);
        data.rb_n[i] = data.rB[i].cross(n);
        data.ra_t[i] = data.rA[i].cross(t);
        data.rb_t[i] = data.rB[i].cross(t);
        data.K_n[i] = data.wA + data.wB + data.IA * data.ra_n[i] * data.ra_n[i] +
                      data.IB * data.rb_n[i] * data.rb_n[i];
        data.K_t[i] = data.wA + data.wB + data.IA * data.ra_t[i] * data.ra_t[i] +
                      data.IB * data.rb_t[i] * data.rb_t[i];

        const FxVec2f vA = bodies.velocity_at(contact.body1, data.rA[i]);
        const FxVec2f vB = bodies.velocity_at(contact.body2, data.rB[i]);
        data.vn_pre[i] = (vB - vA).dot(n);
    }
}

void batch_append(FxContactBatch& b, const FxContact& contact, const FxContactSolverData& data,
                  uint32_t contact_index) {
    b.ia.push_back(contact.body1);
    b.ib.push_back(contact.body2);
    b.nx.push_back(contact.normal.x());
    b.ny.push_back(contact.normal.y());
    b.tx.push_back(-contact.normal.y());
    b.ty.push_back(contact.normal.x());
    b.wA.push_back(data.wA);
    b.wB.push_back(data.wB);
    b.IA.push_back(data.IA);
    b.IB.push_back(data.IB);
    b.restitution.push_back(data.restitution);
    b.mu_s.push_back(data.mu_static);
    b.mu_k.push_back(data.mu_kinetic);
    b.contact_index.push_back(contact_index);
    for (size_t s = 0; s < 2; ++s) {
        b.rAx[s].push_back(data.rA[s].x());
        b.rAy[s].push_back(data.rA[s].y());
        b.rBx[s].push_back(data.rB[s].x());
        b.rBy[s].push_back(data.rB[s].y());
        b.ra_n[s].push_back(data.ra_n[s]);
        b.rb_n[s].push_back(data.rb_n[s]);
        b.ra_t[s].push_back(data.ra_t[s]);
        b.rb_t[s].push_back(data.rb_t[s]);
        b.K_n[s].push_back(data.K_n[s]);
        b.K_t[s].push_back(data.K_t[s]);
        b.vn_pre[s].push_back(data.vn_pre[s]);
        b.jn[s].push_back(contact.jn_accumulated[s]);
        b.jt[s].push_back(contact.jt_accumulated[s]);
    }
}

// The velocity sweep, a whole colour at a time. Every branch is a select so all lanes run the
// same instructions, and (i + iter) % count becomes (slot + iter) & 1 over two fixed slots --
// the same visit order, 0,1,1,0 for two points and 0,0 for one.
template<int SLOTS>
static void resolve_velocities_slots(FxContactBatch& b, std::size_t begin, std::size_t end,
                                     FxSolverBodies& bodies) {
    if (begin >= end) return;

    // Gather. Once per colour per pass rather than once per solve: the six components stay in
    // the columns while all six solves run over them.
    for (std::size_t i = begin; i < end; ++i) {
        const std::size_t ia = static_cast<std::size_t>(b.ia[i]);
        const std::size_t ib = static_cast<std::size_t>(b.ib[i]);
        b.vax[i] = bodies.vx[ia];
        b.vay[i] = bodies.vy[ia];
        b.wav[i] = bodies.w[ia];
        b.vbx[i] = bodies.vx[ib];
        b.vby[i] = bodies.vy[ib];
        b.wbv[i] = bodies.w[ib];
    }

    float* __restrict vax = b.vax.data();
    float* __restrict vay = b.vay.data();
    float* __restrict wav = b.wav.data();
    float* __restrict vbx = b.vbx.data();
    float* __restrict vby = b.vby.data();
    float* __restrict wbv = b.wbv.data();
    const float* __restrict nx = b.nx.data();
    const float* __restrict ny = b.ny.data();
    const float* __restrict tx = b.tx.data();
    const float* __restrict ty = b.ty.data();
    const float* __restrict wA = b.wA.data();
    const float* __restrict wB = b.wB.data();
    const float* __restrict IA = b.IA.data();
    const float* __restrict IB = b.IB.data();

    for (size_t iter = 0; iter < 2; ++iter) {
        for (size_t slot = 0; slot < static_cast<size_t>(SLOTS); ++slot) {
            // One-point manifolds solve point 0 twice, which is what the scalar kernel's
            // (i + iter) % count does when count is 1. Two-point manifolds visit 0,1,1,0.
            const size_t k = (SLOTS == 1) ? 0u : ((slot + iter) & 1u);
            const float* __restrict rAx = b.rAx[k].data();
            const float* __restrict rAy = b.rAy[k].data();
            const float* __restrict rBx = b.rBx[k].data();
            const float* __restrict rBy = b.rBy[k].data();
            const float* __restrict ra_n = b.ra_n[k].data();
            const float* __restrict rb_n = b.rb_n[k].data();
            const float* __restrict Kn = b.K_n[k].data();
            const float* __restrict vn_pre = b.vn_pre[k].data();
            const float* __restrict e = b.restitution.data();
            float* __restrict jn = b.jn[k].data();

            for (std::size_t i = begin; i < end; ++i) {
                const float vn = ((vbx[i] - wbv[i] * rBy[i]) - (vax[i] - wav[i] * rAy[i])) * nx[i] +
                                 ((vby[i] + wbv[i] * rBx[i]) - (vay[i] + wav[i] * rAx[i])) * ny[i];

                const float vn_target = (vn_pre[i] < -kRestitutionSlop) ? -e[i] * vn_pre[i] : 0.0f;

                // Repeat the comparison rather than hold it in a bool: a bool temporary has no
                // vector type and the vectoriser abandons the loop. Forcing the divisor to 1
                // where inactive stops a masked lane producing an infinity.
                const float k_safe = (Kn[i] > 1e-6f) ? Kn[i] : 1.0f;
                const float fresh = -(vn - vn_target) / k_safe;
                const float old_jn = jn[i];
                const float raised = std::max(0.0f, old_jn + fresh);
                const float new_jn = (Kn[i] > 1e-6f) ? raised : old_jn;
                const float delta = new_jn - old_jn;
                jn[i] = new_jn;

                const float pnx = nx[i] * delta, pny = ny[i] * delta;
                vax[i] -= wA[i] * pnx;
                vay[i] -= wA[i] * pny;
                vbx[i] += wB[i] * pnx;
                vby[i] += wB[i] * pny;
                wav[i] -= IA[i] * delta * ra_n[i];
                wbv[i] += IB[i] * delta * rb_n[i];
            }
        }
    }

    // Friction cone budget = the normal impulse this substep, summed once over both slots. A
    // one-point contact leaves slot 1 at zero, so the sum is right for both manifold sizes.
    {
        const float* __restrict jn0 = b.jn[0].data();
        const float* __restrict jn1 = b.jn[1].data();
        float* __restrict sum = b.jn_sum.data();
        for (std::size_t i = begin; i < end; ++i)
            sum[i] = (SLOTS == 1) ? jn0[i] : (jn0[i] + jn1[i]);
    }

    for (size_t slot = 0; slot < static_cast<size_t>(SLOTS); ++slot) {
        const float* __restrict rAx = b.rAx[slot].data();
        const float* __restrict rAy = b.rAy[slot].data();
        const float* __restrict rBx = b.rBx[slot].data();
        const float* __restrict rBy = b.rBy[slot].data();
        const float* __restrict ra_t = b.ra_t[slot].data();
        const float* __restrict rb_t = b.rb_t[slot].data();
        const float* __restrict Kt = b.K_t[slot].data();
        const float* __restrict mu_s = b.mu_s.data();
        const float* __restrict mu_k = b.mu_k.data();
        const float* __restrict jn_sum = b.jn_sum.data();
        float* __restrict jt = b.jt[slot].data();

        for (std::size_t i = begin; i < end; ++i) {
            const float vt = ((vbx[i] - wbv[i] * rBy[i]) - (vax[i] - wav[i] * rAy[i])) * tx[i] +
                             ((vby[i] + wbv[i] * rBx[i]) - (vay[i] + wav[i] * rAx[i])) * ty[i];

            const float kt_safe = (Kt[i] > 1e-8f) ? Kt[i] : 1.0f;
            const float fresh = -vt / kt_safe;
            const float old_jt = jt[i];
            float candidate = old_jt + fresh;

            const float budget = std::max(0.0f, jn_sum[i]);
            const float max_static = mu_s[i] * budget;
            const float max_dynamic = mu_k[i] * budget;
            const float clamped = (candidate >= 0.0f ? 1.0f : -1.0f) * max_dynamic;
            candidate = (std::fabs(candidate) > max_static) ? clamped : candidate;

            const float new_jt = (Kt[i] > 1e-8f) ? candidate : old_jt;
            const float delta = new_jt - old_jt;
            jt[i] = new_jt;

            const float ptx = tx[i] * delta, pty = ty[i] * delta;
            vax[i] -= wA[i] * ptx;
            vay[i] -= wA[i] * pty;
            vbx[i] += wB[i] * ptx;
            vby[i] += wB[i] * pty;
            wav[i] -= IA[i] * delta * ra_t[i];
            wbv[i] += IB[i] * delta * rb_t[i];
        }
    }

    // Scatter. Safe precisely because this range is one colour: no two entries in it touch the
    // same movable body, so no two writes can land on the same slot.
    for (std::size_t i = begin; i < end; ++i) {
        const std::size_t ia = static_cast<std::size_t>(b.ia[i]);
        const std::size_t ib = static_cast<std::size_t>(b.ib[i]);
        bodies.vx[ia] = b.vax[i];
        bodies.vy[ia] = b.vay[i];
        bodies.w[ia] = b.wav[i];
        bodies.vx[ib] = b.vbx[i];
        bodies.vy[ib] = b.vby[i];
        bodies.w[ib] = b.wbv[i];
    }
}

// Manifold size is a property of the range, not of a lane: FxScene sorts each colour by it and
// calls this once per run. One-point contacts in the two-slot kernel are correct but do double
// the arithmetic -- on the circle-heavy `pile` that was 14% slower than scalar.
void resolve_velocities_batched(FxContactBatch& b, std::size_t begin, std::size_t end,
                                FxSolverBodies& bodies, int slots) {
    if (slots == 1) resolve_velocities_slots<1>(b, begin, end, bodies);
    else resolve_velocities_slots<2>(b, begin, end, bodies);
}

void batch_write_back(const FxContactBatch& b, std::vector<FxContact>& contacts) {
    for (std::size_t i = 0; i < b.size(); ++i) {
        FxContact& c = contacts[b.contact_index[i]];
        for (size_t s = 0; s < 2; ++s) {
            c.jn_accumulated[s] = b.jn[s][i];
            c.jt_accumulated[s] = b.jt[s][i];
        }
    }
}

void warm_start(FxContact& contact, const FxContactSolverData& data, FxSolverBodies& bodies) {
    if (!contact_is_solvable(contact)) return;

    const size_t ia = static_cast<size_t>(contact.body1);
    const size_t ib = static_cast<size_t>(contact.body2);
    FxVec2f n = contact.normal;
    FxVec2f t(-n.y(), n.x());

    for (size_t i = 0; i < contact.count; ++i) {
        const float jn = contact.jn_warm[i];
        const float jt = contact.jt_warm[i];
        FxVec2f impulse = n * jn + t * jt;
        bodies.vx[ia] -= data.wA * impulse.x();
        bodies.vy[ia] -= data.wA * impulse.y();
        bodies.vx[ib] += data.wB * impulse.x();
        bodies.vy[ib] += data.wB * impulse.y();
        bodies.w[ia] -= data.IA * (jn * data.ra_n[i] + jt * data.ra_t[i]);
        bodies.w[ib] += data.IB * (jn * data.rb_n[i] + jt * data.rb_t[i]);
        // Book the warm-start as applied so excess can be released later.
        contact.jn_accumulated[i] = jn;
        contact.jt_accumulated[i] = jt;
    }
}

} // namespace FxSolver