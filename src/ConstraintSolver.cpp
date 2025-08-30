#include "Solver.h"
#include "Entity.h"
#include <iostream>
#include <unordered_set>
#include <algorithm>
#include <cmath>

namespace FxSolver {
    void resolve_penetration(const FxContact& contact) {
        // Early exits for invalid contacts
        if (!contact.is_valid) return;
        if (!contact.entity1 || !contact.entity2) return;
        // Allow small penetration - only resolve if depth exceeds threshold
        const float penetration_tolerance = 1e-3f; // tolerance
        if (contact.penetration_depth <= penetration_tolerance) return;
      
        // Get entity references
        FxEntity& A = *contact.entity1;
        FxEntity& B = *contact.entity2;
        
        // Debug output for penetration depth and entity information
        std::cout << "Entity A: " << A.get_name() << " pos: [" << A.pose.x() << ", " << A.pose.y() << "]" << std::endl;
        std::cout << "Entity B: " << B.get_name() << " pos: [" << B.pose.x() << ", " << B.pose.y() << "]" << std::endl;
        std::cout << "Contact position: [" << contact.position.x() << ", " << contact.position.y() << "]" << std::endl;
        std::cout << "Penetration depth: " << contact.penetration_depth << std::endl;
        // Normalize contact normal
        FxVec2f n = contact.normal; 
        
        // Mass and inertia properties
        const float wA = A.inv_mass(), wB = B.inv_mass();
        const float IA = A.inv_inertia(), IB = B.inv_inertia();
        
        // Contact point relative to each entity's center
        FxVec2f rA = contact.position - A.pose.xy();
        FxVec2f rB = contact.position - B.pose.xy();
        
        // --- Position Correction (Penetration Resolution) ---
        float ra_n = rA.cross(n), rb_n = rB.cross(n); 
        float K_n = wA + wB + IA * ra_n * ra_n + IB * rb_n * rb_n;
        
        if (K_n > 1e-8f) { 
            float correction_depth = contact.penetration_depth;
            float lambdaP = correction_depth / K_n; 
            FxVec2f dP = n * lambdaP; 
            A.pose.xy() -= wA * dP; 
            B.pose.xy() += wB * dP; 
            A.prev_pose.xy() -= wA * dP; 
            B.prev_pose.xy() += wB * dP; 
            
            // Apply angular corrections
            A.pose.theta() -= IA * lambdaP * ra_n;
            B.pose.theta() += IB * lambdaP * rb_n;
            A.prev_pose.theta() -= IA * lambdaP * ra_n;
            B.prev_pose.theta() += IB * lambdaP * rb_n;
        }
    }

    // Post-constraint velocity impulses for restitution and dynamic friction
    void resolve_velocities(const FxContact& contact) {
        if (!contact.is_valid || contact.penetration_depth <= 1e-6f) return;
        if (!contact.entity1 || !contact.entity2) return;

        // Get entity references
        FxEntity& A = *contact.entity1;
        FxEntity& B = *contact.entity2;
        
        // Normalize contact normal
        FxVec2f n = contact.normal; 

        // Mass and inertia properties
        const float wA = A.inv_mass(), wB = B.inv_mass();
        const float IA = A.inv_inertia(), IB = B.inv_inertia();
        
        // Contact point relative to each entity's center
        FxVec2f rA = contact.position - A.pose.xy();
        FxVec2f rB = contact.position - B.pose.xy();
        
        // Calculate velocities at contact point 
        FxVec2f vA = A.velocity_at_local_point(rA);
        FxVec2f vB = B.velocity_at_local_point(rB);

        //  // Debug output
        //  std::cout << " | Entity A: " << A.get_name() << " pos: [" << A.pose.x() << ", " << A.pose.y() << "]" << std::endl
        //  << " | Entity B: " << B.get_name() << " pos: [" << B.pose.x() << ", " << B.pose.y() << "]" << std::endl
        //  << " | Contact position: [" << contact.position.x() << ", " << contact.position.y() << "]" << std::endl
        //  << " | vA: [" << vA.x() << ", " << vA.y() << "]" << std::endl
        //  << " | vB: [" << vB.x() << ", " << vB.y() << "]" << std::endl;

        // Relative velocity and its normal component
        float vn = (vB - vA).dot(n); 
        
        // --- Velocity Correction (Normal Impulse) ---
        float e = std::min(A.elasticity, B.elasticity); 
        float ra_n = rA.cross(n), rb_n = rB.cross(n); 
        float K_n = wA + wB + IA * ra_n * ra_n + IB * rb_n * rb_n;
        
        float jn = 0.0f;
        if (vn < -1e-4f && K_n > 1e-6f) { 
            jn = -(1.0f + e) * vn / K_n;
            jn = std::max(0.0f, jn);
            if (jn > 0.0f) {
                FxVec2f Pn = n * jn; 
                // Apply velocity changes directly
                A.velocity.xy() -= wA * Pn; 
                B.velocity.xy() += wB * Pn; 
                A.velocity.theta() -= IA  * jn * ra_n; 
                B.velocity.theta() += IB  * jn * rb_n;
            }
        }
        
        // Recalculate velocities at contact point after normal impulse using local points
        vA = A.velocity_at_local_point(rA);
        vB = B.velocity_at_local_point(rB);
        FxVec2f vRel = vB - vA;
        FxVec2f vRel_t = vRel - (vRel).dot(n)*n; 
        FxVec2f t = vRel_t.normalized();
        float vt = vRel_t.norm();

        // --- Friction (Tangential Impulse) ---
        float ra_t = rA.cross(t), rb_t = rB.cross(t); 
        float K_t = wA + wB + IA * ra_t * ra_t + IB * rb_t * rb_t; 

        float jt = 0.0f;
        if (K_t > 1e-7f) { 
            // Unclamped tangential impulse to cancel vt
            jt = -vt / K_t;
            // Coulomb cone: clamp by μ * jn
            float mu_s = std::max(A.static_friction,  B.static_friction);
            float mu_k = std::max(A.dynamic_friction, B.dynamic_friction);
            float max_static = mu_s * jn;
            if (std::fabs(jt) > max_static && std::abs(vt) > 1e-3f) {
                // Kinetic friction at the edge of the cone
                float sign = (jt >= 0.0f) ? 1.0f : -1.0f;
                jt = sign * (mu_k * jn);
            }
            if (std::fabs(jt) > 1e-7f) {
                FxVec2f Pt = t * jt;
                A.velocity.xy()     -= wA * Pt;
                B.velocity.xy()     += wB * Pt;
                A.velocity.theta()  -= IA * jt * ra_t;
                B.velocity.theta()  += IB * jt * rb_t;
            }
        }
    }



    // // XPBD Constraint Solver following the algorithm outline
    // void solve_constraints_xpbd(const std::vector<FxContact>& contacts, int solverIterations, float dt, float compliance) {
    //     std::cout << "XPBD_SOLVER: Called with " << contacts.size() << " contacts, " 
    //               << solverIterations << " iterations, dt=" << dt << std::endl;

    //     if (contacts.empty() || dt <= 1e-12f) {
    //         std::cout << "XPBD_SOLVER: Early return - no contacts or invalid dt" << std::endl;
    //         return;
    //     }

    //     // Gather all dynamic entities
    //     std::unordered_set<FxEntity*> dynamic_entities;
    //     for (const auto& contact : contacts) {
    //         if (contact.is_valid && contact.entity1 && contact.entity2) {
    //             if (contact.entity1->inv_mass() > 0.0f) dynamic_entities.insert(contact.entity1.get());
    //             if (contact.entity2->inv_mass() > 0.0f) dynamic_entities.insert(contact.entity2.get());
    //         }
    //     }

    //     // --- No compliance for rigid contacts: each sweep fully corrects penetration ---
    //     for (int iter = 0; iter < solverIterations; ++iter) {
    //         for (const auto& orig : contacts) {
    //             // Reevaluate contact on *current* poses:
    //             FxContact c = FxSolver::collision_check(orig.entity1, orig.entity2);
    //             if (!c.is_valid || c.penetration_depth <= 0.f) continue;

    //             FxEntity &A = *c.entity1, &B = *c.entity2;
    //             float wA = A.inv_mass(), wB = B.inv_mass();
    //             if (wA + wB <= 0.f) continue;

    //             // Build normal constraint
    //             FxVec2f n = c.normal;
    //             FxVec2f rA = c.position - A.pose.xy();
    //             FxVec2f rB = c.position - B.pose.xy();
    //             float ra_n = rA.cross(n), rb_n = rB.cross(n);

    //             // Effective mass
    //             float K = wA + wB
    //                     + A.inv_inertia() * ra_n*ra_n
    //                     + B.inv_inertia() * rb_n*rb_n;
    //             if (K < 1e-12f) continue;

    //             // Δλ = -C/K  (rigid PBD)
    //             float deltaLambda = -c.penetration_depth / K;

    //             // Position + angular updates
    //             FxVec2f dp = n * deltaLambda;
    //             if (wA > 0) A.pose.xy() -= dp * wA;
    //             if (wB > 0) B.pose.xy() += dp * wB;
    //             float tauA = -rA.cross(dp) * A.inv_inertia();
    //             float tauB =  rB.cross(dp) * B.inv_inertia();
    //             A.pose.theta() += tauA;
    //             B.pose.theta() += tauB;

    //             // Friction (tangent PBD)
    //             FxVec2f t{-n.y(), n.x()};
    //             FxVec2f deltaX = B.pose.xy() - A.pose.xy();
    //             float Ct = t.dot(deltaX);
    //             float ra_t = rA.cross(t), rb_t = rB.cross(t);
    //             float Kt  = wA + wB
    //                       + A.inv_inertia() * ra_t * ra_t
    //                       + B.inv_inertia() * rb_t * rb_t;
    //             if (Kt < 1e-12f) continue;
    //             // clamp static friction
    //             float staticFriction = std::max(A.static_friction, B.static_friction);
    //             float maxCt = staticFriction * c.penetration_depth;
    //             float deltaLambdaT = -Ct / Kt;
    //             deltaLambdaT = std::clamp(deltaLambdaT, -maxCt, +maxCt);
    //             FxVec2f dpt = t * deltaLambdaT;
    //             if (wA > 0) A.pose.xy() -= dpt * wA;
    //             if (wB > 0) B.pose.xy() += dpt * wB;
    //         }
    //     }

    //     // --- Recompute velocities from corrected poses ---
    //     for (auto* e : dynamic_entities) {
    //         FxVec2f posOld = e->prev_pose.xy(), posNew = e->pose.xy();
    //         e->velocity.xy()   = (posNew - posOld) / dt;
    //         float   thOld  = e->prev_pose.theta(), thNew = e->pose.theta();
    //         e->velocity.theta() = (thNew  - thOld ) / dt;
    //     }
    // }

    // // Post-constraint velocity impulses for restitution and friction
    // void apply_post_constraint_impulses(const std::vector<FxContact>& contacts, float /*dt*/) {
    //     for (const auto& original_contact : contacts) {
    //         if (!original_contact.is_valid || !original_contact.entity1 || !original_contact.entity2) continue;

    //         // Re-evaluate contact for current velocities
    //         auto contact = FxSolver::collision_check(original_contact.entity1, original_contact.entity2);
    //         if (!contact.is_valid) continue;

    //         FxEntity& A = *contact.entity1;
    //         FxEntity& B = *contact.entity2;

    //         float wA = A.inv_mass();
    //         float wB = B.inv_mass();
    //         if (wA <= 0.0f && wB <= 0.0f) continue;

    //         // Contact parameters
    //         FxVec2f n = contact.normal;
    //         FxVec2f contact_pos = contact.position;
    //         FxVec2f rA = contact_pos - A.pose.xy();
    //         FxVec2f rB = contact_pos - B.pose.xy();
            
    //         float iA = A.inv_inertia();
    //         float iB = B.inv_inertia();

    //         // Calculate relative velocity at contact point
    //         FxVec2f vA = A.velocity.xy() + FxVec2f(-rA.y() * A.velocity.theta(), rA.x() * A.velocity.theta());
    //         FxVec2f vB = B.velocity.xy() + FxVec2f(-rB.y() * B.velocity.theta(), rB.x() * B.velocity.theta());
    //         FxVec2f relative_velocity = vB - vA;

    //         // Normal and tangential components
    //         float v_n = relative_velocity.dot(n);
    //         FxVec2f tangent = relative_velocity - n * v_n;
    //         float v_t_mag = tangent.norm();
    //         FxVec2f t = (v_t_mag > 1e-12f) ? tangent / v_t_mag : FxVec2f(0, 0);

    //         // Effective masses
    //         float rA_cross_n = rA.cross(n);
    //         float rB_cross_n = rB.cross(n);
    //         float K_normal = wA + wB + iA * rA_cross_n * rA_cross_n + iB * rB_cross_n * rB_cross_n;
            
    //         float rA_cross_t = rA.cross(t);
    //         float rB_cross_t = rB.cross(t);
    //         float K_tangent = wA + wB + iA * rA_cross_t * rA_cross_t + iB * rB_cross_t * rB_cross_t;

    //         if (K_normal <= 1e-12f) continue;

    //         // === RESTITUTION IMPULSE ===
    //         if (v_n < -1e-6f) {  // Approaching contact
    //             float e = std::min(A.elasticity, B.elasticity);
    //             float restitution_impulse = -(1.0f + e) * v_n / K_normal;
    //             FxVec2f J_restitution = n * restitution_impulse;
                
    //             A.apply_impulse(-J_restitution, contact_pos);
    //             B.apply_impulse(J_restitution, contact_pos);
    //         }

    //         // === FRICTION IMPULSE ===
    //         if (v_t_mag > 1e-6f && K_tangent > 1e-12f) {
    //             float mu_s = std::sqrt(A.static_friction * B.static_friction);
    //             float mu_k = std::sqrt(A.dynamic_friction * B.dynamic_friction);
                
    //             // Normal force estimate (from constraint pressure)
    //             float normal_force = std::abs(v_n) / K_normal;
                
    //             // Static friction threshold
    //             float max_static_friction = mu_s * normal_force;
    //             float friction_impulse = -v_t_mag / K_tangent;
                
    //             // Apply static or kinetic friction
    //             if (std::abs(friction_impulse) <= max_static_friction) {
    //                 // Static friction: prevent sliding
    //                 FxVec2f J_friction = t * friction_impulse;
    //                 A.apply_impulse(-J_friction, contact_pos);
    //                 B.apply_impulse(J_friction, contact_pos);
    //             } else {
    //                 // Kinetic friction: limit sliding
    //                 float kinetic_impulse = -mu_k * normal_force * (friction_impulse > 0 ? 1.0f : -1.0f);
    //                 FxVec2f J_friction = t * kinetic_impulse;
    //                 A.apply_impulse(-J_friction, contact_pos);
    //                 B.apply_impulse(J_friction, contact_pos);
    //             }
    //         }
    //     }
    // }

    // // Wrapper function for backward compatibility and parameter defaults
    // void resolve_contacts(const std::vector<FxContact>& contacts, int pbdIters, float dt, const FxVec2f& /*scene_gravity*/, float beta) {
    //     // Convert beta to compliance: α = 1/(β * dt^2)
    //     float compliance = 1.0f / (beta * dt * dt);

    //     // Call the XPBD constraint solver
    //     solve_constraints_xpbd(contacts, pbdIters, dt, compliance);

    //     // Post-constraint restitution and friction impulses (velocity-level corrections)
    //     // apply_post_constraint_impulses(contacts, dt);
    // }

} // namespace FxConstraints 



// // XPBD Constraint Solver following the algorithm outline
// void solve_constraints_xpbd(const std::vector<FxContact>& contacts, int solverIterations, float dt, float compliance) {
//     std::cout << "XPBD_SOLVER: Called with " << contacts.size() << " contacts, " 
//               << solverIterations << " iterations, dt=" << dt << std::endl;

//     if (contacts.empty() || dt <= 1e-12f) {
//         std::cout << "XPBD_SOLVER: Early return - no contacts or invalid dt" << std::endl;
//         return;
//     }

//     // Gather all dynamic entities
//     std::unordered_set<FxEntity*> dynamic_entities;
//     for (const auto& contact : contacts) {
//         if (contact.is_valid && contact.entity1 && contact.entity2) {
//             if (contact.entity1->inv_mass() > 0.0f) dynamic_entities.insert(contact.entity1.get());
//             if (contact.entity2->inv_mass() > 0.0f) dynamic_entities.insert(contact.entity2.get());
//         }
//     }

//     // --- No compliance for rigid contacts: each sweep fully corrects penetration ---
//     for (int iter = 0; iter < solverIterations; ++iter) {
//         for (const auto& orig : contacts) {
//             // Reevaluate contact on *current* poses:
//             FxContact c = collision_check(orig.entity1, orig.entity2);
//             if (!c.is_valid || c.penetration_depth <= 0.f) continue;

//             FxEntity &A = *c.entity1, &B = *c.entity2;
//             float wA = A.inv_mass(), wB = B.inv_mass();
//             if (wA + wB <= 0.f) continue;

//             // Build normal constraint
//             FxVec2f n = c.normal;
//             FxVec2f rA = c.position - A.pose.xy();
//             FxVec2f rB = c.position - B.pose.xy();
//             float ra_n = rA.cross(n), rb_n = rB.cross(n);

//             // Effective mass
//             float K = wA + wB
//                     + A.inv_inertia() * ra_n*ra_n
//                     + B.inv_inertia() * rb_n*rb_n;
//             if (K < 1e-12f) continue;

//             // Δλ = -C/K  (rigid PBD)
//             float deltaLambda = -c.penetration_depth / K;

//             // Position + angular updates
//             FxVec2f dp = n * deltaLambda;
//             if (wA > 0) A.pose.xy() -= dp * wA;
//             if (wB > 0) B.pose.xy() += dp * wB;
//             float tauA = -rA.cross(dp) * A.inv_inertia();
//             float tauB =  rB.cross(dp) * B.inv_inertia();
//             A.pose.theta() += tauA;
//             B.pose.theta() += tauB;

//             // Friction (tangent PBD)
//             FxVec2f t{-n.y(), n.x()};
//             FxVec2f deltaX = B.pose.xy() - A.pose.xy();
//             float Ct = t.dot(deltaX);
//             float ra_t = rA.cross(t), rb_t = rB.cross(t);
//             float Kt  = wA + wB
//                       + A.inv_inertia() * ra_t * ra_t
//                       + B.inv_inertia() * rb_t * rb_t;
//             if (Kt < 1e-12f) continue;
//             // clamp static friction
//             float staticFriction = std::max(A.static_friction, B.static_friction);
//             float maxCt = staticFriction * c.penetration_depth;
//             float deltaLambdaT = -Ct / Kt;
//             deltaLambdaT = std::clamp(deltaLambdaT, -maxCt, +maxCt);
//             FxVec2f dpt = t * deltaLambdaT;
//             if (wA > 0) A.pose.xy() -= dpt * wA;
//             if (wB > 0) B.pose.xy() += dpt * wB;
//         }
//     }

//     // --- Recompute velocities from corrected poses ---
//     for (auto* e : dynamic_entities) {
//         FxVec2f posOld = e->prev_pose.xy(), posNew = e->pose.xy();
//         e->velocity.xy()   = (posNew - posOld) / dt;
//         float   thOld  = e->prev_pose.theta(), thNew = e->pose.theta();
//         e->velocity.theta() = (thNew  - thOld ) / dt;
//     }
// }

// // Post-constraint velocity impulses for restitution and friction
// void apply_post_constraint_impulses(const std::vector<FxContact>& contacts, float /*dt*/) {
//     for (const auto& original_contact : contacts) {
//         if (!original_contact.is_valid || !original_contact.entity1 || !original_contact.entity2) continue;

//         // Re-evaluate contact for current velocities
//         auto contact = collision_check(original_contact.entity1, original_contact.entity2);
//         if (!contact.is_valid) continue;

//         FxEntity& A = *contact.entity1;
//         FxEntity& B = *contact.entity2;

//         float wA = A.inv_mass();
//         float wB = B.inv_mass();
//         if (wA <= 0.0f && wB <= 0.0f) continue;

//         // Contact parameters
//         FxVec2f n = contact.normal;
//         FxVec2f contact_pos = contact.position;
//         FxVec2f rA = contact_pos - A.pose.xy();
//         FxVec2f rB = contact_pos - B.pose.xy();
        
//         float iA = A.inv_inertia();
//         float iB = B.inv_inertia();

//         // Calculate relative velocity at contact point
//         FxVec2f vA = A.velocity.xy() + FxVec2f(-rA.y() * A.velocity.theta(), rA.x() * A.velocity.theta());
//         FxVec2f vB = B.velocity.xy() + FxVec2f(-rB.y() * B.velocity.theta(), rB.x() * B.velocity.theta());
//         FxVec2f relative_velocity = vB - vA;

//         // Normal and tangential components
//         float v_n = relative_velocity.dot(n);
//         FxVec2f tangent = relative_velocity - n * v_n;
//         float v_t_mag = tangent.norm();
//         FxVec2f t = (v_t_mag > 1e-12f) ? tangent / v_t_mag : FxVec2f(0, 0);

//         // Effective masses
//         float rA_cross_n = rA.cross(n);
//         float rB_cross_n = rB.cross(n);
//         float K_normal = wA + wB + iA * rA_cross_n * rA_cross_n + iB * rB_cross_n * rB_cross_n;
        
//         float rA_cross_t = rA.cross(t);
//         float rB_cross_t = rB.cross(t);
//         float K_tangent = wA + wB + iA * rA_cross_t * rA_cross_t + iB * rB_cross_t * rB_cross_t;

//         if (K_normal <= 1e-12f) continue;

//         // === RESTITUTION IMPULSE ===
//         if (v_n < -1e-6f) {  // Approaching contact
//             float e = std::min(A.elasticity, B.elasticity);
//             float restitution_impulse = -(1.0f + e) * v_n / K_normal;
//             FxVec2f J_restitution = n * restitution_impulse;
            
//             A.apply_impulse(-J_restitution, contact_pos);
//             B.apply_impulse(J_restitution, contact_pos);
//         }

//         // === FRICTION IMPULSE ===
//         if (v_t_mag > 1e-6f && K_tangent > 1e-12f) {
//             float mu_s = std::sqrt(A.static_friction * B.static_friction);
//             float mu_k = std::sqrt(A.dynamic_friction * B.dynamic_friction);
            
//             // Normal force estimate (from constraint pressure)
//             float normal_force = std::abs(v_n) / K_normal;
            
//             // Static friction threshold
//             float max_static_friction = mu_s * normal_force;
//             float friction_impulse = -v_t_mag / K_tangent;
            
//             // Apply static or kinetic friction
//             if (std::abs(friction_impulse) <= max_static_friction) {
//                 // Static friction: prevent sliding
//                 FxVec2f J_friction = t * friction_impulse;
//                 A.apply_impulse(-J_friction, contact_pos);
//                 B.apply_impulse(J_friction, contact_pos);
//             } else {
//                 // Kinetic friction: limit sliding
//                 float kinetic_impulse = -mu_k * normal_force * (friction_impulse > 0 ? 1.0f : -1.0f);
//                 FxVec2f J_friction = t * kinetic_impulse;
//                 A.apply_impulse(-J_friction, contact_pos);
//                 B.apply_impulse(J_friction, contact_pos);
//             }
//         }
//     }
// }

// // Wrapper function for backward compatibility and parameter defaults
// void resolve_contacts(const std::vector<FxContact>& contacts, int pbdIters, float dt, const FxVec2f& /*scene_gravity*/, float beta) {
//     // Convert beta to compliance: α = 1/(β * dt^2)
//     float compliance = 1.0f / (beta * dt * dt);

//     // Call the XPBD constraint solver
//     solve_constraints_xpbd(contacts, pbdIters, dt, compliance);

//     // Post-constraint restitution and friction impulses (velocity-level corrections)
//     // apply_post_constraint_impulses(contacts, dt);
// }