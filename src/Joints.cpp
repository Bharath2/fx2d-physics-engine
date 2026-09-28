#include "Fx2D/Joints.h"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

// Entity name accessor methods for FxJoint
std::string FxJoint::get_entity1_name() const {
    return entity1 ? entity1->get_name() : "";
}

std::string FxJoint::get_entity2_name() const {
    return entity2 ? entity2->get_name() : "";
}

void FxJoint::namespace_constraints() {
    std::unordered_map<std::string, int> seen;
    for (auto& c : m_constraints) {
        if (!c) continue;
        // Constraints construct as e1_e2_Type; keep only the type suffix.
        const std::string& raw = c->m_name;
        const size_t cut = raw.rfind('_');
        const std::string type = (cut == std::string::npos) ? raw : raw.substr(cut + 1);
        const int n = ++seen[type];
        c->m_name = (n == 1) ? m_name + "_" + type : m_name + "_" + type + "_" + std::to_string(n);
    }
}

void FxJoint::reset_pid_state() {
    m_integral = 0.0f;
    m_previous_error = 0.0f;
}

void FxJoint::wake_entities() {
    if (entity1) entity1->wake();
    if (entity2) entity2->wake();
}

float FxJoint::eval_pid(float error, double dt) {
    if (dt <= 0.0) return 0.0f;

    float step_dt = static_cast<float>(dt);
    m_integral += error * step_dt;
    float derivative = (error - m_previous_error) / step_dt;
    m_previous_error = error;
    return m_pid.x() * error + m_pid.y() * m_integral + m_pid.z() * derivative;
}

float FxJoint::clamp_effort(float effort) const {
    return std::clamp(effort, -m_max_effort, m_max_effort);
}

void FxJoint::set_pid(const FxVec3f& pid) {
    m_pid = pid;
    reset_pid_state();
}

void FxJoint::set_effort(float effort) {
    m_target_effort = effort;
    // Direct effort targets should not inherit integral/derivative state from PID tracking modes.
    reset_pid_state();
    wake_entities();
}

void FxJoint::set_max_effort(float max_effort) {
    m_max_effort = std::max(0.0f, max_effort);
}

void FxJoint::set_control_mode(ControlMode mode) {
    if (get_control_mode() == mode) return;
    m_control_mode = mode;
    // Mode switches change the meaning of the control error, so restart the PID state.
    reset_pid_state();
    wake_entities();
}

// FxJoint base class implementation
FxJoint::FxJoint(const std::string& name, const std::shared_ptr<FxEntity>& e1,
                 const std::shared_ptr<FxEntity>& e2) {
    // e2 may be null: a world-anchored joint holds one body only.
    if (!e1) {
        throw std::invalid_argument("Joint entity1 cannot be null");
    }
    if (e2 && e1.get() == e2.get()) {
        throw std::invalid_argument("Joint cannot connect an entity to itself");
    }
    if (!is_valid_name(name)) {
        throw std::invalid_argument("FxJoint: Joint name must be alphanumeric or underscore.");
    }
    this->m_name = name;
    entity1 = e1;
    entity2 = e2;
}

// FxRevoluteJoint implementation
FxRevoluteJoint::FxRevoluteJoint(const std::string& name, const std::shared_ptr<FxEntity>& e1,
                                 const std::shared_ptr<FxEntity>& e2, const FxVec2f& anchor_point,
                                 float angle_min, float angle_max) :
    FxJoint(name, e1, e2), m_anchor_point(anchor_point) {
    m_constraints.reserve(2);

    auto anchor_constraint = std::make_shared<FxAnchorConstraint>(e1, e2, anchor_point, true);
    m_constraints.push_back(anchor_constraint);

    auto angular_limit = std::make_shared<FxAngularLimitConstraint>(e1, e2);
    angular_limit->lower_limit = angle_min;
    angular_limit->upper_limit = angle_max;
    m_constraints.push_back(angular_limit);

    namespace_constraints();
}

void FxRevoluteJoint::apply_torque_effort(float torque) {
    float clamped_torque = clamp_effort(torque);
    entity1->apply_torque(-clamped_torque);
    entity2->apply_torque(clamped_torque);
}

void FxRevoluteJoint::set_theta(float angle, bool instant) {
    m_target_theta = angle;
    reset_pid_state();
    wake_entities();
    instant = instant && m_instant;

    if (instant) {
        // Directly set the relative angle by adjusting entity2's angle
        float current_angle = FxAngleWrap(entity2->pose.theta() - entity1->pose.theta());
        float angle_error = FxAngleWrap(angle - current_angle);

        // Distribute angle correction based on inverse inertia
        float I1 = entity1->inv_inertia();
        float I2 = entity2->inv_inertia();
        float total_inv_inertia = I1 + I2;

        if (total_inv_inertia > 1e-12f) {
            float angle_correction1 = -angle_error * (I1 / total_inv_inertia);
            float angle_correction2 = angle_error * (I2 / total_inv_inertia);

            entity1->pose.theta() = FxAngleWrap(entity1->pose.theta() + angle_correction1);
            entity2->pose.theta() = FxAngleWrap(entity2->pose.theta() + angle_correction2);

            // Also update previous pose to maintain consistency
            entity1->prev_pose.theta() =
                FxAngleWrap(entity1->prev_pose.theta() + angle_correction1);
            entity2->prev_pose.theta() =
                FxAngleWrap(entity2->prev_pose.theta() + angle_correction2);
        }
    }
}

void FxRevoluteJoint::set_omega(float omega, bool instant) {
    m_target_omega = omega;
    reset_pid_state();
    wake_entities();
    instant = instant && m_instant;

    if (instant) {
        // Directly set the relative angular velocity
        float current_omega = entity2->velocity.theta() - entity1->velocity.theta();
        float omega_error = omega - current_omega;

        // Distribute velocity correction based on inverse inertia
        float I1 = entity1->inv_inertia();
        float I2 = entity2->inv_inertia();
        float total_inv_inertia = I1 + I2;

        if (total_inv_inertia > 1e-12f) {
            float omega_correction1 = -omega_error * (I1 / total_inv_inertia);
            float omega_correction2 = omega_error * (I2 / total_inv_inertia);

            entity1->velocity.theta() += omega_correction1;
            entity2->velocity.theta() += omega_correction2;
        }
    }
}

void FxRevoluteJoint::set_torque(float torque) {
    set_effort(torque);
}

float FxRevoluteJoint::get_theta() const {
    return FxAngleWrap(entity2->pose.theta() - entity1->pose.theta());
}

float FxRevoluteJoint::get_omega() const {
    return entity2->velocity.theta() - entity1->velocity.theta();
}

void FxRevoluteJoint::apply_controls(double dt) {
    if (!enabled) return;

    // Effort mode uses the stored target directly; other modes synthesize effort through PID.
    float effort = get_effort();
    if (get_control_mode() == ControlMode::VELOCITY) {
        effort = eval_pid(m_target_omega - get_omega(), dt);
    } else if (get_control_mode() == ControlMode::POSITION) {
        effort = eval_pid(FxAngleWrap(m_target_theta - get_theta()), dt);
    }

    apply_torque_effort(effort);
}

// FxPrismaticJoint implementation
FxPrismaticJoint::FxPrismaticJoint(const std::string& name, const std::shared_ptr<FxEntity>& e1,
                                   const std::shared_ptr<FxEntity>& e2, const FxVec2f& local_axis,
                                   float position_min, float position_max) :
    FxJoint(name, e1, e2),
    m_axis(local_axis.normalized()),
    m_position_min(position_min),
    m_position_max(position_max) {
    // Store initial distance projection along the axis
    FxVec2f world_axis = m_axis.rotate_rad(entity1->pose.theta());
    FxVec2f separation = entity2->pose.xy() - entity1->pose.xy();
    m_initial_distance = world_axis.dot(separation);

    m_constraints.reserve(3);

    auto motion_constraint = std::make_shared<FxMotionAlongAxisConstraint>(e1, e2, m_axis, true);
    m_constraints.push_back(motion_constraint);

    auto separation_constraint = std::make_shared<FxSeparationConstraint>(e1, e2, m_axis, true);
    separation_constraint->lower_limit = m_position_min;
    separation_constraint->upper_limit = m_position_max;
    m_constraints.push_back(separation_constraint);

    auto angle_lock = std::make_shared<FxAngleLockConstraint>(e1, e2, 0.0f);
    m_constraints.push_back(angle_lock);

    namespace_constraints();
}

void FxPrismaticJoint::apply_force_effort(float force) {
    FxVec2f world_axis = m_axis.rotate_rad(entity1->pose.theta());
    FxVec2f force_vector = world_axis * clamp_effort(force);
    entity1->apply_force(-force_vector);
    entity2->apply_force(force_vector);
}

void FxPrismaticJoint::set_position(float position, bool instant) {
    m_target_position = position;
    reset_pid_state();
    wake_entities();
    instant = instant && m_instant;

    if (instant) {
        // Transform local axis to world coordinates
        FxVec2f world_axis = m_axis.rotate_rad(entity1->pose.theta());

        // Calculate current position along axis
        FxVec2f separation = entity2->pose.xy() - entity1->pose.xy();
        float current_position = world_axis.dot(separation) - m_initial_distance;
        float position_error = position - current_position;

        // Distribute position correction based on inverse mass
        float m1 = entity1->inv_mass();
        float m2 = entity2->inv_mass();
        float total_inv_mass = m1 + m2;

        if (total_inv_mass > 1e-12f) {
            FxVec2f correction = world_axis * position_error;
            FxVec2f pos_correction1 = -correction * (m1 / total_inv_mass);
            FxVec2f pos_correction2 = correction * (m2 / total_inv_mass);

            entity1->pose.xy() += pos_correction1;
            entity2->pose.xy() += pos_correction2;

            // Also update previous pose to maintain consistency
            entity1->prev_pose.xy() += pos_correction1;
            entity2->prev_pose.xy() += pos_correction2;
        }
    }
}

void FxPrismaticJoint::set_velocity(float velocity, bool instant) {
    m_target_velocity = velocity;
    reset_pid_state();
    wake_entities();
    instant = instant && m_instant;

    if (instant) {
        // Transform local axis to world coordinates
        FxVec2f world_axis = m_axis.rotate_rad(entity1->pose.theta());

        // Calculate current velocity along axis
        FxVec2f relative_velocity = entity2->velocity.xy() - entity1->velocity.xy();
        float current_velocity = world_axis.dot(relative_velocity);
        float velocity_error = velocity - current_velocity;

        // Distribute velocity correction based on inverse mass
        float m1 = entity1->inv_mass();
        float m2 = entity2->inv_mass();
        float total_inv_mass = m1 + m2;

        if (total_inv_mass > 1e-12f) {
            FxVec2f correction = world_axis * velocity_error;
            FxVec2f vel_correction1 = -correction * (m1 / total_inv_mass);
            FxVec2f vel_correction2 = correction * (m2 / total_inv_mass);

            entity1->velocity.xy() += vel_correction1;
            entity2->velocity.xy() += vel_correction2;
        }
    }
}

void FxPrismaticJoint::set_force(float force) {
    set_effort(force);
}

float FxPrismaticJoint::get_position() const {
    // Transform local axis to world coordinates
    FxVec2f world_axis = m_axis.rotate_rad(entity1->pose.theta());

    // Calculate current position along axis relative to initial
    FxVec2f separation = entity2->pose.xy() - entity1->pose.xy();
    return world_axis.dot(separation) - m_initial_distance;
}

float FxPrismaticJoint::get_velocity() const {
    // Transform local axis to world coordinates
    FxVec2f world_axis = m_axis.rotate_rad(entity1->pose.theta());

    // Calculate current velocity along axis
    FxVec2f relative_velocity = entity2->velocity.xy() - entity1->velocity.xy();
    return world_axis.dot(relative_velocity);
}

void FxPrismaticJoint::apply_controls(double dt) {
    if (!enabled) return;

    // Effort mode uses the stored target directly; other modes synthesize effort through PID.
    float effort = get_effort();
    if (get_control_mode() == ControlMode::VELOCITY) {
        effort = eval_pid(m_target_velocity - get_velocity(), dt);
    } else if (get_control_mode() == ControlMode::POSITION) {
        effort = eval_pid(m_target_position - get_position(), dt);
    }

    apply_force_effort(effort);
}

// FxDistanceJoint implementation
FxDistanceJoint::FxDistanceJoint(const std::string& name, const std::shared_ptr<FxEntity>& e1,
                                 const std::shared_ptr<FxEntity>& e2, const FxVec2f& anchor1,
                                 const FxVec2f& anchor2, float min_length, float max_length) :
    FxJoint(name, e1, e2) {
    m_link = std::make_shared<FxDistanceConstraint>(e1, e2, anchor1, anchor2);

    // Unset limits take the separation the scene was authored with, so a link placed in a scene
    // holds what it was drawn holding.
    const float rest = m_link->length();
    const float lo = (min_length < 0.0f) ? rest : min_length;
    const float hi = (max_length < 0.0f) ? rest : max_length;
    if (lo > hi) {
        throw std::invalid_argument("FxDistanceJoint: min_length must not exceed max_length");
    }
    m_link->min_length = lo;
    m_link->max_length = hi;

    // A joint with no target given should do nothing, and POSITION is the default mode: start
    // the target at the rest length so the motor error is zero until someone sets one.
    m_target_length = rest;

    m_constraints.reserve(1);
    m_constraints.push_back(m_link);

    namespace_constraints();
}

void FxDistanceJoint::set_limits(float min_length, float max_length) {
    if (min_length < 0.0f) min_length = 0.0f;
    if (max_length < min_length) max_length = min_length;
    m_link->min_length = min_length;
    m_link->max_length = max_length;
    wake_entities();
}

void FxDistanceJoint::apply_force_effort(float force) {
    const FxVec2f a1 = m_link->anchor1_world();
    const FxVec2f a2 = m_link->anchor2_world();
    const FxVec2f d = a2 - a1;
    // No line to push along while the anchors coincide.
    if (d.squaredNorm() < 1e-12f) return;

    // Positive effort pushes the anchors apart, matching the prismatic joint's sign. Applied at
    // the anchors rather than the centres, so an offset anchor also twists the body.
    const FxVec2f force_vector = d.normalized() * clamp_effort(force);
    entity1->apply_force(-force_vector, a1);
    entity2->apply_force(force_vector, a2);
}

void FxDistanceJoint::set_length(float length, bool instant) {
    m_target_length = std::max(length, 0.0f);
    reset_pid_state();
    wake_entities();
    instant = instant && m_instant;

    if (instant) {
        const FxVec2f a1 = m_link->anchor1_world();
        const FxVec2f a2 = m_link->anchor2_world();
        const FxVec2f d = a2 - a1;
        const float current = d.norm();
        if (current < 1e-6f) return;

        // Move the bodies along the link until the anchors are the target distance apart,
        // splitting the correction by inverse mass so the heavier body moves less.
        const FxVec2f correction = d.normalized() * (m_target_length - current);
        const float m1 = entity1->inv_mass();
        const float m2 = entity2->inv_mass();
        const float total_inv_mass = m1 + m2;

        if (total_inv_mass > 1e-12f) {
            const FxVec2f pos_correction1 = -correction * (m1 / total_inv_mass);
            const FxVec2f pos_correction2 = correction * (m2 / total_inv_mass);

            entity1->pose.xy() += pos_correction1;
            entity2->pose.xy() += pos_correction2;

            // Shift the previous pose too, so the move registers no velocity.
            entity1->prev_pose.xy() += pos_correction1;
            entity2->prev_pose.xy() += pos_correction2;
        }
    }
}

void FxDistanceJoint::set_rate(float rate, bool instant) {
    m_target_rate = rate;
    reset_pid_state();
    wake_entities();
    instant = instant && m_instant;

    if (instant) {
        const FxVec2f d = m_link->anchor2_world() - m_link->anchor1_world();
        if (d.squaredNorm() < 1e-12f) return;
        const FxVec2f axis = d.normalized();

        const float rate_error = rate - get_rate();
        const float m1 = entity1->inv_mass();
        const float m2 = entity2->inv_mass();
        const float total_inv_mass = m1 + m2;

        if (total_inv_mass > 1e-12f) {
            const FxVec2f correction = axis * rate_error;
            entity1->velocity.xy() += -correction * (m1 / total_inv_mass);
            entity2->velocity.xy() += correction * (m2 / total_inv_mass);
        }
    }
}

void FxDistanceJoint::set_force(float force) {
    set_effort(force);
}

float FxDistanceJoint::get_length() const {
    return m_link->length();
}

float FxDistanceJoint::get_rate() const {
    const FxVec2f a1 = m_link->anchor1_world();
    const FxVec2f a2 = m_link->anchor2_world();
    const FxVec2f d = a2 - a1;
    if (d.squaredNorm() < 1e-12f) return 0.0f;
    // Anchor velocities, not centre velocities: a spinning body moves its anchor.
    const FxVec2f relative_velocity =
        entity2->velocity_at_world_point(a2) - entity1->velocity_at_world_point(a1);
    return d.normalized().dot(relative_velocity);
}

void FxDistanceJoint::apply_controls(double dt) {
    if (!enabled) return;

    // Effort mode uses the stored target directly; other modes synthesize effort through PID.
    float effort = get_effort();
    if (get_control_mode() == ControlMode::VELOCITY) {
        effort = eval_pid(m_target_rate - get_rate(), dt);
    } else if (get_control_mode() == ControlMode::POSITION) {
        // Too short is a positive error, and positive effort pushes apart.
        effort = eval_pid(m_target_length - get_length(), dt);
    }

    apply_force_effort(effort);
}

// ---------------------------------------------------------------- FxMouseJoint

bool FxMouseJoint::attach(const std::shared_ptr<FxEntity>& entity, const FxVec2f& world_point) {
    release();
    if (!entity || !entity->enabled || entity->is_sensor || entity->inv_mass() <= 0.0f)
        return false;

    m_entity = entity;
    m_local_anchor = entity->to_entity_frame(world_point);
    m_target = world_point;

    // Spring constants for this mass: k = m (2 pi f)^2, c = 2 m zeta (2 pi f). XPBD wants
    // compliance 1/k and, for damping, the coefficient beta = c; see resolve().
    const double mass = static_cast<double>(entity->mass());
    const double omega =
        2.0 * std::numbers::pi * static_cast<double>(std::max(frequency_hz, 0.01f));
    const double k = mass * omega * omega;
    m_compliance = 1.0 / k;
    m_beta = 2.0 * mass * static_cast<double>(std::max(damping_ratio, 0.0f)) * omega;
    m_max_lambda = (max_force_per_kg > 0.0f) ? max_force_per_kg * entity->mass() : FxInfinityf;

    entity->wake();
    return true;
}

void FxMouseJoint::release() {
    if (m_entity) m_entity->wake();
    m_entity.reset();
}

FxVec2f FxMouseJoint::anchor_world() const {
    return m_entity ? m_entity->to_world_frame(m_local_anchor) : FxVec2f{0.0f, 0.0f};
}

void FxMouseJoint::resolve(double dt) {
    if (!m_entity || dt <= 0.0) return;
    if (!m_entity->enabled) {
        release();
        return;
    }
    // A held body never sleeps, and any sleep it was in ends now.
    m_entity->wake();

    const FxVec2f anchor = m_entity->to_world_frame(m_local_anchor);
    const FxVec2f d = anchor - m_target;
    const float C = d.norm();
    if (C < 1e-6f) return;
    const FxVec2f n = d / C;

    // Gradient on the body: n on the centre, n . (r perp) on the angle, r the lever arm.
    const FxVec2f r = anchor - m_entity->pose.xy();
    const float gth = n.dot(r.perp());
    const float w = m_entity->inv_mass();
    const float I = m_entity->inv_inertia();

    // XPBD with damping (Macklin et al. 2016, eq. 26): the damping term acts on how far the
    // anchor moved along the constraint this substep, so it opposes velocity, not position.
    const double alpha_t = m_compliance / (dt * dt);
    const double gamma = m_compliance * m_beta / dt;
    const FxVec2f anchor_prev =
        m_entity->prev_pose.xy() + m_local_anchor.rotate_rad(m_entity->prev_pose.theta());
    const float moved = n.dot(anchor - anchor_prev);

    const double numer = -static_cast<double>(C) - gamma * static_cast<double>(moved);
    const double denom = (1.0 + gamma) * static_cast<double>(w + I * gth * gth) + alpha_t;
    if (denom <= 1e-12) return;
    double dlambda = numer / denom;

    // The multiplier is force x dt^2, so the force cap becomes a bound on it.
    const double lambda_cap = static_cast<double>(m_max_lambda) * dt * dt;
    dlambda = std::clamp(dlambda, -lambda_cap, lambda_cap);

    const float dl = static_cast<float>(dlambda);
    const FxVec2f dxy = w * dl * n;
    m_entity->apply_pose_correction(FxVec3f{dxy.x(), dxy.y(), I * dl * gth});
}
