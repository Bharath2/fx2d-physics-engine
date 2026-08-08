#include "Fx2D/Joints.h"
#include <stdexcept>

// Entity name accessor methods for FxJoint
std::string FxJoint::get_entity1_name() const {
    return entity1 ? entity1->get_name() : "";
}

std::string FxJoint::get_entity2_name() const {
    return entity2 ? entity2->get_name() : "";
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
    if (!e1 || !e2) {
        throw std::invalid_argument("Joint entities cannot be null");
    }
    if (e1.get() == e2.get()) {
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
    FxJoint(name, e1, e2),
    m_anchor_point(anchor_point),
    m_angle_min(angle_min),
    m_angle_max(angle_max) {
    m_constraints.reserve(2);

    // Create anchor constraint
    auto anchor_constraint = std::make_shared<FxAnchorConstraint>(e1, e2, anchor_point, true);
    // anchor_constraint->m_name = name + "_anchor";
    m_constraints.push_back(anchor_constraint);

    // Create angular limit constraint
    auto angular_limit = std::make_shared<FxAngularLimitConstraint>(e1, e2);
    // angular_limit->m_name = name + "_angular_limit";
    angular_limit->lower_limit = angle_min;
    angular_limit->upper_limit = angle_max;
    m_constraints.push_back(angular_limit);
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

    // Create motion along axis constraint
    auto motion_constraint = std::make_shared<FxMotionAlongAxisConstraint>(e1, e2, m_axis, true);
    // motion_constraint->m_name = name + "_motion";
    m_constraints.push_back(motion_constraint);

    // Create separation constraint
    auto separation_constraint = std::make_shared<FxSeparationConstraint>(e1, e2, m_axis, true);
    // separation_constraint->m_name = name + "_separation";
    separation_constraint->lower_limit = m_position_min;
    separation_constraint->upper_limit = m_position_max;
    m_constraints.push_back(separation_constraint);

    // Create angle lock constraint
    auto angle_lock = std::make_shared<FxAngleLockConstraint>(e1, e2, 0.0f);
    // angle_lock->m_name = name + "_angle_lock";
    m_constraints.push_back(angle_lock);
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
