#include "Fx2D/Joints.h"

#include <cmath>
#include <stdexcept>

// FxJoint base class implementation
FxJoint::FxJoint(std::shared_ptr<FxEntity> e1, std::shared_ptr<FxEntity> e2, const std::string& joint_name) {
    if (!e1 || !e2) {
        throw std::invalid_argument("Joint entities cannot be null");
    }
    if (e1.get() == e2.get()) {
        throw std::invalid_argument("Joint cannot connect an entity to itself");
    }
    entity1 = e1;
    entity2 = e2;
    name = joint_name;
}

// FxRevoluteJoint implementation
FxRevoluteJoint::FxRevoluteJoint(std::shared_ptr<FxEntity> e1, std::shared_ptr<FxEntity> e2, const FxVec2f& origin)
    : FxJoint(e1, e2, e1->get_name() + "_" + e2->get_name() + "_RevoluteJoint"), joint_origin(origin) {
}

void FxRevoluteJoint::set_theta(float angle) {
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
        
        entity1->pose.theta() += angle_correction1;
        entity2->pose.theta() += angle_correction2;
        
        // Also update previous pose to maintain consistency
        entity1->prev_pose.theta() += angle_correction1;
        entity2->prev_pose.theta() += angle_correction2;
    }
}

void FxRevoluteJoint::set_omega(float omega) {
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

void FxRevoluteJoint::set_torque(float torque) {
    // Apply torque as angular impulse to effective impulse moments
    // This will be applied during the entity's step() method
    entity1->m_eff_impulse_moment -= torque * entity1->inv_inertia();
    entity2->m_eff_impulse_moment += torque * entity2->inv_inertia();
}

float FxRevoluteJoint::get_theta() const {
    return FxAngleWrap(entity2->pose.theta() - entity1->pose.theta());
}

float FxRevoluteJoint::get_omega() const {
    return entity2->velocity.theta() - entity1->velocity.theta();
}


// FxPrismaticJoint implementation
FxPrismaticJoint::FxPrismaticJoint(std::shared_ptr<FxEntity> e1, std::shared_ptr<FxEntity> e2, const FxVec2f& local_axis)
    : FxJoint(e1, e2, e1->get_name() + "_" + e2->get_name() + "_PrismaticJoint") {
    m_axis = local_axis.normalized();
    
    // Store initial distance projection along the axis
    FxVec2f world_axis = m_axis.rotate_rad(entity1->pose.theta());
    FxVec2f separation = entity2->pose.xy() - entity1->pose.xy();
    m_initial_distance = world_axis.dot(separation);
}

void FxPrismaticJoint::set_position(float position) {
    if (!enabled || !is_valid()) return;
    
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

void FxPrismaticJoint::set_velocity(float velocity) {
    if (!enabled || !is_valid()) return;
    
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

void FxPrismaticJoint::set_force(float force) {
    if (!enabled || !is_valid()) return;
    
    // Transform local axis to world coordinates
    FxVec2f world_axis = m_axis.rotate_rad(entity1->pose.theta());
    
    // Apply force as impulse to effective force system
    FxVec2f force_vector = world_axis * force;
    entity1->m_eff_impulse -= force_vector * entity1->inv_mass();
    entity2->m_eff_impulse += force_vector * entity2->inv_mass();
}

float FxPrismaticJoint::get_position() const {
    if (!is_valid()) return 0.0f;
    
    // Transform local axis to world coordinates
    FxVec2f world_axis = m_axis.rotate_rad(entity1->pose.theta());
    
    // Calculate current position along axis relative to initial
    FxVec2f separation = entity2->pose.xy() - entity1->pose.xy();
    return world_axis.dot(separation) - m_initial_distance;
}

float FxPrismaticJoint::get_velocity() const {
    if (!is_valid()) return 0.0f;
    
    // Transform local axis to world coordinates
    FxVec2f world_axis = m_axis.rotate_rad(entity1->pose.theta());
    
    // Calculate current velocity along axis
    FxVec2f relative_velocity = entity2->velocity.xy() - entity1->velocity.xy();
    return world_axis.dot(relative_velocity);
}
