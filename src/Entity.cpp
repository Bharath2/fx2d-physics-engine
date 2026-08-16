#include "Fx2D/Entity.h"

FxEntity::FxEntity(const std::string& entityName) : m_name(entityName) {
    if (!is_valid_name(m_name)) {
        throw std::invalid_argument("FxEntity: Entity name must be alphanumeric or underscore.");
    }
}

// Setter for mass: Enforces inertia >= 0.
void FxEntity::set_mass(const float& mass) {
    if (mass < 0.0f) {
        throw std::invalid_argument("FxEntity: Mass must be non-negative");
    } else if (mass > 1e5f) {
        throw std::invalid_argument("FxEntity: Mass can not be greater than 1e5");
    }
    _mass = mass;
    _inv_mass = (mass == 0.0f) ? 0.0f : 1.0f / mass;
}

// Setter for inertia: Enforces inertia >= 0.
void FxEntity::set_inertia(const float& inertia) {
    if (inertia < 0.0f) {
        throw std::invalid_argument("FxEntity: Inertia must be non-negative");
    }
    _inertia = inertia;
    _inv_inertia = (inertia == 0.0f) ? 0.0f : 1.0f / inertia;
}

// Setter for inertia: Calculates from visual shape
void FxEntity::set_inertia() {
    if (_mass <= 0.0f || !m_visual) {
        _inertia = 0.0f;
        _inv_inertia = 0.0f;
        return;
    }
    _inertia = m_visual->calc_inertia(_mass);
    _inv_inertia = (_inertia == 0.0f) ? 0.0f : 1.0f / _inertia;
}

// resets current state to inital state
void FxEntity::reset() {
    pose = _init_pose;
    velocity = _init_velocity;
    prev_pose = _init_pose;
    prev_velocity = _init_velocity;
    m_pose_carry = {0, 0, 0};
    m_correction_carry = {0, 0, 0};
    m_eff_force = {0.0f, 0.0f};
    m_eff_moment = 0.0f;
    m_eff_impulse = {0.0f, 0.0f};
    m_eff_impulse_moment = 0.0f;
    // A body asleep before the reset must not stay frozen after it.
    m_sleeping = false;
    m_sleep_timer = 0.0f;
    // Push the restored pose into the shapes so the cached bounds are not left stale.
    if (m_collision) m_bounding_box = m_collision->set_world_pose(pose);
    if (m_visual) m_visual->set_world_pose(pose);
}

// method to set initial pose
void FxEntity::set_init_pose(const FxVec3f& o_pose) {
    _init_pose = o_pose;
    pose = o_pose;
    prev_pose = o_pose;
}

// method to set initial velocity
void FxEntity::set_init_velocity(const FxVec3f& o_velocity) {
    _init_velocity = o_velocity;
    velocity = o_velocity;
    prev_velocity = o_velocity;
}

// Enable or disable external forces and torques, including effects due to collisions
void FxEntity::enable_external_forces(bool enable) {
    if (!enable) {
        _inv_mass = 0.0f; // Disable external forces by setting inverse mass to zero
        _inv_inertia = 0.0f; // Disable external torques by setting inverse inertia to zero
    } else {
        _inv_mass = (_mass > 0.0f) ? 1.0f / _mass : 0.0f; // Recalculate inverse mass
        _inv_inertia = (_inertia > 0.0f) ? 1.0f / _inertia : 0.0f; // Recalculate inverse inertia
    }
}

// Apply force at center of mass, affecting linear acceleration
void FxEntity::apply_force(const FxVec2f& force) {
    wake();
    if (_inv_mass > 0.0f) {
        m_eff_force += force;
    }
}

// Apply force at an arbitrary point, contributes linear and angular effects
void FxEntity::apply_force(const FxVec2f& force, const FxVec2f& contact_point) {
    wake();
    float torque = 0.0f;
    if (_inv_mass > 0.0f) {
        m_eff_force += force;
    }
    if (_inv_inertia > 0.0f) {
        FxVec2f r = contact_point - pose.xy(); // r is the vector from center of mass to contact
                                               // point
        torque = r.cross(force);
        m_eff_moment += torque;
    }
}

// Directly apply moment
void FxEntity::apply_torque(float torque) {
    wake();
    if (_inv_inertia > 0.0f) {
        m_eff_moment += torque;
    }
}

// For impulse applied at center of mass, accumulate for step application
void FxEntity::apply_impulse(const FxVec2f& impulse) {
    wake();
    if (_inv_mass > 0.0f) {
        m_eff_impulse += impulse; // accumulate impulse for step application
    }
}

// Apply impulse at an arbitrary point, accumulate for step application
void FxEntity::apply_impulse(const FxVec2f& impulse, const FxVec2f& contact_point) {
    wake();
    if (_inv_mass > 0.0f) {
        m_eff_impulse += impulse; // accumulate impulse for step application
    }
    if (_inv_inertia > 0.0f) {
        FxVec2f r = contact_point - pose.xy(); // r is the vector from center of mass to contact
                                               // point
        float torque = r.cross(impulse);
        m_eff_impulse_moment += torque; // accumulate impulse moment for step application
    }
}

// Advance sleep timer; put entity to sleep when below threshold long enough
void FxEntity::tick_sleep(float dt) {
    if (_inv_mass == 0.0f) {
        m_sleeping = false;
        m_sleep_timer = 0.0f;
        return;
    } // static bodies never sleep
    float speed_lin = velocity.head<2>().norm();
    float speed_ang = std::abs(velocity.theta());
    if (speed_lin < sleep_threshold_linear && speed_ang < sleep_threshold_angular) {
        m_sleep_timer += dt;
        if (m_sleep_timer >= sleep_time_required) sleep();
    } else {
        m_sleep_timer = 0.0f;
    }
}

// Get instantaneous velocity at a specific position
FxVec2f FxEntity::velocity_at_world_point(const FxVec2f& position) const {
    FxVec2f r = position - pose.xy(); // vector from center of mass to position
    return velocity.xy() + velocity.theta() * r.perp();
}

// Get instantaneous velocity at a local point (relative to entity's center)
FxVec2fArray FxEntity::velocity_at_local_point(const FxVec2fArray& local_position) const {
    return velocity.xy() + velocity.theta() * local_position.perp();
}

// Get instantaneous velocity at a specific position
FxVec2fArray FxEntity::velocity_at_world_point(const FxVec2fArray& position) const {
    auto r = position - pose.xy(); // vector from center of mass to position
    return velocity.xy() + velocity.theta() * r.perp();
}

// Get instantaneous velocity at a local point (relative to entity's center)
FxVec2f FxEntity::velocity_at_local_point(const FxVec2f& local_position) const {
    return velocity.xy() + velocity.theta() * local_position.perp();
}

// Convert local point to world coordinates
FxVec2f FxEntity::to_world_frame(const FxVec2f& local_point) const {
    // Rotate local point by entity's orientation and translate by entity's position
    return pose.xy() + local_point.rotate_rad(pose.theta());
}

// Convert world point to local coordinates
FxVec2f FxEntity::to_entity_frame(const FxVec2f& world_point) const {
    // Translate by entity's position and rotate by negative entity's orientation
    FxVec2f translated = world_point - pose.xy();
    return translated.rotate_rad(-pose.theta());
}

// Calculte the effect of all forces and moments with no gravity
FxVec3f FxEntity::calc_acceleration() {
    return calc_acceleration({0.0f, 0.0f}); // no gravity
}

// Calculte the effect of all forces and moments
FxVec3f FxEntity::calc_acceleration(const FxVec2f& gravity) {
    FxVec3f acc{0.0, 0.0, 0.0};
    if (_mass > 0.0f) {
        acc.xy() += m_eff_force * _inv_mass; // Linear acceleration
        acc.xy() += gravity_scale * gravity; // Gravity effect
    }
    if (_inertia > 0.0f) {
        acc.theta() += m_eff_moment * _inv_inertia; // Angular acceleration
    }
    return acc;
}

// Returns the axis aligned bounding box in world coordinates
const FxArray<float> FxEntity::bounding_box() const {
    return m_bounding_box;
}

// Broad phase check: if two entities axis aligned bounding boxes overlap
bool FxEntity::aabb_overlap_check(const FxEntity& other) const {
    auto aabb1 = bounding_box();
    auto aabb2 = other.bounding_box();
    // check if they are overlapping
    return !(aabb1(2) < aabb2(0) || aabb2(2) < aabb1(0) || // this.maxX < other.minX or other.maxX <
                                                           // this.minX
             aabb1(3) < aabb2(1) || aabb2(3) < aabb1(1)); // this.maxY < other.minY or  other.maxY <
                                                          // this.minY
}

// Overload of aabb_overlap_check() that accepts a shared pointer.
bool FxEntity::aabb_overlap_check(const std::shared_ptr<FxEntity>& other) const {
    if (!other) return false;
    return aabb_overlap_check(*other);
}

// Update pose from velocity using mixed precision
void FxEntity::__update_pose(const double& step_dt) {
    FxCarryAdd(pose.x(), (double)velocity.x() * step_dt, m_pose_carry.x());
    FxCarryAdd(pose.y(), (double)velocity.y() * step_dt, m_pose_carry.y());
    FxCarryAdd(pose.theta(), (double)velocity.theta() * step_dt, m_pose_carry.theta());
    // wrap to [-pi, pi) when incremented
    pose.set_theta(FxAngleWrap(pose.theta()));
}

// Mixed-precision correction, reporting how much survived rounding into the float pose.
FxVec3f FxEntity::apply_pose_correction(const FxVec3f& delta) {
    FxVec3f applied{0.0f, 0.0f, 0.0f};
    applied.x() = FxCarryAdd(pose.x(), (double)delta.x(), m_correction_carry.x());
    applied.y() = FxCarryAdd(pose.y(), (double)delta.y(), m_correction_carry.y());
    applied.theta() = FxCarryAdd(pose.theta(), (double)delta.theta(), m_correction_carry.theta());
    return applied;
}

void FxEntity::step(const FxVec2f& gravity, const double& step_dt) {
    // Apply accumulated impulses to velocity
    if (_inv_mass > 0.0f) {
        velocity.xy() += m_eff_impulse * _inv_mass;
    }
    if (_inv_inertia > 0.0f) {
        velocity.theta() += m_eff_impulse_moment * _inv_inertia;
    }

    // Update pose and velocity using float precision
    prev_pose = pose;
    prev_velocity = velocity;
    velocity += calc_acceleration(gravity) * step_dt;
    __update_pose(step_dt); // pose += velocity * step_dt;

    // update pose of the collision shape and visual shape
    if (m_collision != nullptr) {
        m_bounding_box = m_collision->set_world_pose(pose);
    }
    if (m_visual != nullptr) {
        m_visual->set_world_pose(pose);
    }

    // reset effective force, moment, and impulses
    m_eff_force = {0.0f, 0.0f}; // reset effective force
    m_eff_moment = 0.0f; // reset effective moment
    m_eff_impulse = {0.0f, 0.0f}; // reset effective impulse
    m_eff_impulse_moment = 0.0f; // reset effective impulse moment
}