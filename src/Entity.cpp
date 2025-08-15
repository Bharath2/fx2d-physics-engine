#include "Entity.h"

// FxEntity contructor with name validation
FxEntity::FxEntity(const std::string &entityName) : name(entityName) {
    static const std::regex pattern{"^[A-Za-z0-9_]+$"};  // alphanumerics charaters or underscores are only allowed
    if (!std::regex_match(name, pattern)) {
        throw std::invalid_argument("FxEntity: Entity name must be alphanumeric.");
    }
}

// Setter for mass: Enforces inertia >= 0.
void FxEntity::set_mass(const float& mass) {
    if (mass < 0.0f ) {
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
    m_eff_force = {0.0f, 0.0f};
    m_eff_moment = 0.0f;
    m_eff_impulse = {0.0f, 0.0f};
    m_eff_impulse_moment = 0.0f;
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
        _inv_mass = 0.0f;  // Disable external forces by setting inverse mass to zero
        _inv_inertia = 0.0f; // Disable external torques by setting inverse inertia to zero
    } else {
        _inv_mass = (_mass > 0.0f) ? 1.0f / _mass : 0.0f; // Recalculate inverse mass
        _inv_inertia = (_inertia > 0.0f) ? 1.0f / _inertia : 0.0f; // Recalculate inverse inertia
    }
}

// Apply force at center of mass, affecting linear acceleration
void FxEntity::apply_force(const FxVec2f& force) {
    if (_inv_mass > 0.0f) {
        m_eff_force += force; 
    }
}

// Apply force at an arbitrary point, contributes linear and angular effects
void FxEntity::apply_force(const FxVec2f& force, const FxVec2f& contact_point) {
    float torque = 0.0f;
    if (_inv_mass > 0.0f) {
        m_eff_force += force;
    }
    if (_inv_inertia > 0.0f) {
        FxVec2f r = contact_point - pose.xy(); // r is the vector from center of mass to contact point
        torque = r.cross(force);
        m_eff_moment += torque;
    }
}

// Directly apply moment
void FxEntity::apply_torque(float torque) {
    if (_inv_inertia > 0.0f) {
        m_eff_moment += torque;
    }
}

// For impulse applied at center of mass, accumulate for step application
void FxEntity::apply_impulse(const FxVec2f& impulse) {
    if (_inv_mass > 0.0f) {
        m_eff_impulse += impulse;   // accumulate impulse for step application
    }
}

// Apply impulse at an arbitrary point, accumulate for step application
void FxEntity::apply_impulse(const FxVec2f& impulse, const FxVec2f& contact_point) {
    if (_inv_mass > 0.0f) {
        m_eff_impulse += impulse; // accumulate impulse for step application
    }
    if (_inv_inertia > 0.0f) {
        FxVec2f r = contact_point - pose.xy(); // r is the vector from center of mass to contact point
        float torque = r.cross(impulse);
        m_eff_impulse_moment += torque; // accumulate impulse moment for step application
    }
}

// Get instantaneous velocity at a specific position
FxVec2f FxEntity::velocity_at_world_point(const FxVec2f& position) const {
    FxVec2f r = position - pose.xy(); // vector from center of mass to position
    return velocity.xy() + FxVec2f(-r.y() * velocity.theta(), r.x() * velocity.theta());
}

// Get instantaneous velocity at a local point (relative to entity's center)
FxVec2f FxEntity::velocity_at_local_point(const FxVec2f& local_position) const {
    return velocity.xy() + FxVec2f(-local_position.y() * velocity.theta(), local_position.x() * velocity.theta());
}

// Calculte the effect of all forces and moments with no gravity
FxVec3f FxEntity::calc_acceleration() {
    return calc_acceleration({0.0f, 0.0f});  // no gravity
}

// Calculte the effect of all forces and moments
FxVec3f FxEntity::calc_acceleration(const FxVec2f& gravity) {
    FxVec3f acc{0.0f, 0.0f, 0.0f};
    if (_mass > 0.0f) {
        acc.xy() += m_eff_force * _inv_mass;  // Linear acceleration
        acc.xy() += gravity_scale * gravity;   // Gravity effect
    }
    if (_inertia > 0.0f) {
        acc.theta() += m_eff_moment * _inv_inertia;  // Angular acceleration
    }
    return acc;
}

// Returns the axis aligned bounding box in world coordinates
const FxArray<float> FxEntity::bounding_box() const {
    return m_bounding_box; 
}

// Broad phase check: if two entities axis aligned bounding boxes overlap  
bool FxEntity::aabb_overlap_check(const FxEntity& other) const {
    auto aabb1 =       bounding_box();
    auto aabb2 = other.bounding_box();
    // check if they are overlapping
    return !(aabb1(2) < aabb2(0) || aabb2(2) < aabb1(0) || // this.maxX < other.minX or other.maxX < this.minX
             aabb1(3) < aabb2(1) || aabb2(3) < aabb1(1));  // this.maxY < other.minY or  other.maxY < this.minY
}

// Overload of aabb_overlap_check() that accepts a shared pointer.
bool FxEntity::aabb_overlap_check(const std::shared_ptr<FxEntity>& other) const {
    if (!other) return false;
    return aabb_overlap_check(*other);
}

void FxEntity::step(const FxVec2f& gravity, const float& step_dt){
    // Apply accumulated impulses to velocity
    if (_inv_mass > 0.0f) {
        velocity.xy() += m_eff_impulse * _inv_mass;
    }
    if (_inv_inertia > 0.0f) {
        velocity.theta() += m_eff_impulse_moment * _inv_inertia;
    }

    FxVec3f half_step_acc =  0.5*calc_acceleration(gravity);
    prev_pose = pose - half_step_acc * step_dt * step_dt;  
    prev_velocity = velocity;  
    velocity += half_step_acc * step_dt;
    pose += velocity * step_dt;
    
    // Normalize theta to [0, 2*pi) range
    while (pose.theta() < 0.0f) {
        pose.theta() += 2.0f * FxPif;
    }
    while (pose.theta() >= 2.0f * FxPif) {
        pose.theta() -= 2.0f * FxPif;
    }
    
    // Safety check: prevent invalid positions
    if (std::isnan(pose.x()) || std::isnan(pose.y()) || std::isnan(pose.theta()) ||
        std::abs(pose.x()) > 1000.0f || std::abs(pose.y()) > 1000.0f) {
        // Reset to safe position if invalid
        pose = _init_pose;
        velocity = {0.0f, 0.0f, 0.0f};
    }
    
    // update pose of the collision shape and visual shape
    if(m_collision != nullptr) {
        m_bounding_box = m_collision->set_world_pose(pose);
    }
    if(m_visual != nullptr) {
        m_visual->set_world_pose(pose); 
    }
    
    // reset effective force, moment, and impulses
    m_eff_force = {0.0f, 0.0f}; // reset effective force
    m_eff_moment = 0.0f; // reset effective moment
    m_eff_impulse = {0.0f, 0.0f}; // reset effective impulse
    m_eff_impulse_moment = 0.0f; // reset effective impulse moment
} 