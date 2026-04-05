#pragma once

#include "Fx2D/Entity.h"
#include "Fx2D/Solver.h"
#include <memory>
#include <string>


// Motor control mode: track a position, velocity, or direct effort target.
enum class ControlMode { POSITION, VELOCITY, EFFORT };

// Base joint class that manages relationships between entities and constraints
class FxJoint {
protected:
    std::shared_ptr<FxEntity> entity1;
    std::shared_ptr<FxEntity> entity2;
    std::string m_name;       // Joint name for identification
    std::vector<std::shared_ptr<FxConstraint>> m_constraints; // Constraints that define this joint
    
    // PID control parameters
    FxVec3f m_pid {1.0f, 0.0f, 0.0f}; // {p, i, d}
    float m_integral = 0.0f;  // Integral accumulator
    float m_previous_error = 0.0f; // Previous error for derivative calculation
    float m_target_effort = 0.0f; // Shared effort target for torque/force control
    bool m_instant = true;    // Whether to apply controls instantly or use PID

    void reset_pid_state();
    void wake_entities();
    float eval_pid(float error, double dt);
    float clamp_effort(float effort) const;
    
public:
    bool enabled = true;    // Whether joint is enabled
    bool entities_collide = false;  // Whether connected entities should collide
    
    FxJoint(const std::string& name, const std::shared_ptr<FxEntity>& e1, const std::shared_ptr<FxEntity>& e2);
    virtual ~FxJoint() = default;

    // Accessor method for name
    const std::string& get_name() const { return m_name; }
    
    // Entity accessor methods (read-only)
    const std::shared_ptr<FxEntity>& get_entity1() const { return entity1; }
    const std::shared_ptr<FxEntity>& get_entity2() const { return entity2; }
    
    // Entity name accessor methods
    std::string get_entity1_name() const;
    std::string get_entity2_name() const;
    
    // Constraints accessor
    const std::vector<std::shared_ptr<FxConstraint>>& get_constraints() const { return m_constraints; }
    
    // PID control methods
    void set_pid(const FxVec3f& pid);
    FxVec3f get_pid() const { return m_pid; }
    void set_p(float p) { m_pid.x() = p; }
    void set_i(float i) { m_pid.y() = i; }
    void set_d(float d) { m_pid.z() = d; }
    float get_p() const { return m_pid.x(); }
    float get_i() const { return m_pid.y(); }
    float get_d() const { return m_pid.z(); }
    void set_instant(bool instant) { m_instant = instant; }
    bool get_instant() const { return m_instant; }
    void set_effort(float effort);
    float get_effort() const { return m_target_effort; }
    void set_max_effort(float max_effort);
    float get_max_effort() const { return m_max_effort; }
    void set_control_mode(ControlMode mode);
    ControlMode get_control_mode() const { return m_control_mode; }
    
    // Type checking methods
    virtual bool is_revolute() const { return false; }
    virtual bool is_prismatic() const { return false; }
    
    // Virtual apply_controls method for applying joint controls
    virtual void apply_controls(double dt) = 0;

private:
    float m_max_effort = FxInfinityf;
    ControlMode m_control_mode = ControlMode::POSITION;
};

// Revolute joint with anchor and angular limit constraints
class FxRevoluteJoint : public FxJoint {
private:
    FxVec2f m_anchor_point; // Anchor point in entity1's local coordinates
    float m_angle_min, m_angle_max; // Angular limits
    float m_target_theta = 0.0f;    // Target angle for PID control
    float m_target_omega = 0.0f;    // Target angular velocity for PID control

    void apply_torque_effort(float torque);
    
public:
    FxRevoluteJoint(const std::string& name, const std::shared_ptr<FxEntity>& e1, const std::shared_ptr<FxEntity>& e2, 
                    const FxVec2f& anchor_point, float angle_min = -3.14159f, float angle_max = 3.14159f);
    
    // Type checking override
    bool is_revolute() const override { return true; }
    
    // Control methods - set targets for PID control
    void set_theta(float angle, bool instant = true);
    void set_omega(float omega, bool instant = true);
    void set_torque(float torque);
    
    // Query methods
    float get_theta() const;
    float get_omega() const;
    void set_max_torque(float max_torque) { set_max_effort(max_torque); }
    float get_max_torque() const { return get_max_effort(); }

    // Apply controls method
    void apply_controls(double dt) override;
};

// Prismatic joint with motion, separation, and angle lock constraints
class FxPrismaticJoint : public FxJoint {
private:
    FxVec2f m_axis;         // Local axis on entity1 (normalized)
    float m_initial_distance; // Initial distance projection along axis
    float m_position_min, m_position_max; // Position limits along axis
    float m_target_position = 0.0f;    // Target position for PID control
    float m_target_velocity = 0.0f;    // Target velocity for PID control

    void apply_force_effort(float force);
    
public:
    FxPrismaticJoint(const std::string& name, const std::shared_ptr<FxEntity>& e1, const std::shared_ptr<FxEntity>& e2, 
                     const FxVec2f& local_axis, float position_min = -1000.0f, float position_max = 1000.0f);
    
    // Type checking override
    bool is_prismatic() const override { return true; }
    
    // Control methods - set targets for PID control
    void set_position(float position, bool instant = true);      // Set target position along axis
    void set_velocity(float velocity, bool instant = true);      // Set target velocity along axis
    void set_force(float force);            // Apply force along axis
    
    // Query methods
    float get_position() const;     // Get current relative position along axis
    float get_velocity() const;     // Get current relative velocity along axis
    void set_max_force(float max_force) { set_max_effort(max_force); }
    float get_max_force() const { return get_max_effort(); }

    // Apply controls method
    void apply_controls(double dt) override;
};
