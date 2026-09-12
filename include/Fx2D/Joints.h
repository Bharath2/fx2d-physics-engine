#pragma once

#include "Fx2D/Entity.h"
#include "Fx2D/Solver.h"
#include <memory>
#include <string>
#include <unordered_map>

// Motor control mode: track a position, velocity, or direct effort target.
enum class ControlMode { POSITION, VELOCITY, EFFORT };

// Base joint class that manages relationships between entities and constraints
class FxJoint {
  protected:
    std::shared_ptr<FxEntity> entity1;
    std::shared_ptr<FxEntity> entity2;
    std::string m_name; // Joint name for identification
    std::vector<std::shared_ptr<FxConstraint>> m_constraints; // Constraints that define this joint

    // PID control parameters
    FxVec3f m_pid{1.0f, 0.0f, 0.0f}; // {p, i, d}
    float m_integral = 0.0f; // Integral accumulator
    float m_previous_error = 0.0f; // Previous error for derivative calculation
    float m_target_effort = 0.0f; // Shared effort target for torque/force control
    bool m_instant = true; // Whether to apply controls instantly or use PID

    // Renames owned constraints to <joint>_<Type>, appending _N on same-type repeats.
    // Joint names are unique in the registry, so constraint names are unique by construction
    // and stable under entity renames.
    void namespace_constraints();
    void reset_pid_state();
    void wake_entities();
    float eval_pid(float error, double dt);
    float clamp_effort(float effort) const;

  public:
    bool enabled = true; // Whether joint is enabled
    bool entities_collide = false; // Whether connected entities should collide

    FxJoint(const std::string& name, const std::shared_ptr<FxEntity>& e1,
            const std::shared_ptr<FxEntity>& e2);
    virtual ~FxJoint() = default;

    // Accessor method for name
    const std::string& get_name() const { return m_name; }
    // Clears integrator windup and derivative history; gains and targets are settings, kept.
    void reset() { reset_pid_state(); }

    // Entity accessor methods (read-only)
    const std::shared_ptr<FxEntity>& get_entity1() const { return entity1; }
    const std::shared_ptr<FxEntity>& get_entity2() const { return entity2; }

    // Entity name accessor methods
    std::string get_entity1_name() const;
    std::string get_entity2_name() const;

    // Constraints accessor
    const std::vector<std::shared_ptr<FxConstraint>>& get_constraints() const {
        return m_constraints;
    }

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
    float m_target_theta = 0.0f; // Target angle for PID control
    float m_target_omega = 0.0f; // Target angular velocity for PID control

    void apply_torque_effort(float torque);

  public:
    FxRevoluteJoint(const std::string& name, const std::shared_ptr<FxEntity>& e1,
                    const std::shared_ptr<FxEntity>& e2, const FxVec2f& anchor_point,
                    float angle_min = -3.14159f, float angle_max = 3.14159f);

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
    FxVec2f m_axis; // Local axis on entity1 (normalized)
    float m_initial_distance; // Initial distance projection along axis
    float m_position_min, m_position_max; // Position limits along axis
    float m_target_position = 0.0f; // Target position for PID control
    float m_target_velocity = 0.0f; // Target velocity for PID control

    void apply_force_effort(float force);

  public:
    FxPrismaticJoint(const std::string& name, const std::shared_ptr<FxEntity>& e1,
                     const std::shared_ptr<FxEntity>& e2, const FxVec2f& local_axis,
                     float position_min = -1000.0f, float position_max = 1000.0f);

    // Type checking override
    bool is_prismatic() const override { return true; }

    // Control methods - set targets for PID control
    void set_position(float position, bool instant = true); // Set target position along axis
    void set_velocity(float velocity, bool instant = true); // Set target velocity along axis
    void set_force(float force); // Apply force along axis

    // Query methods
    float get_position() const; // Get current relative position along axis
    float get_velocity() const; // Get current relative velocity along axis
    void set_max_force(float max_force) { set_max_effort(max_force); }
    float get_max_force() const { return get_max_effort(); }

    // Apply controls method
    void apply_controls(double dt) override;
};

// A soft spring from a world point to an anchor on one body, for click-dragging. Not an
// FxJoint: it pairs a body with a point rather than two bodies, so it lives outside the joint
// registry and is transient. FxScene owns one; reset() and deleting the held entity release it.
//
// Tuned like a damped spring rather than by raw compliance, so a drag feels the same on a
// 0.1 kg ball and a 50 kg crate: stiffness and damping scale with the body's mass on attach.
class FxMouseJoint {
  private:
    std::shared_ptr<FxEntity> m_entity;
    FxVec2f m_local_anchor{0.0f, 0.0f};
    FxVec2f m_target{0.0f, 0.0f};
    double m_compliance = 0.0; // XPBD alpha for the attached body, from frequency and mass
    double m_beta = 0.0; // XPBD damping coefficient, from damping ratio and mass
    float m_max_lambda = FxInfinityf; // force cap expressed as a Lagrange-multiplier bound

  public:
    // Undamped natural frequency of the spring, in Hz. Higher pulls harder.
    float frequency_hz = 5.0f;
    // 0 oscillates freely, 1 is critically damped.
    float damping_ratio = 0.7f;
    // Force cap as a multiple of the body's weight-equivalent (mass x this, in N per kg), so
    // a body pinned against a wall cannot be driven through it.
    float max_force_per_kg = 1000.0f;

    // Grabs `entity` at `world_point`, which becomes the anchor. Static (zero inverse mass),
    // disabled and sensor bodies are refused. Returns whether the joint is now attached.
    bool attach(const std::shared_ptr<FxEntity>& entity, const FxVec2f& world_point);
    // Moves the point the anchor is pulled toward.
    void set_target(const FxVec2f& world_point) { m_target = world_point; }
    void release();

    bool attached() const { return m_entity != nullptr; }
    const std::shared_ptr<FxEntity>& entity() const { return m_entity; }
    const FxVec2f& target() const { return m_target; }
    const FxVec2f& local_anchor() const { return m_local_anchor; }
    // Anchor in world coordinates at the body's current pose; zero when detached.
    FxVec2f anchor_world() const;

    // One damped XPBD position correction toward the target. Called by FxScene each substep.
    void resolve(double dt);
};
