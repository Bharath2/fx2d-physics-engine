#pragma once

#include "Fx2D/Entity.h"
#include "Fx2D/Solver.h"
#include <memory>
#include <string>


// Base joint class that manages relationships between entities and constraints
class FxJoint {
protected:
    std::shared_ptr<FxEntity> entity1;
    std::shared_ptr<FxEntity> entity2;
    
public:
    std::string name;       // Joint name for identification
    bool entities_collide = false;  // Whether connected entities should collide
    
    FxJoint(std::shared_ptr<FxEntity> e1, std::shared_ptr<FxEntity> e2, const std::string& joint_name);
    virtual ~FxJoint() = default;
    
    // Entity access
    std::shared_ptr<FxEntity> get_entity1() const { return entity1; }
    std::shared_ptr<FxEntity> get_entity2() const { return entity2; }
};

// Revolute joint that directly controls entity states (not constraint-based)
class FxRevoluteJoint : public FxJoint {
public:
    FxRevoluteJoint(std::shared_ptr<FxEntity> e1, std::shared_ptr<FxEntity> e2);
    
    // Control methods - directly apply changes instantly
    void set_theta(float angle);
    void set_omega(float omega);
    void set_torque(float torque);
    
    // Query methods
    float get_theta() const;
    float get_omega() const;
};

// Prismatic joint that directly controls linear motion along an axis
class FxPrismaticJoint : public FxJoint {
private:
    FxVec2f m_axis;         // Local axis on entity1 (normalized)
    float m_initial_distance; // Initial distance projection along axis
    
public:
    FxPrismaticJoint(std::shared_ptr<FxEntity> e1, std::shared_ptr<FxEntity> e2, const FxVec2f& local_axis);
    
    // Control methods - directly apply changes instantly
    void set_position(float position);      // Set relative position along axis
    void set_velocity(float velocity);      // Set relative velocity along axis
    void set_force(float force);            // Apply force along axis
    
    // Query methods
    float get_position() const;     // Get current relative position along axis
    float get_velocity() const;     // Get current relative velocity along axis
};
