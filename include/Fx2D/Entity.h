
#pragma once 

#include <memory>
#include <string>
#include <regex>
#include <stdexcept>

#include "Fx2D/Math.h"

// container for visual shape 
struct FxVisualShape : public FxShape {
    private:    
        FxVec4ui8 m_fillColor {200, 200, 200, 255};    // Fill color (RGBA)
        FxVec4ui8 m_outlineColor {10, 10, 10, 255};    // Outline color (RGBA)
        float m_outlineThickness = 2.5f;               // Outline thickness in pixels
        std::string m_fillTexturePath = "";                // File path to texture file (empty = no texture)
    public:
        // 1. "Inherit" all of FxShape's ctors
        using FxShape::FxShape;
        // 2. Conversion‐style ctor: build a VisualShape from any Shape
        FxVisualShape(const FxShape& base) : FxShape(base) {}
        // Method to set a fillColor, outlineColor and texture
        void set_fillColor(const FxVec4ui8& color) { m_fillColor = color; m_fillTexturePath = ""; }
        void set_outlineColor(const FxVec4ui8& color) { m_outlineColor = color; }
        void set_outlineThickness(const float& thickness){ m_outlineThickness = thickness; }
        void set_fillTexture(const std::string& filePath) { m_fillTexturePath = filePath; }
        // getters for all the attributes
        const FxVec4ui8& fillColor() const {return m_fillColor;}
        const std::string& fillTexture() const {return m_fillTexturePath;}
        const FxVec4ui8& outlineColor() const {return m_outlineColor;}
        float outlineThickness() const {return m_outlineThickness;}
};

// container for collision shape
using FxCollisionShape = FxShape;

// Class for entity attributes and methods
class FxEntity {
  private:
    // unique identifier assigned by scene
    size_t m_entity_id = 0;
    
    // mass and inertia
    float _mass = 1.0f;     
    float _inertia = 1.0f;   // around center of mass
    float _inv_mass = 0.125f;
    float _inv_inertia = 8.0f;
    
    // store initial state for reset
    FxVec3f _init_pose {0, 0, 0};
    FxVec3f _init_velocity {0, 0, 0};
    FxVec3d m_pose_carry {0, 0, 0};
    
    // visual and collision shapes
    std::shared_ptr<FxCollisionShape> m_collision;
    std::shared_ptr<FxVisualShape> m_visual = std::make_shared<FxVisualShape>();

    // total resultant force and moment on the body
    float m_eff_moment = 0.0f;
    FxVec2f m_eff_force {0, 0};

    // accumulated impulses for the current step
    float m_eff_impulse_moment = 0.0f;
    FxVec2f m_eff_impulse {0, 0};

    // axis aligned bounding box in world coordinates
    FxArray<float> m_bounding_box {-1.0f, -1.0f, -1.0f, -1.0f}; 

    //update pose from velocity
    void __update_pose(const double& step_dt);
    
  public:
    // unique name for an entity
    const std::string name;

    // current state (public interface - single precision)
    FxVec3f pose {0, 0, 0};    // x, y, theta
    FxVec3f velocity {0, 0, 0};   // velocity along x, y axis and angular velocity along z axis

    // previous pose and velocity for tracking changes (public interface)
    FxVec3f prev_pose {0, 0, 0};  
    FxVec3f prev_velocity {0, 0, 0};

    // physics config
    float elasticity = 1.0f;
    float vel_damping = 0.0f;
    float gravity_scale = 1.0f;
    float static_friction = 0.0f;
    float dynamic_friction = 0.0f;
    
    // entity state
    bool enabled = true;  // If false, entity is skipped in physics updates, collisions, and rendering

    // contructor with name validation
    explicit FxEntity(const std::string& entityName);

    // getter for the name and ID
    const std::string& get_name() const {return name;}
    size_t get_entity_id() const {return m_entity_id;}
    void set_entity_id(size_t id) {m_entity_id = id;}

    // resets current state to inital state
    void reset();

    // methods to set mass and inertia
    void set_mass(const float& mass);
    void set_inertia(); //calculate based on shape
    void set_inertia(const float& inertia);
    // methods to get mass and inertia
    float mass() const { return _mass; }
    float inertia() const { return _inertia; }
    float inv_mass() const { return _inv_mass; }
    float inv_inertia() const { return _inv_inertia; }

    // methods to set initial pose and velocity
    void set_init_pose(const FxVec3f& o_pose);
    void set_init_velocity(const FxVec3f& o_velocity);
    // Enable or disable external forces and torques, including effects due to collisions
    void enable_external_forces(bool enable);

    // clear existing visual and collison shapes and assign new
    void set_visual_geometry(FxVisualShape visual){
        m_visual = std::make_shared<FxVisualShape>(std::move(visual));
        m_visual->set_world_pose(pose);
    }
    void set_collision_geometry(FxCollisionShape collision){
        m_collision = std::make_shared<FxCollisionShape>(std::move(collision));
        m_collision->set_world_pose(pose);
    }
    // getters for visual and collison shapes
    std::shared_ptr<FxVisualShape> visual_geometry() const {return m_visual;}
    std::shared_ptr<FxCollisionShape> collision_geometry() const {return m_collision;}
    // methods to delete visual and collision shapes
    void del_visual_geometry(){ m_visual.reset(); };
    void del_collision_geometry(){ m_collision.reset(); };

    // collision detection methods
    const FxArray<float> bounding_box() const;
    bool aabb_overlap_check(const FxEntity& other) const;
    bool aabb_overlap_check(const std::shared_ptr<FxEntity>& other) const;

    // apply external influences
    void apply_torque(float torque);
    void apply_force(const FxVec2f& force);
    void apply_force(const FxVec2f& force, const FxVec2f& contact_point);
    void apply_impulse(const FxVec2f& impulse);
    void apply_impulse(const FxVec2f& impulse, const FxVec2f& contact_point);

    // Get instantaneous velocity at a specific position
    FxVec2f velocity_at_world_point(const FxVec2f& position) const;
    FxVec2f velocity_at_local_point(const FxVec2f& local_position) const;
    // Vector implementations of above 
    FxArray<FxVec2f> velocity_at_world_point(const FxArray<FxVec2f>& position) const;
    FxArray<FxVec2f> velocity_at_local_point(const FxArray<FxVec2f>& local_position) const;
    
    // Convert local point to world coordinates and vice-versa
    FxVec2f to_world_frame(const FxVec2f& local_point) const;
    FxVec2f to_entity_frame(const FxVec2f& world_point) const;

    // resolve the forces and torques and calculate acceleration;
    FxVec3f calc_acceleration();
    FxVec3f calc_acceleration(const FxVec2f& gravity);
    
    void step(const FxVec2f& gravity, const double& step_dt);

    ~FxEntity(){}
};