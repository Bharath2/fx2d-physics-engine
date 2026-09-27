
#pragma once

#include <memory>
#include <stdexcept>
#include <string>

#include "Fx2D/Geometry.h"

inline bool is_valid_name(const std::string& s) {
    if (s.empty()) return false;
    return std::all_of(s.begin(), s.end(), [](unsigned char c) {
        return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
               c == '_';
    });
}

// container for visual shape
struct FxVisualShape : public FxShape {
  private:
    FxVec4ui8 m_fillColor{200, 200, 200, 255}; // Fill color (RGBA)
    FxVec4ui8 m_outlineColor{10, 10, 10, 255}; // Outline color (RGBA)
    float m_outlineThickness = 2.5f; // Outline thickness in pixels
    std::string m_fillTexturePath = ""; // File path to texture file (empty = no texture)
  public:
    // 1. "Inherit" all of FxShape's ctors
    using FxShape::FxShape;
    // 2. Conversion‐style ctor: build a VisualShape from any Shape
    FxVisualShape(const FxShape& base) : FxShape(base) {}
    // Method to set a fillColor, outlineColor and texture
    void set_fillColor(const FxVec4ui8& color) {
        m_fillColor = color;
        m_fillTexturePath = "";
    }
    void set_outlineColor(const FxVec4ui8& color) { m_outlineColor = color; }
    void set_outlineThickness(const float& thickness) { m_outlineThickness = thickness; }
    void set_fillTexture(const std::string& filePath) { m_fillTexturePath = filePath; }
    // getters for all the attributes
    const FxVec4ui8& fillColor() const { return m_fillColor; }
    const std::string& fillTexture() const { return m_fillTexturePath; }
    const FxVec4ui8& outlineColor() const { return m_outlineColor; }
    float outlineThickness() const { return m_outlineThickness; }
};

// container for collision shape
using FxCollisionShape = FxShape;

// Class for entity attributes and methods
class FxEntity {
  private:
    // unique identifier assigned by scene or registry
    uint32_t m_entity_id = 0;

    // unique name for an entity
    std::string m_name;

    // mass and inertia
    float _mass = 1.0f;
    float _inertia = 1.0f; // around center of mass
    float _inv_mass = 1.0f;
    float _inv_inertia = 1.0f;

    // store initial state for reset
    FxVec3f _init_pose{0, 0, 0};
    FxVec3f _init_velocity{0, 0, 0};
    FxVec3d m_pose_carry{0, 0, 0};
    // Constraint-correction residual, separate so it cannot borrow the integration leftover.
    FxVec3d m_correction_carry{0, 0, 0};

    // visual and collision shapes
    std::shared_ptr<FxCollisionShape> m_collision;
    std::shared_ptr<FxVisualShape> m_visual = std::make_shared<FxVisualShape>();

    // total resultant force and moment on the body
    float m_eff_moment = 0.0f;
    FxVec2f m_eff_force{0, 0};

    // accumulated impulses for the current step
    float m_eff_impulse_moment = 0.0f;
    FxVec2f m_eff_impulse{0, 0};

    // axis aligned bounding box in world coordinates
    FxArray<float> m_bounding_box{-1.0f, -1.0f, -1.0f, -1.0f};

    // sleep state tracking
    float m_sleep_timer = 0.0f;
    bool m_sleeping = false;

    // Broad-phase bookkeeping, owned by FxEntityRegistry. Held on the entity rather than in a
    // side map because the broad phase reads them once per entity per substep, and a hash
    // lookup there is pure overhead on a value the entity can just carry.
    int32_t m_broad_phase_node = -1; // leaf index in the registry's AABB tree, -1 = not in tree
    int32_t m_packed_index = -1; // position in the registry's packed storage, -1 = unregistered

    // update pose from velocity
    void __update_pose(const double& step_dt);

  public:
    // current state (public interface - single precision)
    FxVec3f pose{0, 0, 0}; // x, y, theta
    FxVec3f velocity{0, 0, 0}; // velocity along x, y axis and angular velocity along z axis

    // previous pose and velocity for tracking changes (public interface)
    FxVec3f prev_pose{0, 0, 0};
    FxVec3f prev_velocity{0, 0, 0};

    // physics config
    float elasticity = 0.1f; // restitution mixes by max, so 1.0 made every
                             // unconfigured body a perfect trampoline
    float vel_damping = 0.0f;
    float gravity_scale = 1.0f;
    float static_friction = 0.0f;
    float dynamic_friction = 0.0f;

    // entity state
    bool enabled = true; // If false, entity is skipped in physics updates, collisions, and
                         // rendering
    bool enable_ccd = false; // If true, speculative contacts are generated for this entity to
                             // prevent tunneling
    bool is_sensor = false; // Detects overlaps but exchanges no impulses; reported as events
    int32_t collision_group = 0; // Entities sharing a negative value never collide with each
                                 // other; 0 means no group filtering

    // sleep configuration
    float sleep_threshold_linear = 0.01f; // linear speed below which entity may sleep
    float sleep_threshold_angular = 0.05f; // angular speed below which entity may sleep
    float sleep_time_required = 0.5f; // seconds of low motion before sleeping
    bool is_sleeping() const { return m_sleeping; }
    void wake() {
        m_sleeping = false;
        m_sleep_timer = 0.0f;
    }
    void sleep() { m_sleeping = true; }
    void tick_sleep(float dt); // advance sleep timer; called by FxScene each step

    // contructor with name validation
    explicit FxEntity(const std::string& entityName);

    // getter for the name and ID
    const std::string& get_name() const { return m_name; }
    uint32_t get_entity_id() const { return m_entity_id; }
    void set_entity_id(uint32_t id) { m_entity_id = id; }

    // Registry-owned broad-phase bookkeeping. These are written by FxEntityRegistry as entities
    // are added, removed and inserted into the tree; nothing else should set them.
    int32_t broad_phase_node() const { return m_broad_phase_node; }
    void set_broad_phase_node(int32_t node) { m_broad_phase_node = node; }
    int32_t packed_index() const { return m_packed_index; }
    void set_packed_index(int32_t index) { m_packed_index = index; }

    // resets current state to inital state
    void reset();

    // methods to set mass and inertia
    void set_mass(const float& mass);
    void set_inertia(); // calculate based on shape
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
    void set_visual_geometry(FxVisualShape visual) {
        m_visual = std::make_shared<FxVisualShape>(std::move(visual));
        m_visual->set_world_pose(pose);
    }
    void set_collision_geometry(FxCollisionShape collision) {
        m_collision = std::make_shared<FxCollisionShape>(std::move(collision));
        // Filled here, not left to the first integration: an immovable body may never
        // integrate at all, and a box still at its {-1,-1,-1,-1} default is rejected by the
        // broad phase and silently collides with nothing.
        m_collision->set_world_pose(pose, m_bounding_box);
    }
    // getters for visual and collison shapes
    // By reference: returning by value cost an atomic pair per call, and the broad and narrow
    // phases call these several times per pair per substep. Valid until the shape is replaced.
    const std::shared_ptr<FxVisualShape>& visual_geometry() const { return m_visual; }
    const std::shared_ptr<FxCollisionShape>& collision_geometry() const { return m_collision; }
    // methods to delete visual and collision shapes
    void del_visual_geometry() { m_visual.reset(); };
    void del_collision_geometry() { m_collision.reset(); };

    // collision detection methods
    // By reference: returning the FxArray by value made every narrow-phase pair allocate.
    const FxArray<float>& bounding_box() const;
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
    FxVec2fArray velocity_at_world_point(const FxVec2fArray& position) const;
    FxVec2fArray velocity_at_local_point(const FxVec2fArray& local_position) const;

    // Convert local point to world coordinates and vice-versa
    FxVec2f to_world_frame(const FxVec2f& local_point) const;
    FxVec2f to_entity_frame(const FxVec2f& world_point) const;

    // resolve the forces and torques and calculate acceleration;
    FxVec3f calc_acceleration();
    FxVec3f calc_acceleration(const FxVec2f& gravity);

    // Mixed-precision positional correction, returning the delta that landed so prev_pose can
    // be shifted to match. Sub-ulp corrections are banked in double rather than rounded away.
    FxVec3f apply_pose_correction(const FxVec3f& delta);

    void step(const FxVec2f& gravity, const double& step_dt);

    ~FxEntity() {}
};