#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <functional>
#include <execution>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <utility>

#include "Fx2D/Math.h"
#include "Fx2D/Entity.h"
#include "Fx2D/Solver.h"
#include "Fx2D/Registry.h"

// Scene class takes care of entities motion and collisions
class FxScene {
  private:
    // no of entities in the scene can not exceed 4096
    static constexpr size_t m_enitities_limit = 4096; 
    // max and min time step values that can be use in step method 
    static constexpr double m_max_time_step = 0.06;
    static constexpr double m_min_time_step = 1e-3;
    size_t m_substeps = 11;
    // dirty flag to track when constraints need cleaning
    bool m_constraints_dirty = false; 

  protected:
    FxEntityRegistry m_entities;              // stores pointers to all entities with collision management
    FxNamedRegistry<FxConstraint> m_constraints; // stores all constraints

  public:
    // scene size [x, y] units
    const FxVec2ui size; 
    // background color or texture path
    FxVec4ui8 fillColor {230, 230, 230, 255};
    std::string fillTexturePath = "";
    // gravity config [x, y]
    FxVec2f gravity {0.0f, -9.81f}; 

    // constructor, destructor 
    FxScene(FxVec2ui SceneSize) : m_entities(m_enitities_limit), size(SceneSize) {};
    ~FxScene() {};

    // calls reset of all entities
    void reset();
    // simulation step
    void step(double step_dt); 
    void set_substeps(const size_t& substeps) { m_substeps = substeps; }
    void set_gravity(const FxVec2f& o_gravity) { gravity = o_gravity; }
    // custom call back function called after every time step, user gets access to the scene.
    void set_step_callback(const std::function<void(FxScene&, double dt)>& callback){
        m_func_step_callback = callback;
    }
    // Method to set a fillColor
    void set_fillColor(const FxVec4ui8& color) { fillColor = color; fillTexturePath = ""; }
    void set_fillTexture(const std::string& filePath) { fillTexturePath = filePath;}
    // Returns true if added; false if an entity with the name already exists.
    bool add_entity(const std::shared_ptr<FxEntity>& entity);
    // Returns true if deletion succeeded, false if the entity wasn't found.
    bool delete_entity(const std::string& name);
    // Returns the entity pointer if found; otherwise returns nullptr.
    std::shared_ptr<FxEntity> get_entity(const std::string& name) const;

    // Returns true if added; false if a constraint with the name already exists
    bool add_constraint(const std::shared_ptr<FxConstraint>& constraint);
    // Returns true if deletion succeeded, false if the constraint wasn't found
    bool delete_constraint(const std::string& name);

    // Registry access methods
    size_t entity_count() const { return m_entities.size(); }
    size_t constraint_count() const { return m_constraints.size(); }
    bool entity_exists(const std::string& name) const { return m_entities.get_rawptr(name) != nullptr; }
    bool constraint_exists(const std::string& name) const { return m_constraints.get_rawptr(name) != nullptr; }

    // Collision pair management (delegated to entity registry)
    void enable_collision(const std::string& entity1_name, const std::string& entity2_name) {
        m_entities.enable_collision(entity1_name, entity2_name);
    }
    void disable_collision(const std::string& entity1_name, const std::string& entity2_name) {
        m_entities.disable_collision(entity1_name, entity2_name);
    }

    // for_each_entity applies the given function on each entity in a given execution mode
    template <typename ExecPolicy, typename Func>
    void for_each_entity(ExecPolicy&& policy, Func&& func) {
        m_entities.for_each(std::forward<ExecPolicy>(policy), std::forward<Func>(func));
    }

    // transform_entities collects return values vector in a given execution mode
    template <typename ExecPolicy, typename Func>
    void transform_entities(ExecPolicy&& policy, Func&& func,
        std::vector<std::invoke_result_t<Func, std::shared_ptr<FxEntity>>>& results){
        m_entities.transform(std::forward<ExecPolicy>(policy), std::forward<Func>(func), results);
    }
    
  private:
    // Removes constraints with dead entities
    void sweep_dead_constraints();

     //custom callback function invoked in the step method
     std::function<void(FxScene&, double dt)> m_func_step_callback;
};
