#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <string>
#include <functional>
#include <execution>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <utility>

#include "MathTypes.h"
#include "Entity.h"
#include "Solver.h"

// Scene class takes care of entities motion and collisions
class FxScene {
private:
    // no of entities in the scene can not exceed 4096
    static constexpr size_t m_enitities_limit = 4096; 
    // max and min time step values that can be use in step method 
    double m_max_time_step = 0.1;
    static constexpr double m_min_time_step = 1e-3;
    size_t m_substeps = 11;
    //custom callback function invoked in the step method
    std::function<void(FxScene&, double dt)> m_func_step_callback; 
    
protected:
    std::vector<std::shared_ptr<FxEntity>> m_entities_vec; // stores pointers to all entities
    std::unordered_map<std::string, size_t> m_entities_map; // maps entity's name to index in the entities vector

public:
    // scene size [x, y] units
    const FxVec2ui size; 
    // background color or texture path
    FxVec4ui8 fillColor {230, 230, 230, 255};
    std::string fillTexturePath = "";
    // gravity config [x, y]
    FxVec2f gravity {0.0f, -9.81f}; 

    // constructor, destructor 
    FxScene(FxVec2ui SceneSize) : size(SceneSize) {};
    ~FxScene() {};

    // calls reset of all entities
    void reset();
    // simulation step
    void step(double step_dt); 
    void set_max_time_step(const double& step_dt);
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
    bool add_entity(std::shared_ptr<FxEntity> entity);
    // Returns true if deletion succeeded, false if the entity wasn't found.
    bool delete_entity(const std::string& name);
    // Returns the entity pointer if found; otherwise returns nullptr.
    std::shared_ptr<FxEntity> get_entity(const std::string& name) const;

    // for_each_entity applies the given function on each entity in a given execution mode
    template <typename ExecPolicy, typename Func>
    void for_each_entity(ExecPolicy&& policy, Func&& func) {
        // copy to a new vector of raw pointers
        std::vector<FxEntity*> raw_entities_vec;
        raw_entities_vec.reserve(m_entities_vec.size());
        for (const auto& entity : m_entities_vec) {
            raw_entities_vec.push_back(entity.get());
        }
        // pass to std::for_each to do the required optimization
        std::for_each(std::forward<ExecPolicy>(policy),
                      raw_entities_vec.begin(),
                      raw_entities_vec.end(),
                      std::forward<Func>(func));
    }

    // transform_entities collects return values vector in a given execution mode
    template <typename ExecPolicy, typename Func>
    void transform_entities(ExecPolicy&& policy, Func&& func,
        std::vector<std::invoke_result_t<Func, std::shared_ptr<FxEntity>>>& results){
        // set results vector size
        results.resize(m_entities_vec.size()); 
        // copy to a new vector of raw pointers
        std::vector<FxEntity*> raw_entities_vec;
        raw_entities_vec.reserve(m_entities_vec.size());
        for (const auto& entity : m_entities_vec) {
            raw_entities_vec.push_back(entity.get());
        }
        // pass to std::tranform to do the required optimization
        std::transform(std::forward<ExecPolicy>(policy),
                    raw_entities_vec.begin(),
                    raw_entities_vec.end(),
                    results.begin(),
                    std::forward<Func>(func));
    }
};

