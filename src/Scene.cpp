#include "Fx2D/Entity.h"
#include "Fx2D/Solver.h"
#include "Fx2D/Scene.h"

// calls reset of all entities
void FxScene::reset() {
    for_each_entity(std::execution::par, [](auto entity) {
        entity->reset(); // Apply reset() to each entity
    });
}

// Returns true if added; false if an entity with the name already exists.
bool FxScene::add_entity(std::shared_ptr<FxEntity> entity) {
    if (entity.get() == nullptr) {
        std::cerr << "FxScene: Cannot add a null entity." << std::endl;
        return false;
    }
    if(m_entities_vec.size() < m_enitities_limit){
        if (m_entities_map.find(entity->get_name()) != m_entities_map.end()) {
            std::cerr << "FxScene: Entity with name '" << entity->get_name() << "' already exists." << std::endl;
            return false;
        }
        // Assign unique entity ID
        entity->set_entity_id(m_next_entity_id++);
        m_entities_vec.push_back(entity);
        m_entities_map[entity->get_name()] = m_entities_vec.size() - 1;
        return true;
    } else {
        std::cerr << "FxScene: Entities limit exceeded." << std::endl;
        return false;
    }
}

// Returns true if deletion succeeded, false if the entity wasn't found.
bool FxScene::delete_entity(const std::string& name) {
    auto it = m_entities_map.find(name);
    if (it == m_entities_map.end()) {
        std::cerr << "FxScene: Entity with name '" << name << "' not found." << std::endl;
        return false;
    }
    // swap the last element with element to be deleted;
    size_t idx = it->second;
    size_t last_idx = m_entities_vec.size() - 1;
    if (idx != last_idx) {
        m_entities_vec[idx] = m_entities_vec[last_idx];
        m_entities_map[m_entities_vec[last_idx]->get_name()] = idx;
    }
    // Remove the last element from the vector.
    m_entities_vec.pop_back();
    // erase from map
    m_entities_map.erase(it);
    // constraints need sweeping
    m_constraints_dirty = true;
    return true;
}

// Returns the entity pointer if found; otherwise returns nullptr.
std::shared_ptr<FxEntity> FxScene::get_entity(const std::string& name) const {
    auto it = m_entities_map.find(name);
    if (it == m_entities_map.end()) {
        std::cerr << "FxScene: Entity with name '" << name << "' not found." << std::endl;
        return nullptr;
    }
    size_t idx = it->second;
    return m_entities_vec[idx];
}

// Returns true if added; false if a constraint with the name already exists.
bool FxScene::add_constraint(std::shared_ptr<FxConstraint> constraint) {
    if (constraint.get() == nullptr) {
        std::cerr << "FxScene: Cannot add a null constraint." << std::endl;
        return false;
    }
    const std::string& name = constraint->name;
    if (m_constraints_map.find(name) != m_constraints_map.end()) {
        std::cerr << "FxScene: Constraint with name '" << name << "' already exists." << std::endl;
        return false;
    }
    m_constraints_map[name] = m_constraints_vec.size();
    m_constraints_vec.push_back(constraint);
    
    // Add to collision exclusion if entities should not collide
    if (constraint->entity1 && constraint->entity2) {
        disable_collision(constraint->entity1, constraint->entity2);
    }
    
    // std::cout<<m_constraints_vec.size()<<std::endl;
    return true;
}

// Returns true if deletion succeeded, false if the constraint wasn't found.
bool FxScene::delete_constraint(const std::string& name) {
    auto it = m_constraints_map.find(name);
    if (it == m_constraints_map.end()) {
        std::cerr << "FxScene: Constraint with name '" << name << "' not found." << std::endl;
        return false;
    }
    // Remove from collision exclusion before deleting the constraint
    auto& constraint = m_constraints_vec[it->second];
    if (constraint->entity1 && constraint->entity2) {
         enable_collision(constraint->entity1, constraint->entity2);
    }
    
    // swap the last element with element to be deleted;
    size_t idx = it->second;
    size_t last_idx = m_constraints_vec.size() - 1;
    if (idx != last_idx) {
        m_constraints_vec[idx] = std::move(m_constraints_vec[last_idx]);
        m_constraints_map[m_constraints_vec[idx]->name] = idx;
    }
    // Remove the last element from the vector.
    m_constraints_vec.pop_back();
    // erase from map
    m_constraints_map.erase(it);
    return true;
}

// Returns the constraint pointer if found; otherwise returns nullptr.
FxConstraint* FxScene::get_constraint(const std::string& name) const {
    auto it = m_constraints_map.find(name);
    if (it == m_constraints_map.end()) {
        std::cerr << "FxScene: Constraint with name '" << name << "' not found." << std::endl;
        return nullptr;
    }
    size_t idx = it->second;
    return m_constraints_vec[idx].get();
}

void FxScene::sweep_dead_constraints() {
    std::vector<std::string> dead_names;
    for (const auto& c : m_constraints_vec) {
        bool dead = !c->entity1 || !c->entity2 ||
                    m_entities_map.find(c->entity1->get_name()) == m_entities_map.end() ||
                    m_entities_map.find(c->entity2->get_name()) == m_entities_map.end();
        if (dead) dead_names.push_back(c->name);
    }
    for (const auto& name : dead_names) {
        delete_constraint(name);
    }
}

// Enable collision between two entities by name
void FxScene::enable_collision(const std::string& entity1_name, const std::string& entity2_name) {
    auto e1 = get_entity(entity1_name);
    auto e2 = get_entity(entity2_name);
    if (e1 && e2) {
        enable_collision(e1, e2);
    }
}

// Enable collision between two entities by shared_ptr
void FxScene::enable_collision(std::shared_ptr<FxEntity> entity1, std::shared_ptr<FxEntity> entity2) {
    if (!entity1 || !entity2 || entity1.get() == entity2.get()) {
        std::cerr << "FxScene: Invalid entities for collision enabling." << std::endl;
        return;
    }
    uint32_t pair_id = pack_id_pair(entity1->get_entity_id(), entity2->get_entity_id());
    // Remove from no-collision set if it exists (enable collision)
    m_no_collision_pairs.erase(pair_id);
}

// Disable collision between two entities by name
void FxScene::disable_collision(const std::string& entity1_name, const std::string& entity2_name) {
    auto e1 = get_entity(entity1_name);
    auto e2 = get_entity(entity2_name);
    if (e1 && e2) {
        disable_collision(e1, e2);
    }
}

// Disable collision between two entities by shared_ptr
void FxScene::disable_collision(std::shared_ptr<FxEntity> entity1, std::shared_ptr<FxEntity> entity2) {
    if (!entity1 || !entity2 || entity1.get() == entity2.get()) {
        std::cerr << "FxScene: Invalid entities for collision disabling." << std::endl;
        return;
    }
    uint32_t pair_id = pack_id_pair(entity1->get_entity_id(), entity2->get_entity_id());
    // Add to no-collision set (disable collision)
    m_no_collision_pairs.insert(pair_id);
}


// simulation step
void FxScene::step(double step_dt) {
    // Throw an error if dt is negative
    if (step_dt < m_min_time_step) {
        throw std::invalid_argument("FxScene: dt (delta time) is too small");
    }
    double clamped_dt = std::clamp(step_dt, m_min_time_step, m_max_time_step);
    const double substep_dt = clamped_dt / static_cast<double>(m_substeps);

    // Sweep dead constraints if needed
    if (m_constraints_dirty) {
        sweep_dead_constraints();
        m_constraints_dirty = false;
    }

    // Substeps
    // std::cout<<substep_dt<<std::endl;
    std::vector<FxContact> contacts;
    for (size_t iter = 0; iter < m_substeps; ++iter) {
        // Integrate (stores prev_pose internally) - skip disabled entities
        for_each_entity(std::execution::par, [&](auto entity) {
            if (!entity->enabled) return;  // Skip disabled entities
            
            entity->step(gravity, substep_dt);
            // Simple boundary handling
            if ((entity->pose.x() >= size.x() && entity->velocity.x() > 0.0f) ||
                (entity->pose.x() <= 0.0f && entity->velocity.x() < 0.0f)) {
                entity->pose.x() = entity->prev_pose.x() - entity->velocity.x() * substep_dt;
            }
            if ((entity->pose.y() >= size.y() && entity->velocity.y() > 0.0f) ||
                (entity->pose.y() <= 0.0f && entity->velocity.y() < 0.0f)) {
                entity->pose.y() = entity->prev_pose.y() - entity->velocity.y() * substep_dt;
            }
        });

        // compute contacts - skip disabled entities
        contacts.clear();
        for (size_t i = 0; i < m_entities_vec.size(); ++i) {
            if (!m_entities_vec[i]->enabled) continue;  // Skip disabled entities
            
            for (size_t j = i + 1; j < m_entities_vec.size(); ++j) {
                if (!m_entities_vec[j]->enabled) continue;  // Skip disabled entities
                
                // Skip collision detection for excluded pairs
                if (is_collision_pair(m_entities_vec[i]->get_entity_id(), 
                                      m_entities_vec[j]->get_entity_id())) {
                    FxContact c = FxSolver::collision_check(m_entities_vec[i], m_entities_vec[j]);
                    if (c.is_valid()){
                        contacts.emplace_back(c);
                    };
                }
            }
        }

        // Solve contact penetration (position-level)
        for (const auto& c : contacts) {
            FxSolver::resolve_penetration(c, substep_dt);
        }

        // Solve constraints (position-level)
        for (const auto& c : m_constraints_vec) {
            c->resolve(substep_dt);
        }

        // Update velocities from positions with velocity clamping - skip disabled entities
        for_each_entity(std::execution::par, [&](auto entity) {
            if (!entity->enabled) return;  // Skip disabled entities
            
            FxVec3f delta = (entity->pose - entity->prev_pose);
            delta.set_theta(FxAngleWrap(delta.theta()));
            entity->velocity = delta / substep_dt;
        });

        // Solve velocity constraints (dynamic friction and restitution)
        for (const auto& c : contacts) {
            FxSolver::resolve_velocities(c);
        }
    }

    // If a custom step callback function is set, call it
    if (m_func_step_callback) {
        m_func_step_callback(*this, clamped_dt);
    }
}


// // set the maximum limit for time step
// void FxScene::set_max_time_step(const double& step_dt){
//     if (step_dt < m_min_time_step){
//         throw std::invalid_argument("FxScene: max time step (dt) must be greater than 1e-3");
//     } else if (step_dt > 0.1){
//         throw std::invalid_argument("FxScene: max time step (dt) must be less than or equal to 0.1");
//     }
//     m_max_time_step = step_dt;
// }