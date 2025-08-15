#include "Entity.h"
#include "Solver.h"
#include "Scene.h"

// calls reset of all entities
void FxScene::reset() {
    for_each_entity(std::execution::par, [](auto entity) {
        entity->reset(); // Apply reset() to each entity
    });
}

// set the maximum limit for time step
void FxScene::set_max_time_step(const float& step_dt){
    if (step_dt < m_min_time_step){
        throw std::invalid_argument("FxScene: max time step (dt) must be greater than 2e-6");
    } else if (step_dt > 0.1f){
        throw std::invalid_argument("FxScene: max time step (dt) must be less than or equal to 0.1");
    }
    m_max_time_step = step_dt;
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

// simulation step
void FxScene::step(float step_dt) {
    // Throw an error if dt is negative
    if (step_dt < m_min_time_step) {
        throw std::invalid_argument("FxScene: dt (delta time) is too small");
    }
    float clamped_dt = std::clamp(step_dt, m_min_time_step, m_max_time_step);
    const float substep_dt = clamped_dt / static_cast<float>(m_substeps);

    // Broad phase: collect candidate pairs once per frame
    std::vector<std::pair<size_t, size_t>> broad_pairs;
    broad_pairs.reserve(m_entities_vec.size());
    for (size_t i = 0; i < m_entities_vec.size(); ++i) {
        for (size_t j = i + 1; j < m_entities_vec.size(); ++j) {
            if (FxSolver::aabb_overlap_check(m_entities_vec[i], m_entities_vec[j])) {
                broad_pairs.emplace_back(i, j);
            }
        }
    }

    // Substeps
    for (size_t iter = 0; iter < m_substeps; ++iter) {
        // Integrate (stores prev_pose internally)
        for_each_entity(std::execution::par, [&](auto entity) {
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

        // Narrow phase: compute contacts for candidate pairs
        std::vector<FxContact> contacts;
        contacts.reserve(broad_pairs.size());
        for (const auto& [i, j] : broad_pairs) {
            FxContact c = FxSolver::collision_check(m_entities_vec[i], m_entities_vec[j]);
            if (c.is_valid) contacts.emplace_back(std::move(c));
        }

        // Solve contact penetration (position-level)
        for (const auto& c : contacts) {
            FxSolver::resolve_penetration(c);
        }

        // // Solve constraints (position-level)
        // for (const auto& c : contraints) {
        //     FxSolver::resolve_constraints_xpbd(contraints, substep_dt);
        // }

        // Update velocities from positions
        for_each_entity(std::execution::par, [&](auto entity) {
            entity->velocity = (entity->pose - entity->prev_pose) / substep_dt;
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