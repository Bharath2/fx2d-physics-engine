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
bool FxScene::add_entity(const std::shared_ptr<FxEntity>& entity) {
    if (entity.get() == nullptr) {
        std::cerr << "FxScene: Cannot add a null entity." << std::endl;
        return false;
    }
    return m_entities.add(entity);
}

// Returns true if deletion succeeded, false if the entity wasn't found.
bool FxScene::delete_entity(const std::string& name) {
    bool success = m_entities.remove(name);
    if (success) {
        // constraints need sweeping
        m_constraints_dirty = true;
    }
    return success;
}

// Returns the entity pointer if found; otherwise returns nullptr.
std::shared_ptr<FxEntity> FxScene::get_entity(const std::string& name) const {
    return m_entities.get(name);
}

// Returns true if added; false if a constraint with the name already exists.
bool FxScene::add_constraint(const std::shared_ptr<FxConstraint>& constraint) {
    if (constraint.get() == nullptr) {
        std::cerr << "FxScene: Cannot add a null constraint." << std::endl;
        return false;
    }
    bool success = m_constraints.add(constraint);
    if (success) {
        // Add to collision exclusion if entities should not collide
        if (constraint->entity1 && constraint->entity2) {
            m_entities.disable_collision(constraint->entity1->get_name(), constraint->entity2->get_name());
        }
    }
    return success;
}

// Returns true if deletion succeeded, false if the constraint wasn't found.
bool FxScene::delete_constraint(const std::string& name) {
    // Get constraint before deletion to handle collision exclusion
    auto constraint = m_constraints.get(name);
    if (constraint) {
        // Remove from collision exclusion before deleting the constraint
        if (constraint->entity1 && constraint->entity2) {
             m_entities.enable_collision(constraint->entity1->get_name(), constraint->entity2->get_name());
        }
    }
    return m_constraints.remove(name);
}

// // Returns the constraint pointer if found; otherwise returns nullptr.
// FxConstraint* FxScene::get_constraint(const std::string& name) const {
//     return m_constraints.get_rawptr(name);
// }

void FxScene::sweep_dead_constraints() {
    std::vector<std::string> dead_names;
    const auto& constraints_vec = m_constraints.items();
    for (const auto& c : constraints_vec) {
        bool dead = !c->entity1 || !c->entity2 ||
                    !m_entities.get_rawptr(c->entity1->get_name()) ||
                    !m_entities.get_rawptr(c->entity2->get_name());
        if (dead) dead_names.push_back(c->name);
    }
    for (const auto& name : dead_names) {
        delete_constraint(name);
    }
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
    std::vector<FxContact> contacts;
    const auto& entities_vec = m_entities.items();
    for (size_t iter = 0; iter < m_substeps; ++iter) {
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
        auto broad_phase_pairs = m_entities.get_broad_phase_pairs();
        for (const auto& pair : broad_phase_pairs) {
            FxContact c = FxSolver::collision_check(entities_vec[pair.first], entities_vec[pair.second]);
            if (c.is_valid()){
                contacts.emplace_back(std::move(c));
            };
        }

        // Solve contact penetration (position-level)
        for (const auto& c : contacts) {
            FxSolver::resolve_penetration(c, substep_dt);
        }

        // Solve constraints (XPBD-style)
        const auto& constraints_vec = m_constraints.items();
        for (const auto& c : constraints_vec) {
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