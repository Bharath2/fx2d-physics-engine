#include "Fx2D/Scene.h"

namespace {
uint64_t pack_contact_key(uint32_t a, uint32_t b) {
    if (a > b) std::swap(a, b);
    return static_cast<uint64_t>(a) << 32 | static_cast<uint64_t>(b);
}

// Sweeps over the full contact list per substep in the velocity pass.
constexpr size_t kVelocityPasses = 8;
} // namespace

// calls reset of all entities
void FxScene::reset() {
    m_time_elapsed = 0.0; // Reset elapsed time
    m_contact_cache.clear();
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
        // constraints and joints need sweeping
        m_entities_dirty = true;
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
        if (constraint->get_entity1() && constraint->get_entity2()) {
            m_entities.disable_collision(constraint->get_entity1_name(),
                                         constraint->get_entity2_name());
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
        if (constraint->get_entity1() && constraint->get_entity2()) {
            m_entities.enable_collision(constraint->get_entity1_name(),
                                        constraint->get_entity2_name());
        }
    }
    return m_constraints.remove(name);
}

// Returns the constraint pointer if found; otherwise returns nullptr.
std::shared_ptr<FxConstraint> FxScene::get_constraint(const std::string& name) const {
    return m_constraints.get(name);
}

void FxScene::sweep_dead_constraints() {
    std::vector<std::string> dead_names;
    const auto& constraints_vec = m_constraints.items();
    for (const auto& c : constraints_vec) {
        bool dead = !c->get_entity1() || !c->get_entity2() ||
                    !m_entities.get_rawptr(c->get_entity1_name()) ||
                    !m_entities.get_rawptr(c->get_entity2_name());
        if (dead) dead_names.push_back(c->get_name());
    }
    for (const auto& name : dead_names) {
        delete_constraint(name);
    }
}

// Returns true if added; false if a joint with the name already exists.
bool FxScene::add_joint(const std::shared_ptr<FxJoint>& joint) {
    if (joint.get() == nullptr) {
        std::cerr << "FxScene: Cannot add a null joint." << std::endl;
        return false;
    }
    bool success = m_joints.add(joint);
    if (success) {
        const auto& constraints = joint->get_constraints();
        for (const auto& constraint : constraints) {
            add_constraint(constraint);
        }
    }
    return success;
}

// Returns true if deletion succeeded, false if the joint wasn't found.
bool FxScene::delete_joint(const std::string& name) {
    // Get joint before deletion to handle collision exclusion and constraints
    auto joint = m_joints.get(name);
    if (joint) {
        const auto& constraints = joint->get_constraints();
        for (const auto& constraint : constraints) {
            delete_constraint(constraint->get_name());
        }
    }
    return m_joints.remove(name);
}

// Returns the joint pointer if found; otherwise returns nullptr.
std::shared_ptr<FxJoint> FxScene::get_joint(const std::string& name) const {
    return m_joints.get(name);
}

void FxScene::sweep_dead_joints() {
    std::vector<std::string> dead_names;
    const auto& joints_vec = m_joints.items();
    for (const auto& j : joints_vec) {
        bool dead = !j->get_entity1() || !j->get_entity2() ||
                    !m_entities.get_rawptr(j->get_entity1_name()) ||
                    !m_entities.get_rawptr(j->get_entity2_name());
        if (dead) dead_names.push_back(j->get_name());
    }
    for (const auto& name : dead_names) {
        delete_joint(name);
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

    // Sweep dead constraints and joints if needed
    if (m_entities_dirty) {
        sweep_dead_joints();
        sweep_dead_constraints();
        m_contact_cache.clear();
        m_entities_dirty = false;
    }

    // Substeps
    std::vector<FxContact> contacts;
    std::unordered_set<uint64_t> active_keys;
    const auto& entities_vec = m_entities.items();
    for (size_t iter = 0; iter < m_substeps; ++iter) {
        // apply any required pid controls on the joints
        m_joints.for_each(std::execution::seq,
                          [&](auto joint) { joint->apply_controls(substep_dt); });

        m_entities.for_each(std::execution::par, [&](auto entity) {
            if (!entity->enabled || entity->is_sleeping())
                return; // Skip disabled/sleeping entities
            entity->step(gravity, substep_dt);
            // Boundary handling with elasticity bounce
            if ((entity->pose.x() >= static_cast<float>(size.x()) && entity->velocity.x() > 0.0f) ||
                (entity->pose.x() <= 0.0f && entity->velocity.x() < 0.0f)) {
                entity->pose.x() = entity->prev_pose.x();
                entity->velocity.x() *= -entity->elasticity;
            }
            if ((entity->pose.y() >= static_cast<float>(size.y()) && entity->velocity.y() > 0.0f) ||
                (entity->pose.y() <= 0.0f && entity->velocity.y() < 0.0f)) {
                entity->pose.y() = entity->prev_pose.y();
                entity->velocity.y() *= -entity->elasticity;
            }
        });

        // compute contacts - skip disabled entities
        contacts.clear();
        auto broad_phase_pairs = m_entities.get_broad_phase_pairs(static_cast<float>(substep_dt));
        for (const auto& pair : broad_phase_pairs) {
            FxContact c =
                FxSolver::collision_check(entities_vec[pair.first], entities_vec[pair.second]);
            if (!c.is_valid() &&
                (entities_vec[pair.first]->enable_ccd || entities_vec[pair.second]->enable_ccd))
                c = FxSolver::speculative_contact_check(entities_vec[pair.first],
                                                        entities_vec[pair.second],
                                                        static_cast<float>(substep_dt));
            if (c.is_valid()) {
                // Wake only when a moving, awake partner disturbs a sleeper.
                auto wake_if_disturbed = [](const std::shared_ptr<FxEntity>& sleeper,
                                            const std::shared_ptr<FxEntity>& other) {
                    if (!sleeper || !other || !sleeper->is_sleeping()) return;
                    if (other->is_sleeping()) return;
                    const bool moving =
                        other->velocity.head<2>().norm() > sleeper->sleep_threshold_linear ||
                        std::fabs(other->velocity.theta()) > sleeper->sleep_threshold_angular;
                    if (moving) sleeper->wake();
                };
                wake_if_disturbed(c.entity1, c.entity2);
                wake_if_disturbed(c.entity2, c.entity1);

                uint64_t key =
                    pack_contact_key(c.entity1->get_entity_id(), c.entity2->get_entity_id());
                active_keys.insert(key);
                auto cache_it = m_contact_cache.find(key);
                if (cache_it != m_contact_cache.end()) {
                    // If the contact normal changed significantly the tangent basis flipped;
                    // discard cached friction impulse to avoid spurious lateral forces.
                    bool normal_stable = cache_it->second.normal.dot(c.normal) > 0.99f;
                    for (size_t contact_idx = 0; contact_idx < 2; ++contact_idx) {
                        // Warm-start guess only; jn_accumulated stays 0 until warm_start.
                        c.jn_warm[contact_idx] = cache_it->second.jn[contact_idx];
                        c.jt_warm[contact_idx] =
                            normal_stable ? cache_it->second.jt[contact_idx] : 0.0f;
                    }
                }
                contacts.emplace_back(std::move(c));
            }
        }

        // Solve contact penetration (position-level)
        for (const auto& c : contacts) {
            FxSolver::resolve_penetration(c, substep_dt);
        }

        // Solve constraints (XPBD-style)
        m_constraints.for_each(std::execution::seq,
                               [&](auto constraint) { constraint->resolve(substep_dt); });

        // Update velocities from positions - skip disabled/sleeping entities
        m_entities.for_each(std::execution::par, [&](auto entity) {
            if (!entity->enabled || entity->is_sleeping())
                return; // Skip disabled/sleeping entities
            FxVec3f delta = (entity->pose - entity->prev_pose);
            delta.set_theta(FxAngleWrap(delta.theta()));
            entity->velocity = delta / substep_dt;
        });

        // Restitution target, then warm-start (first substep only), then velocity sweeps.
        for (auto& c : contacts)
            FxSolver::init_velocity_pass(c);

        if (iter == 0) {
            for (auto& c : contacts)
                FxSolver::warm_start(c);
        }

        // Multiple passes: one solve per contact leaves stack velocity residuals.
        for (size_t pass = 0; pass < kVelocityPasses; ++pass) {
            for (auto& c : contacts)
                FxSolver::resolve_velocities(c);
        }

        for (auto& c : contacts) {
            uint64_t key = pack_contact_key(c.entity1->get_entity_id(), c.entity2->get_entity_id());
            auto& cache_entry = m_contact_cache[key];
            cache_entry.normal = c.normal; // save basis so next frame can detect flips
            for (size_t contact_idx = 0; contact_idx < 2; ++contact_idx) {
                cache_entry.jn[contact_idx] = c.jn_accumulated[contact_idx];
                cache_entry.jt[contact_idx] = c.jt_accumulated[contact_idx];
            }
        }
    }

    for (auto it = m_contact_cache.begin(); it != m_contact_cache.end();) {
        if (active_keys.find(it->first) == active_keys.end()) {
            it = m_contact_cache.erase(it);
        } else {
            ++it;
        }
    }

    // Update total elapsed time
    m_time_elapsed += clamped_dt;

    // Advance sleep timers for all awake, enabled entities that aren't held by a constraint
    std::unordered_set<uint32_t> constrained_ids;
    for (const auto& c : m_constraints.items()) {
        if (c->get_entity1()) constrained_ids.insert(c->get_entity1()->get_entity_id());
        if (c->get_entity2()) constrained_ids.insert(c->get_entity2()->get_entity_id());
    }
    const float sleep_dt = static_cast<float>(clamped_dt);
    m_entities.for_each(std::execution::seq, [&](auto entity) {
        if (entity->enabled && !entity->is_sleeping() &&
            constrained_ids.find(entity->get_entity_id()) == constrained_ids.end()) {
            entity->tick_sleep(sleep_dt);
        }
    });

    // If a custom step callback function is set, call it
    if (m_func_step_callback) {
        m_func_step_callback(*this, clamped_dt);
    }
}
