#include "Fx2D/Scene.h"

#include "Fx2D/Profile.h"

namespace {
uint64_t pack_contact_key(uint32_t a, uint32_t b) {
    if (a > b) std::swap(a, b);
    return static_cast<uint64_t>(a) << 32 | static_cast<uint64_t>(b);
}

} // namespace

// Resolves the pair slot for a freshly detected contact, allocating one if this pair has not
// been seen before, and stamps the key and slot index onto the contact. This is the single hash
// lookup the pair pays; every later touch this substep and the next reaches the slot by index.
void FxScene::bind_contact_slot(FxContact& contact, size_t pair_index) {
    contact.pair_key =
        pack_contact_key(contact.entity1->get_entity_id(), contact.entity2->get_entity_id());

    // Same pair, same slot, for as long as the pair list stands. Only the first substep that
    // sees a pair pays the hash.
    if (pair_index < m_pair_slot.size() && m_pair_slot[pair_index] != FxContact::kNoCacheSlot) {
        contact.cache_slot = m_pair_slot[pair_index];
        return;
    }

    auto [it, inserted] = m_contact_slot_index.emplace(contact.pair_key, 0u);
    if (inserted) {
        // Recycle a slot from a pair that stopped touching, or grow. Either way the slot starts
        // empty: a recycled slot carrying the previous impulses would warm-start the new pair
        // with a force that was never applied to it.
        uint32_t slot;
        if (!m_free_contact_slots.empty()) {
            slot = m_free_contact_slots.back();
            m_free_contact_slots.pop_back();
            m_contact_slots[slot] = FxContactPairSlot{};
        } else {
            slot = static_cast<uint32_t>(m_contact_slots.size());
            m_contact_slots.emplace_back();
        }
        it->second = slot;
    }
    contact.cache_slot = it->second;
    if (pair_index < m_pair_slot.size()) m_pair_slot[pair_index] = contact.cache_slot;
}

// Inserts a contact, replacing any earlier one for the pair, so impulses end up final.
// After the first substep that records a pair, its position in the step buffer is known from
// the pair slot, so the repeat writes never hash.
void FxScene::record_contact(const FxContact& contact) {
    FxContactPairSlot& slot = m_contact_slots[contact.cache_slot];
    if (slot.step_stamp == m_step_counter && slot.step_slot != kNoStepSlot) {
        m_step_contacts[slot.step_slot] = contact;
        return;
    }

    auto [it, inserted] = m_step_contact_index.emplace(contact.pair_key, m_step_contacts.size());
    if (inserted) {
        m_step_contacts.push_back(contact);
    } else {
        m_step_contacts[it->second] = contact;
    }
    slot.step_stamp = m_step_counter;
    slot.step_slot = it->second;
}

// A pair whose stamp is not this step produced no contact this step, so its warm-start impulses
// are stale and its slot goes back on the free list.
void FxScene::evict_stale_contact_slots() {
    for (auto it = m_contact_slot_index.begin(); it != m_contact_slot_index.end();) {
        if (m_contact_slots[it->second].step_stamp != m_step_counter) {
            m_free_contact_slots.push_back(it->second);
            it = m_contact_slot_index.erase(it);
        } else {
            ++it;
        }
    }
}

// Borrows become owns, once per step. The packed index each contact carries is what makes this
// a lookup-free sweep of the registry rather than a search.
void FxScene::pin_contact_entities() {
    const auto& entities = m_entities.items();
    m_step_pins.clear();
    m_step_pins.reserve(m_step_contacts.size() * 2);
    for (const FxContact& c : m_step_contacts) {
        if (c.body1 >= 0 && static_cast<size_t>(c.body1) < entities.size())
            m_step_pins.push_back(entities[static_cast<size_t>(c.body1)]);
        if (c.body2 >= 0 && static_cast<size_t>(c.body2) < entities.size())
            m_step_pins.push_back(entities[static_cast<size_t>(c.body2)]);
    }
}

// Diffs this step's touching pairs against the previous step's to produce begin/end events.
void FxScene::build_contact_events() {
    auto make_event = [](const FxContact& c) {
        FxContactEvent event;
        event.entity1 = c.entity1;
        event.entity2 = c.entity2;
        event.is_trigger =
            (c.entity1 && c.entity1->is_sensor) || (c.entity2 && c.entity2->is_sensor);
        return event;
    };

    for (const auto& [key, index] : m_step_contact_index) {
        if (m_prev_contact_index.find(key) == m_prev_contact_index.end()) {
            m_begin_contacts.push_back(make_event(m_step_contacts[index]));
        }
    }
    for (const auto& [key, index] : m_prev_contact_index) {
        if (m_step_contact_index.find(key) == m_step_contact_index.end()) {
            m_end_contacts.push_back(make_event(m_prev_contacts[index]));
        }
    }

    // unordered_map order is unspecified, so sort to keep runs reproducible.
    auto by_entity_id = [](const FxContactEvent& lhs, const FxContactEvent& rhs) {
        uint32_t lhs1 = lhs.entity1 ? lhs.entity1->get_entity_id() : 0;
        uint32_t rhs1 = rhs.entity1 ? rhs.entity1->get_entity_id() : 0;
        if (lhs1 != rhs1) return lhs1 < rhs1;
        uint32_t lhs2 = lhs.entity2 ? lhs.entity2->get_entity_id() : 0;
        uint32_t rhs2 = rhs.entity2 ? rhs.entity2->get_entity_id() : 0;
        return lhs2 < rhs2;
    };
    std::sort(m_begin_contacts.begin(), m_begin_contacts.end(), by_entity_id);
    std::sort(m_end_contacts.begin(), m_end_contacts.end(), by_entity_id);
}

// Records the composition that reset() restores.
void FxScene::capture_initial_state() {
    m_initial_entities = m_entities.items();
    m_initial_constraints = m_constraints.items();
    m_initial_joints = m_joints.items();
    m_initial_groups.clear();
    for (const auto& g : m_groups.items())
        m_initial_groups.emplace_back(g, g->m_members);
    m_has_initial_snapshot = true;
}

std::shared_ptr<FxEntityGroup> FxScene::create_group(const std::string& name, bool self_collide) {
    auto group = std::make_shared<FxEntityGroup>(name);
    if (!self_collide) group->m_collision_group = m_next_collision_group--;
    if (!m_groups.add(group)) return nullptr;
    return group;
}

bool FxScene::add_to_group(const std::shared_ptr<FxEntityGroup>& group,
                           const std::shared_ptr<FxEntity>& entity) {
    if (!group || !entity) return false;
    if (!m_entities.get_rawptr(entity->get_name()) && !m_entities.add(entity)) return false;
    entity->collision_group = group->m_collision_group;
    group->m_members.push_back(entity);
    return true;
}

bool FxScene::delete_group(const std::string& name) {
    auto group = m_groups.get(name);
    if (!group) return false;
    for (const auto& member : group->m_members) {
        if (member) delete_entity(member->get_name());
    }
    return m_groups.remove(name);
}

// calls reset of all entities
void FxScene::reset() {
    m_time_elapsed = 0.0; // Reset elapsed time
    m_contact_slots.clear();
    m_contact_slot_index.clear();
    m_free_contact_slots.clear();
    m_step_contacts.clear();
    m_step_contact_index.clear();
    m_prev_contacts.clear();
    m_prev_contact_index.clear();
    m_step_pins.clear();
    m_prev_pins.clear();
    m_begin_contacts.clear();
    m_end_contacts.clear();
    // Not reset with the rest: the counter only has to be different from every stamp still
    // sitting in a slot, and slots are cleared above. Letting it keep counting is what makes
    // that guarantee hold without walking anything.
    m_input.clear();
    m_entities_dirty = false;

    // Rebuild rather than patch, so the AABB tree, entity ids and collision-exclusion pairs
    // are rebuilt too and nothing survives that shouldn't.
    if (m_has_initial_snapshot) {
        m_entities.clear();
        m_constraints.clear();
        m_joints.clear();
        m_groups.clear();
        for (const auto& [group, members] : m_initial_groups) {
            group->m_members = members;
            m_groups.add(group);
        }
        for (const auto& entity : m_initial_entities)
            m_entities.add(entity);
        // Joints re-register their own constraints, so they go back first.
        for (const auto& joint : m_initial_joints)
            add_joint(joint);
        for (const auto& constraint : m_initial_constraints) {
            if (!m_constraints.get_rawptr(constraint->get_name())) add_constraint(constraint);
        }
    }

    for_each_entity(std::execution::seq, [](auto entity) {
        entity->reset(); // pose, velocity, accumulated forces, sleep state
    });
    // Motors carry integrator windup across a reset otherwise.
    m_joints.for_each(std::execution::seq, [](auto joint) { joint->reset(); });
    // Last, so user state is restored against a scene already back at its initial pose.
    if (m_func_reset_callback) {
        m_func_reset_callback(*this);
    }
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
        // A null entity2 is a world anchor, not a dangling reference; only a named entity
        // that has since been deleted makes a constraint dead.
        bool dead = !c->get_entity1() || !m_entities.get_rawptr(c->get_entity1_name()) ||
                    (c->get_entity2() && !m_entities.get_rawptr(c->get_entity2_name()));
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
    if (!m_joints.add(joint)) return false;

    // Half a joint is worse than no joint: its limits and motors would be silently weaker than
    // asked for, and the caller would have been told it worked. Roll back to what was there.
    const auto& constraints = joint->get_constraints();
    for (const auto& constraint : constraints) {
        if (add_constraint(constraint)) continue;
        std::cerr << "FxScene: joint '" << joint->get_name()
                  << "' has a constraint that could not be added; rolling the joint back."
                  << std::endl;
        for (const auto& added : constraints) {
            if (added == constraint) break;
            delete_constraint(added->get_name());
        }
        m_joints.remove(joint->get_name());
        return false;
    }
    return true;
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
        bool dead = !j->get_entity1() || !m_entities.get_rawptr(j->get_entity1_name()) ||
                    (j->get_entity2() && !m_entities.get_rawptr(j->get_entity2_name()));
        if (dead) dead_names.push_back(j->get_name());
    }
    for (const auto& name : dead_names) {
        delete_joint(name);
    }
}

// The entity loops below are sequential by measurement: std::execution::par was slower at
// every body count from 10 to 3000 while burning up to 32x the CPU, because the work is
// memory-bound and dispatched once per substep. Do not reintroduce it without an A/B run.

// The velocity half of one substep: gather the bodies into columns, resolve the contacts colour
// by colour, scatter back what moved. Its own method because it was the largest phase in step()
// and needs nothing but members and this substep's contacts.
void FxScene::solve_contact_velocities(std::vector<FxContact>& contacts,
                                       const std::vector<std::shared_ptr<FxEntity>>& entities_vec,
                                       size_t iter) {
    FX2D_PROF_SCOPE(VelocityPasses);
    // Gather after the narrow phase (waking a sleeper changes its inverse mass) and
    // after velocity derivation (which sets what is gathered). Sleeping and immovable
    // bodies gather a zero inverse mass, which is what makes them immovable below.
    m_solver_bodies.resize(entities_vec.size());
    for (size_t body = 0; body < entities_vec.size(); ++body) {
        const FxEntity& e = *entities_vec[body];
        const bool movable = !e.is_sleeping();
        m_solver_bodies.vx[body] = e.velocity.x();
        m_solver_bodies.vy[body] = e.velocity.y();
        m_solver_bodies.w[body] = e.velocity.theta();
        m_solver_bodies.inv_m[body] = movable ? e.inv_mass() : 0.0f;
        m_solver_bodies.inv_i[body] = movable ? e.inv_inertia() : 0.0f;
    }

    // Per-substep scratch, one record per contact, in the same order. A member for its
    // capacity: it is refilled every substep and should not allocate after the first.
    m_contact_solver_data.resize(contacts.size());

    // Restitution target. Order-independent: it reads velocity and writes only to the
    // contact's own scratch, so it stays in contact order.
    for (size_t i = 0; i < contacts.size(); ++i)
        FxSolver::init_velocity_pass(contacts[i], m_contact_solver_data[i], m_solver_bodies);

    if (iter == 0) {
        for (size_t i = 0; i < contacts.size(); ++i)
            FxSolver::warm_start(contacts[i], m_contact_solver_data[i], m_solver_bodies);
    }

    // Group the contacts by colour, then transpose each colour into columns. The
    // transpose happens once per substep and is swept velocity_passes times, so it is
    // paid once against four sweeps of contiguous, branch-free arithmetic.
    m_contact_graph.group_contacts(m_contact_colors, m_colored_contacts);
    m_contact_batch.clear();
    m_batch_runs.clear();

    // Two-point manifolds first within each colour, one-point after, each run tagged
    // with its slot count. Mixing them would push the one-point contacts through the
    // two-slot kernel and double their arithmetic for nothing.
    const size_t group_total = m_contact_graph.group_count();
    for (size_t g = 0; g < group_total; ++g) {
        const uint32_t g_begin = m_contact_graph.group_start(g);
        const uint32_t g_end = m_contact_graph.group_start(g + 1);
        // The overflow group is the one colour whose contacts may share a body, so its
        // entries go into runs of one: batching them would let two lanes gather the
        // same velocity and the later scatter drop the earlier one's impulse.
        const bool serial = m_contact_graph.is_overflow_group(g);
        for (int slots = 2; slots >= 1; --slots) {
            const uint32_t run_begin = static_cast<uint32_t>(m_contact_batch.size());
            for (uint32_t idx = g_begin; idx < g_end; ++idx) {
                const uint32_t k = m_colored_contacts[idx];
                const FxContact& c = contacts[k];
                if (!FxSolver::contact_is_solvable(c)) continue;
                if (static_cast<int>(c.count) != slots) continue;
                const uint32_t at = static_cast<uint32_t>(m_contact_batch.size());
                FxSolver::batch_append(m_contact_batch, c, m_contact_solver_data[k], k);
                if (serial) m_batch_runs.push_back({at, at + 1, slots});
            }
            const uint32_t run_end = static_cast<uint32_t>(m_contact_batch.size());
            if (!serial && run_end > run_begin) m_batch_runs.push_back({run_begin, run_end, slots});
        }
    }
    m_contact_batch.size_velocity_columns();

    // Multiple passes: one solve per contact leaves stack velocity residuals.
    for (size_t pass = 0; pass < m_velocity_passes; ++pass) {
        for (const FxBatchRun& run : m_batch_runs) {
            FxSolver::resolve_velocities_batched(m_contact_batch, run.begin, run.end,
                                                 m_solver_bodies, run.slots);
        }
    }
    FxSolver::batch_write_back(m_contact_batch, contacts);

    // Scatter back. Only bodies the sweeps could have moved are written: a sleeping or
    // immovable body gathered a zero inverse mass, so its column is unchanged, and
    // writing it back would be a store per body per substep for no effect.
    for (size_t body = 0; body < entities_vec.size(); ++body) {
        if (m_solver_bodies.inv_m[body] == 0.0f && m_solver_bodies.inv_i[body] == 0.0f) continue;
        FxEntity& e = *entities_vec[body];
        e.velocity.x() = m_solver_bodies.vx[body];
        e.velocity.y() = m_solver_bodies.vy[body];
        e.velocity.theta() = m_solver_bodies.w[body];
    }
}

void FxScene::step(double step_dt) {
    FX2D_PROF_SCOPE(StepTotal);
    // Throw an error if dt is negative
    if (step_dt < m_min_time_step) {
        throw std::invalid_argument("FxScene: dt (delta time) is too small");
    }
    double clamped_dt = std::clamp(step_dt, m_min_time_step, m_max_time_step);
    const double substep_dt = clamped_dt / static_cast<double>(m_substeps);

    // What the scene looks like when it first runs is what reset() restores.
    if (!m_has_initial_snapshot) capture_initial_state();

    // Sweep dead constraints and joints if needed
    if (m_entities_dirty) {
        sweep_dead_joints();
        sweep_dead_constraints();
        m_contact_slots.clear();
        m_contact_slot_index.clear();
        m_free_contact_slots.clear();
        m_entities_dirty = false;
    }

    // This step's contacts become the previous step's, which the begin/end diff needs, and
    // keeps entities that stopped touching alive long enough to be named.
    m_prev_contacts.swap(m_step_contacts);
    m_prev_contact_index.swap(m_step_contact_index);
    m_prev_pins.swap(m_step_pins);
    m_step_pins.clear();
    m_step_contacts.clear();
    m_step_contact_index.clear();
    m_begin_contacts.clear();
    m_end_contacts.clear();
    // Stamps every pair slot touched from here on. Bumped before the substeps so a slot left
    // over from the previous step cannot be mistaken for one recorded during this one.
    ++m_step_counter;

    // Substeps. Both buffers live outside the loop so their capacity is reached once and the
    // substeps never allocate; at 14 substeps that was 28 allocations per step.
    std::vector<FxContact> contacts;
    std::vector<std::pair<size_t, size_t>> broad_phase_pairs;
    const auto& entities_vec = m_entities.items();

    for (size_t iter = 0; iter < m_substeps; ++iter) {
        // apply any required pid controls on the joints
        m_joints.for_each(std::execution::seq,
                          [&](auto joint) { joint->apply_controls(substep_dt); });

        // Sequential by measurement, not by omission: see the note above step().
        {
            FX2D_PROF_SCOPE(Integration);
            m_entities.for_each(std::execution::seq, [&](auto entity) {
                if (!entity->enabled || entity->is_sleeping())
                    return; // Skip disabled/sleeping entities
                entity->step(gravity, substep_dt);
                // Boundary handling with elasticity bounce
                if ((entity->pose.x() >= static_cast<float>(size.x()) &&
                     entity->velocity.x() > 0.0f) ||
                    (entity->pose.x() <= 0.0f && entity->velocity.x() < 0.0f)) {
                    entity->pose.x() = entity->prev_pose.x();
                    entity->velocity.x() *= -entity->elasticity;
                }
                if ((entity->pose.y() >= static_cast<float>(size.y()) &&
                     entity->velocity.y() > 0.0f) ||
                    (entity->pose.y() <= 0.0f && entity->velocity.y() < 0.0f)) {
                    entity->pose.y() = entity->prev_pose.y();
                    entity->velocity.y() *= -entity->elasticity;
                }
            });
        }

        // The proxy sync is per substep; the pair walk behind it is not. If no leaf moved the
        // tree is unchanged and the walk would rebuild the identical list, so it is skipped --
        // exact, and about 90% of substeps in a settled scene. See docs/collision_resolution.md.
        {
            FX2D_PROF_SCOPE(BroadPhase);
            // Swept over the whole step on the first substep, so a body at constant velocity
            // needs no reinsertion for the rest of it. Later substeps use the substep dt, which
            // only has to catch bodies thrown off that path.
            const bool tree_changed =
                m_entities.sync_broad_phase(static_cast<float>(iter == 0 ? clamped_dt : substep_dt),
                                            /*sweep_all_movers=*/true);
            if (tree_changed || iter == 0) {
                m_entities.collect_broad_phase_pairs(broad_phase_pairs,
                                                     /*skip_sleeping_pairs=*/false);
                // The pair list moved, so every cached slot index refers to the wrong pair.
                m_pair_slot.assign(broad_phase_pairs.size(), FxContact::kNoCacheSlot);
                // Colour it while it stands. Pairs in one colour touch disjoint movable bodies,
                // which is what lets the velocity sweeps run a colour several lanes at a time.
                m_contact_graph.color_pairs(broad_phase_pairs, entities_vec);
            }
        }

        // compute contacts - skip disabled entities
        contacts.clear();
        m_contact_colors.clear();
        {
            FX2D_PROF_SCOPE(NarrowPhase);
            for (size_t pair_index = 0; pair_index < broad_phase_pairs.size(); ++pair_index) {
                const auto& pair = broad_phase_pairs[pair_index];
                const std::shared_ptr<FxEntity>& first = entities_vec[pair.first];
                const std::shared_ptr<FxEntity>& second = entities_vec[pair.second];
                const bool ccd = first->enable_ccd || second->enable_ccd;

                // Both tests live here, not in the query: the pair list outlives the substep
                // that built it, while both answers change as the substeps run -- a sleeper can
                // be woken by an earlier pair, and the bodies keep moving.
                if (!ccd && first->is_sleeping() && second->is_sleeping()) continue;
                // A CCD pair is never rejected on boxes, because a speculative contact is
                // generated precisely when the bodies are still apart.
                if (!ccd && !FxSolver::aabb_overlap_check(*first, *second)) continue;

                FxContact c = FxSolver::collision_check(first, second);
                if (!c.is_valid() && ccd)
                    c = FxSolver::speculative_contact_check(first, second,
                                                            static_cast<float>(substep_dt));
                if (c.is_valid()) {
                    c.body1 = c.entity1->packed_index();
                    c.body2 = c.entity2->packed_index();
                    bind_contact_slot(c, pair_index);

                    // A sensor is buffered for events but skipped by every solver stage, and must
                    // not wake a sleeper since it applies no force.
                    if (c.entity1->is_sensor || c.entity2->is_sensor) {
                        record_contact(c);
                        continue;
                    }

                    // Wake only when a moving, awake partner disturbs a sleeper.
                    auto wake_if_disturbed = [](FxEntity* sleeper, FxEntity* other) {
                        if (!sleeper || !other || !sleeper->is_sleeping()) return;
                        if (other->is_sleeping()) return;
                        const bool moving =
                            other->velocity.head<2>().norm() > sleeper->sleep_threshold_linear ||
                            std::fabs(other->velocity.theta()) > sleeper->sleep_threshold_angular;
                        if (moving) sleeper->wake();
                    };
                    wake_if_disturbed(c.entity1, c.entity2);
                    wake_if_disturbed(c.entity2, c.entity1);

                    const FxContactPairSlot& cached = m_contact_slots[c.cache_slot];
                    // A significantly changed normal means the tangent basis flipped, so the
                    // cached friction impulse is dropped. A slot allocated this substep holds a
                    // zero normal, so the test fails and both guesses stay zero.
                    const bool normal_stable = cached.normal.dot(c.normal) > 0.99f;
                    for (size_t contact_idx = 0; contact_idx < 2; ++contact_idx) {
                        // Warm-start guess only; jn_accumulated stays 0 until warm_start.
                        c.jn_warm[contact_idx] = cached.jn[contact_idx];
                        c.jt_warm[contact_idx] = normal_stable ? cached.jt[contact_idx] : 0.0f;
                    }
                    contacts.emplace_back(std::move(c));
                    m_contact_colors.push_back(m_contact_graph.color_of_pair(pair_index));
                }
            }
        }
        // Solve contact penetration (position-level)
        {
            FX2D_PROF_SCOPE(PositionSolve);
            for (const auto& c : contacts) {
                FxSolver::resolve_penetration(c, substep_dt);
            }
        }

        // Solve constraints (XPBD-style)
        {
            FX2D_PROF_SCOPE(Constraints);
            m_constraints.for_each(std::execution::seq,
                                   [&](auto constraint) { constraint->resolve(substep_dt); });
        }

        // Update velocities from positions - skip disabled/sleeping entities
        {
            FX2D_PROF_SCOPE(VelocityDerive);
            m_entities.for_each(std::execution::seq, [&](auto entity) {
                if (!entity->enabled || entity->is_sleeping())
                    return; // Skip disabled/sleeping entities
                FxVec3f delta = (entity->pose - entity->prev_pose);
                delta.set_theta(FxAngleWrap(delta.theta()));
                entity->velocity = delta / substep_dt;
            });
        }

        solve_contact_velocities(contacts, entities_vec, iter);

        FX2D_PROF_SCOPE(Bookkeeping);
        for (auto& c : contacts) {
            FxContactPairSlot& cache_entry = m_contact_slots[c.cache_slot];
            cache_entry.normal = c.normal; // save basis so next frame can detect flips
            for (size_t contact_idx = 0; contact_idx < 2; ++contact_idx) {
                cache_entry.jn[contact_idx] = c.jn_accumulated[contact_idx];
                cache_entry.jt[contact_idx] = c.jt_accumulated[contact_idx];
            }
            // Buffer the solved contact, so the exposed copy carries the applied impulses.
            record_contact(c);
        }
    }

    evict_stale_contact_slots();
    pin_contact_entities();

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

    // Built before the callback so user code can read contacts and events from inside it.
    build_contact_events();

    // If a custom step callback function is set, call it
    if (m_func_step_callback) {
        m_func_step_callback(*this, clamped_dt);
    }

    FX2D_PROF_STEP();
}
