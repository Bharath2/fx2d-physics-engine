#pragma once

#include <algorithm>
#include <execution>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "Fx2D/Entity.h"
#include "Fx2D/Input.h"
#include "Fx2D/Joints.h"
#include "Fx2D/Math.h"
#include "Fx2D/Registry.h"
#include "Fx2D/Solver.h"

// A named set of entities managed as one thing: deleted together, enabled together, and by
// default exempt from colliding with one another. Membership carries no connectivity or
// uniformity requirement; builders that populate groups may impose their own.
class FxEntityGroup {
    friend class FxScene;
    std::string m_name;
    int32_t m_collision_group = 0; // unique negative id when self-collision is off, else 0
    std::vector<std::shared_ptr<FxEntity>> m_members;

  public:
    explicit FxEntityGroup(const std::string& name) : m_name(name) {}
    const std::string& get_name() const { return m_name; }
    const std::vector<std::shared_ptr<FxEntity>>& members() const { return m_members; }
    size_t size() const { return m_members.size(); }
    void set_enabled(bool enabled) {
        for (auto& e : m_members) {
            if (!e) continue;
            e->enabled = enabled;
            if (enabled) e->wake();
        }
    }
};

// A pair of entities that started or stopped touching during a step.
struct FxContactEvent {
    // Borrowed, like FxContact's -- and kept alive by the same pins. Valid until the next
    // step(); do not hold on to an event past the frame that produced it.
    FxEntity* entity1 = nullptr;
    FxEntity* entity2 = nullptr;
    // True if either entity is a sensor, so no impulse was applied for this pair.
    bool is_trigger = false;
};

// Scene class takes care of entities motion and collisions
class FxScene {
  private:
    // Per-pair solver state surviving between substeps and steps. Held in a slot vector rather
    // than looked up by key each time: at 14 substeps that lookup was the bulk of the step's
    // hashing.
    struct FxContactPairSlot {
        // Warm-start impulses. Scalars only, so the cache never keeps entities alive.
        float jn[2] = {0.0f, 0.0f};
        float jt[2] = {0.0f, 0.0f};
        FxVec2f normal{0.0f, 0.0f}; // contact normal at time of caching, used to detect basis flips

        // Which step this pair was last seen in, and where its contact sits in m_step_contacts
        // for that step. Together they let the per-substep write-back skip the hash entirely
        // after the first substep that sees the pair.
        uint64_t step_stamp = 0;
        size_t step_slot = kNoStepSlot;
    };
    static constexpr size_t kNoStepSlot = static_cast<size_t>(-1);

    // no of entities in the scene can not exceed 4096
    static constexpr size_t m_enitities_limit = 4096;
    // max and min time step values that can be use in step method
    static constexpr double m_max_time_step = 0.06;
    static constexpr double m_min_time_step = 1e-3;
    // 14x4 measured fastest among configurations passing the full quality suite;
    // fewer passes fail tall stacks, more substeps at 8 passes just cost more.
    size_t m_substeps = 14;
    size_t m_velocity_passes = 4;
    // dirty flag to track when any entity is deleted
    bool m_entities_dirty = false;
    // total time elapsed since scene start
    double m_time_elapsed = 0.0;
    // Pair slots, plus the key -> slot map and the free list that recycles slots of pairs that
    // stopped touching. A contact carries its slot index, so only the first substep that sees a
    // pair in a given step pays for a lookup.
    std::vector<FxContactPairSlot> m_contact_slots;
    std::unordered_map<uint64_t, uint32_t> m_contact_slot_index;
    std::vector<uint32_t> m_free_contact_slots;
    // Slot already resolved for each entry of the current broad-phase pair list, or kNoSlot.
    // Reset whenever that list is rebuilt; the pair list is stable across the substeps of a
    // step, so after the first substep a contact finds its slot by index instead of by hash.
    std::vector<uint32_t> m_pair_slot;
    // Monotonic step counter, used only to stamp pair slots. Never reset by reset(), because a
    // stale stamp equal to the current step would revive a dead slot's step_slot.
    uint64_t m_step_counter = 0;

    // This step's touching pairs and the previous step's, keeping the last occurrence of each
    // so impulses are final. The previous buffer lets end events still name their entities.
    std::vector<FxContact> m_step_contacts;
    std::unordered_map<uint64_t, size_t> m_step_contact_index;
    std::vector<FxContact> m_prev_contacts;
    std::unordered_map<uint64_t, size_t> m_prev_contact_index;
    std::vector<FxContactEvent> m_begin_contacts;
    std::vector<FxContactEvent> m_end_contacts;

    // Ownership for the borrowed pointers in the contact buffers. Rebuilt once per step and
    // swapped alongside them, so an entity deleted between steps stays alive exactly as long as
    // a buffer still names it.
    std::vector<std::shared_ptr<FxEntity>> m_step_pins;
    std::vector<std::shared_ptr<FxEntity>> m_prev_pins;

    // Solver-local velocity columns, gathered and scattered once per substep around the
    // velocity passes. A member so the allocation happens once per scene, not once per step.
    FxSolverBodies m_solver_bodies;
    // Per-substep velocity-solver scratch, one entry per live contact.
    std::vector<FxContactSolverData> m_contact_solver_data;

    // Colour partition of the broad-phase pair list, the colour each contact inherited from its
    // pair, the contact order that groups them, and the transposed batch the sweeps run over.
    // All members for their capacity: rebuilt every substep, allocating only on the first.
    FxContactGraph m_contact_graph;
    std::vector<uint32_t> m_contact_colors;
    std::vector<uint32_t> m_colored_contacts;
    FxContactBatch m_contact_batch;

    // One contiguous run of the batch: a single colour and a single manifold size.
    struct FxBatchRun {
        uint32_t begin = 0;
        uint32_t end = 0;
        int slots = 2;
    };
    std::vector<FxBatchRun> m_batch_runs;

    // Inserts a contact into the current step buffer, replacing any earlier one for the pair.
    void record_contact(const FxContact& contact);
    // Takes ownership of every entity the current step buffer names, so the buffer can outlive
    // a delete_entity call.
    void pin_contact_entities();
    // The velocity half of one substep: gather, solve colour by colour, scatter.
    void solve_contact_velocities(std::vector<FxContact>& contacts,
                                  const std::vector<std::shared_ptr<FxEntity>>& entities_vec,
                                  size_t iter);
    // Resolves (or allocates) the pair slot for a contact, stamping the key and slot onto it.
    // `pair_index` addresses the broad-phase pair list, which is what lets the hash lookup
    // happen once per pair per step rather than once per pair per substep.
    void bind_contact_slot(FxContact& contact, size_t pair_index);
    // Drops slots for pairs that produced no contact this step.
    void evict_stale_contact_slots();
    // Diffs the current step buffer against the previous one to build begin/end events.
    void build_contact_events();

  protected:
    FxEntityRegistry m_entities; // stores pointers to all entities with collision management
    FxNamedRegistry<FxConstraint> m_constraints; // stores all constraints
    FxNamedRegistry<FxEntityGroup> m_groups; // named entity groups
    FxNamedRegistry<FxJoint> m_joints; // stores all joints

  public:
    // scene size [x, y] units
    const FxVec2ui size;
    // background color or texture path
    FxVec4ui8 fillColor{230, 230, 230, 255};
    std::string fillTexturePath = "";
    // gravity config [x, y]
    FxVec2f gravity{0.0f, -9.81f};

    // constructor, destructor
    FxScene(FxVec2ui SceneSize) : m_entities(m_enitities_limit), size(SceneSize) {}
    ~FxScene() = default;

    // Restores the captured composition and every entity to its initial state, clearing sleep,
    // PID, contact and input state. Unconditional: nothing running can decline it.
    void reset();
    // Marks the current composition as what reset() restores. Taken on the first step() if not
    // called explicitly; call it when entities are added or removed before that.
    void capture_initial_state();
    // simulation step
    void step(double step_dt);
    // get total time elapsed since scene start
    double time_elapsed() const { return m_time_elapsed; }
    void set_substeps(const size_t& substeps) { m_substeps = substeps; }
    // Velocity sweeps per substep; convergence trades against cost jointly with substeps.
    void set_velocity_passes(size_t passes) { m_velocity_passes = passes; }
    void set_gravity(const FxVec2f& o_gravity) { gravity = o_gravity; }
    // custom call back function called after every time step, user gets access to the scene.
    void set_step_callback(const std::function<void(FxScene&, double dt)>& callback) {
        m_func_step_callback = callback;
    }
    // Called at the end of reset(), once entities are restored, so state layered on top of the
    // scene can be restored too.
    void set_reset_callback(const std::function<void(FxScene&)>& callback) {
        m_func_reset_callback = callback;
    }
    // Method to set a fillColor
    void set_fillColor(const FxVec4ui8& color) {
        fillColor = color;
        fillTexturePath = "";
    }
    void set_fillTexture(const std::string& filePath) { fillTexturePath = filePath; }
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
    // Returns the constraint pointer if found; otherwise returns nullptr.
    std::shared_ptr<FxConstraint> get_constraint(const std::string& name) const;

    // Returns true if added; false if a joint with the name already exists
    bool add_joint(const std::shared_ptr<FxJoint>& joint);
    // Returns true if deletion succeeded, false if the joint wasn't found
    bool delete_joint(const std::string& name);
    // Returns the joint pointer if found; otherwise returns nullptr.
    std::shared_ptr<FxJoint> get_joint(const std::string& name) const;

    // Keyboard and mouse state, filled by a renderer each frame. A headless scene has none
    // until user code injects it, which is how it scripts triggers.
    const FxInput& input() const { return m_input; }
    FxInput& input() { return m_input; }

    // Contacts from the most recent step, one per touching pair, in a reproducible but
    // unspecified order. Sensor pairs appear with zero impulses. Valid until step() or reset().
    const std::vector<FxContact>& contacts() const { return m_step_contacts; }
    // Pairs that began touching during the most recent step (not touching in the step before).
    const std::vector<FxContactEvent>& begin_contact_events() const { return m_begin_contacts; }
    // Pairs that stopped touching during the most recent step (touching in the step before).
    const std::vector<FxContactEvent>& end_contact_events() const { return m_end_contacts; }

    // ---------------------------------------------------------------- spatial queries
    // Overlap runs the simulation's own narrow phase, so queries and contacts agree. Disabled
    // entities and those without collision geometry are never reported.

    // Nearest entity struck, searching `max_distance` along `direction`. A ray starting inside
    // a body reports it at distance 0.
    bool raycast(const FxVec2f& origin, const FxVec2f& direction, float max_distance,
                 FxRayHit& out_hit) const;
    // Every entity along the ray, nearest first.
    void raycast_all(const FxVec2f& origin, const FxVec2f& direction, float max_distance,
                     std::vector<FxRayHit>& out_hits) const;

    // Entities overlapping an arbitrary shape placed at `pose` (x, y, theta).
    void overlap_shape(const FxShape& shape, const FxVec3f& pose,
                       std::vector<std::shared_ptr<FxEntity>>& out) const;
    // Entities overlapping a circle, an axis-aligned box, or a single point.
    void overlap_circle(const FxVec2f& centre, float radius,
                        std::vector<std::shared_ptr<FxEntity>>& out) const;
    void overlap_box(const FxVec2f& centre, const FxVec2f& extents,
                     std::vector<std::shared_ptr<FxEntity>>& out) const;
    void overlap_point(const FxVec2f& point, std::vector<std::shared_ptr<FxEntity>>& out) const;
    // First entity covering the point, or nullptr — "what is under the cursor".
    std::shared_ptr<FxEntity> entity_at_point(const FxVec2f& point) const;

    // Registry access methods
    size_t entity_count() const { return m_entities.size(); }
    size_t constraint_count() const { return m_constraints.size(); }
    size_t joint_count() const { return m_joints.size(); }
    bool entity_exists(const std::string& name) const {
        return m_entities.get_rawptr(name) != nullptr;
    }
    bool constraint_exists(const std::string& name) const {
        return m_constraints.get_rawptr(name) != nullptr;
    }
    bool joint_exists(const std::string& name) const {
        return m_joints.get_rawptr(name) != nullptr;
    }

    // ---------------------------------------------------------------- entity groups
    // Returns the new group, or nullptr if the name is taken. With self_collide false the
    // group's members never collide with one another; outsiders are unaffected.
    std::shared_ptr<FxEntityGroup> create_group(const std::string& name, bool self_collide = false);
    // Adds the entity to the scene if needed, then to the group.
    bool add_to_group(const std::shared_ptr<FxEntityGroup>& group,
                      const std::shared_ptr<FxEntity>& entity);
    std::shared_ptr<FxEntityGroup> get_group(const std::string& name) const {
        return m_groups.get(name);
    }
    // Deletes every member entity, then the group itself.
    bool delete_group(const std::string& name);
    size_t group_count() const { return m_groups.size(); }
    // base if free, else base_1, base_2, ... — for builders generating entity names.
    std::string unique_entity_name(const std::string& base) const {
        return m_entities.make_unique_name(base);
    }

    // Collision pair management (delegated to entity registry)
    void enable_collision(const std::string& entity1_name, const std::string& entity2_name) {
        m_entities.enable_collision(entity1_name, entity2_name);
    }
    void disable_collision(const std::string& entity1_name, const std::string& entity2_name) {
        m_entities.disable_collision(entity1_name, entity2_name);
    }

    // for_each_entity applies the given function on each entity in a given execution mode
    template<typename ExecPolicy, typename Func>
    void for_each_entity(ExecPolicy&& policy, Func&& func) {
        m_entities.for_each(std::forward<ExecPolicy>(policy), std::forward<Func>(func));
    }

    // transform_entities collects return values vector in a given execution mode
    template<typename ExecPolicy, typename Func>
    void transform_entities(
        ExecPolicy&& policy, Func&& func,
        std::vector<std::invoke_result_t<Func, std::shared_ptr<FxEntity>>>& results) {
        m_entities.transform(std::forward<ExecPolicy>(policy), std::forward<Func>(func), results);
    }

  private:
    // Removes constraints with dead entities
    void sweep_dead_constraints();
    // Removes joints with dead entities
    void sweep_dead_joints();

    // custom callback function invoked in the step method
    std::function<void(FxScene&, double dt)> m_func_step_callback;
    std::function<void(FxScene&)> m_func_reset_callback;

    // Composition reset() restores; shared_ptr so a deleted entity can be put back.
    std::vector<std::shared_ptr<FxEntity>> m_initial_entities;
    std::vector<std::shared_ptr<FxConstraint>> m_initial_constraints;
    std::vector<std::shared_ptr<FxJoint>> m_initial_joints;
    // Membership is copied at capture, since the live group object mutates afterwards.
    std::vector<std::pair<std::shared_ptr<FxEntityGroup>, std::vector<std::shared_ptr<FxEntity>>>>
        m_initial_groups;
    int32_t m_next_collision_group = -1;
    bool m_has_initial_snapshot = false;

    // Never read by the solver; input reaches the simulation only through user callbacks.
    FxInput m_input;
};
