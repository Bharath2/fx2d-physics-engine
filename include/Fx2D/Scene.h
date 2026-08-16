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
#include "Fx2D/Query.h"
#include "Fx2D/Registry.h"
#include "Fx2D/Solver.h"

// A pair of entities that started or stopped touching during a step.
struct FxContactEvent {
    std::shared_ptr<FxEntity> entity1 = nullptr;
    std::shared_ptr<FxEntity> entity2 = nullptr;
    // True if either entity is a sensor, so no impulse was applied for this pair.
    bool is_trigger = false;
};

// Scene class takes care of entities motion and collisions
class FxScene {
  private:
    struct FxContactImpulseCache {
        float jn[2] = {0.0f, 0.0f};
        float jt[2] = {0.0f, 0.0f};
        FxVec2f normal{0.0f, 0.0f}; // contact normal at time of caching, used to detect basis flips
    };

    // no of entities in the scene can not exceed 4096
    static constexpr size_t m_enitities_limit = 4096;
    // max and min time step values that can be use in step method
    static constexpr double m_max_time_step = 0.06;
    static constexpr double m_min_time_step = 1e-3;
    size_t m_substeps = 11;
    // dirty flag to track when any entity is deleted
    bool m_entities_dirty = false;
    // total time elapsed since scene start
    double m_time_elapsed = 0.0;
    // Stores only scalar impulses so the cache never keeps entities alive accidentally.
    std::unordered_map<uint64_t, FxContactImpulseCache> m_contact_cache;

    // Contacts observed during the most recent step, one entry per touching pair, and the same
    // for the step before it. A pair may be found in several substeps; the entry keeps the last
    // one seen, so solved contacts carry their final accumulated impulses. The previous step's
    // buffer is retained so end-of-contact events can still name entities that stopped touching.
    std::vector<FxContact> m_step_contacts;
    std::unordered_map<uint64_t, size_t> m_step_contact_index;
    std::vector<FxContact> m_prev_contacts;
    std::unordered_map<uint64_t, size_t> m_prev_contact_index;
    std::vector<FxContactEvent> m_begin_contacts;
    std::vector<FxContactEvent> m_end_contacts;

    // Inserts a contact into the current step buffer, replacing any earlier one for the pair.
    void record_contact(const FxContact& contact, uint64_t key);
    // Diffs the current step buffer against the previous one to build begin/end events.
    void build_contact_events();

  protected:
    FxEntityRegistry m_entities; // stores pointers to all entities with collision management
    FxNamedRegistry<FxConstraint> m_constraints; // stores all constraints
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

    // Restores the scene to its initial state: every entity, constraint and joint that existed
    // when the scene was captured is present again and back at its starting pose, anything
    // added since is gone, anything deleted since is back, sleep and PID state are cleared,
    // and the contact caches and input are dropped. A reset is unconditional — no running
    // scene, callback or held body can decline it.
    //
    // The snapshot is taken by capture_initial_state(), automatically on the first step() if
    // it has not been taken explicitly. Build the scene, then let it step.
    void reset();
    // Marks the current composition as the state reset() restores. Call it explicitly after
    // building a scene if you intend to add or remove entities before the first step.
    void capture_initial_state();
    // simulation step
    void step(double step_dt);
    // get total time elapsed since scene start
    double time_elapsed() const { return m_time_elapsed; }
    void set_substeps(const size_t& substeps) { m_substeps = substeps; }
    void set_gravity(const FxVec2f& o_gravity) { gravity = o_gravity; }
    // custom call back function called after every time step, user gets access to the scene.
    void set_step_callback(const std::function<void(FxScene&, double dt)>& callback) {
        m_func_step_callback = callback;
    }
    // Called at the end of reset(), once entities are back at their initial state. Anything the
    // user layered on top of the scene — a held projectile, a score, a game phase — lives
    // outside the scene and would otherwise survive a reset that is meant to undo everything.
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

    // Keyboard and mouse state for this frame.
    //
    // A windowed scene has it filled in by FxRylbRenderer once per rendered frame, so a step
    // callback can drive gameplay without touching raylib. A headless scene has no window and
    // therefore no input: every query reads false and available() is false, until user code
    // injects state through the same object, which is how a headless scene scripts triggers.
    const FxInput& input() const { return m_input; }
    FxInput& input() { return m_input; }

    // Contacts from the most recent step, one entry per touching pair, in broad-phase order
    // (unspecified, but reproducible across identical runs). Valid until the next call to step()
    // or reset(). Pairs involving a sensor appear here with zero impulses; every other pair
    // carries the impulses that were actually applied.
    const std::vector<FxContact>& contacts() const { return m_step_contacts; }
    // Pairs that began touching during the most recent step (not touching in the step before).
    const std::vector<FxContactEvent>& begin_contact_events() const { return m_begin_contacts; }
    // Pairs that stopped touching during the most recent step (touching in the step before).
    const std::vector<FxContactEvent>& end_contact_events() const { return m_end_contacts; }

    // ---------------------------------------------------------------- spatial queries
    //
    // These ask what is where, without stepping. Overlap queries run the same narrow phase the
    // simulation uses, so a query and a contact agree about what is touching.
    //
    // Disabled entities and entities without collision geometry are never reported.
    //
    // Note these scan the entity list with a cheap bounding-box rejection rather than
    // descending the broad-phase tree. The tree is only synced during step(), so querying it
    // between steps — or before the first one — would answer from stale boxes. Correctness
    // first; if query volume ever justifies it, the fix is to sync the tree on demand.

    // Nearest entity struck by the ray, searching along `direction` from `origin` out to
    // `max_distance`. Returns false if nothing was hit. A ray starting inside a body reports
    // that body at distance 0.
    bool raycast(const FxVec2f& origin, const FxVec2f& direction, float max_distance,
                 FxRayHit& out_hit) const;
    // Every entity along the ray, nearest first. Useful for lidar-style observations.
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

    // The composition reset() restores. Held as shared_ptr so an entity deleted at runtime can
    // be put back rather than merely forgotten.
    std::vector<std::shared_ptr<FxEntity>> m_initial_entities;
    std::vector<std::shared_ptr<FxConstraint>> m_initial_constraints;
    std::vector<std::shared_ptr<FxJoint>> m_initial_joints;
    bool m_has_initial_snapshot = false;

    // Filled by the renderer each frame, or by user code in a headless scene. Never read by
    // the solver — input only reaches the simulation through user callbacks.
    FxInput m_input;
};
