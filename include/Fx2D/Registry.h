#pragma once

#include <algorithm>
#include <cstdint>
#include <execution>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Fx2D/AABBTree.h"

template<typename T>
class FxNamedRegistry {
  protected:
    std::vector<std::shared_ptr<T>> m_items_vec; // packed storage for cache-friendly iteration
    std::unordered_map<std::string, size_t> m_name_map; // name -> packed index in m_items_vec
    size_t m_size_limit = std::numeric_limits<size_t>::max(); // Hard ceiling imposed by the derived
                                                              // registry
    size_t m_max_size = std::numeric_limits<size_t>::max();

    bool _add(const std::shared_ptr<T>& item) {
        // The base registry only manages packed storage and name lookup.
        if (!item) {
            std::cerr << "FxNamedRegistry: Cannot add null item.\n";
            return false;
        }
        if (m_items_vec.size() >= m_max_size) {
            std::cerr << "FxNamedRegistry: Items limit exceeded.\n";
            return false;
        }

        const std::string& name = item->get_name();
        if (m_name_map.find(name) != m_name_map.end()) {
            std::cerr << "FxNamedRegistry: Item '" << name << "' already exists.\n";
            return false;
        }

        // New items are always appended so indices stay contiguous.
        m_items_vec.push_back(item);
        m_name_map.emplace(name, m_items_vec.size() - 1);
        return true;
    }

    bool _remove(const std::string& name) {
        // Removal is swap-pop so the packed vector stays dense.
        auto it = m_name_map.find(name);
        if (it == m_name_map.end()) {
            std::cerr << "FxNamedRegistry: Item '" << name << "' not found.\n";
            return false;
        }

        size_t idx = it->second;
        size_t last = m_items_vec.size() - 1;
        if (idx != last) {
            // Move the last live item into the hole and repair its name -> index mapping.
            m_items_vec[idx] = std::move(m_items_vec[last]);
            const std::string& moved_name = m_items_vec[idx]->get_name();
            m_name_map[moved_name] = idx;
        }
        // Pop the duplicated tail slot after the optional move above.
        m_items_vec.pop_back();
        m_name_map.erase(it);
        return true;
    }

  public:
    FxNamedRegistry() = default;
    explicit FxNamedRegistry(size_t max_size) : m_max_size(max_size) {}

    void reserve(size_t n) {
        // Reserve both containers together so packed indices remain valid after insertion.
        if (n > m_max_size) n = m_max_size;
        m_items_vec.reserve(n);
        m_name_map.reserve(n);
    }

    // Packed vector size is the canonical item count.
    size_t size() const noexcept { return m_items_vec.size(); }
    bool empty() const noexcept { return m_items_vec.empty(); }
    void set_max_size(size_t n) {
        if (n > m_size_limit) {
            std::cerr << "FxNamedRegistry: clamping max_size " << n << " to limit.\n";
            m_max_size = m_size_limit;
        } else {
            m_max_size = n;
        }
    }

    virtual bool add(const std::shared_ptr<T>& item) { return _add(item); }
    virtual bool remove(const std::string& name) { return _remove(name); }

    std::shared_ptr<T> get(const std::string& name) const {
        // Returning a shared_ptr extends the item's lifetime for the caller.
        auto it = m_name_map.find(name);
        if (it == m_name_map.end()) {
            std::cerr << "FxNamedRegistry: Item '" << name << "' not found.\n";
            return nullptr;
        }
        return m_items_vec[it->second];
    }

    T* get_rawptr(const std::string& name) const noexcept {
        // Raw access avoids refcount churn when the caller only needs a borrowed pointer.
        auto it = m_name_map.find(name);
        return (it == m_name_map.end()) ? nullptr : m_items_vec[it->second].get();
    }

    // base if free, else base_1, base_2, ... until a free name is found.
    std::string make_unique_name(const std::string& base) const {
        if (m_name_map.find(base) == m_name_map.end()) return base;
        for (int i = 1;; ++i) {
            std::string candidate = base + "_" + std::to_string(i);
            if (m_name_map.find(candidate) == m_name_map.end()) return candidate;
        }
    }

    const std::vector<std::shared_ptr<T>>& items() const noexcept { return m_items_vec; }
    std::vector<std::shared_ptr<T>>& items() noexcept { return m_items_vec; }

    void clear() {
        // Derived registries can override this when they have side caches to reset as well.
        m_items_vec.clear();
        m_name_map.clear();
    }

    void shrink_to_fit() {
        // Rehash(0) lets the unordered_map release extra buckets after compaction.
        m_items_vec.shrink_to_fit();
        m_name_map.rehash(0);
    }

  private:
    // Scratch for the raw-pointer snapshot below, kept as a member so its capacity survives.
    std::vector<T*> m_raw_items;
    bool m_raw_items_in_use = false;

    void fill_raw(std::vector<T*>& out) const {
        out.clear();
        out.reserve(m_items_vec.size());
        for (const auto& item : m_items_vec) {
            out.push_back(item.get());
        }
    }

    // Borrows the shared scratch buffer for one sweep, falling back to a local vector if a
    // sweep is already using it. A fresh vector per sweep was tens of allocations per step; the
    // fallback keeps that safe when a callback starts a second sweep of the same registry.
    class RawSweep {
      public:
        explicit RawSweep(FxNamedRegistry& registry) : m_registry(registry) {
            if (m_registry.m_raw_items_in_use) {
                m_registry.fill_raw(m_local);
                m_items = &m_local;
                return;
            }
            m_registry.m_raw_items_in_use = true;
            m_owns_shared = true;
            m_registry.fill_raw(m_registry.m_raw_items);
            m_items = &m_registry.m_raw_items;
        }
        ~RawSweep() {
            if (m_owns_shared) m_registry.m_raw_items_in_use = false;
        }
        RawSweep(const RawSweep&) = delete;
        RawSweep& operator=(const RawSweep&) = delete;

        std::vector<T*>& items() const { return *m_items; }

      private:
        FxNamedRegistry& m_registry;
        std::vector<T*> m_local;
        std::vector<T*>* m_items = nullptr;
        bool m_owns_shared = false;
    };

  public:
    template<typename ExecPolicy, typename Func>
    void for_each(ExecPolicy&& policy, Func&& func) {
        // Snapshot raw pointers first so parallel algorithms iterate a simple contiguous array,
        // and so the callee cannot invalidate the range it is walking.
        RawSweep sweep(*this);
        std::vector<T*>& raw = sweep.items();

        std::for_each(std::forward<ExecPolicy>(policy), raw.begin(), raw.end(),
                      std::forward<Func>(func));
    }

    template<typename ExecPolicy, typename Func>
    void transform(ExecPolicy&& policy, Func&& func,
                   std::vector<std::invoke_result_t<Func, std::shared_ptr<T>>>& results) {
        // Pre-size the output so std::transform can write by index in one pass.
        results.resize(m_items_vec.size());

        // Same snapshot as for_each, for the same reasons.
        RawSweep sweep(*this);
        std::vector<T*>& raw = sweep.items();

        std::transform(std::forward<ExecPolicy>(policy), raw.begin(), raw.end(), results.begin(),
                       std::forward<Func>(func));
    }
};

class FxEntity;

// Specialized registry for entities that handles collision pair exclusion.
class FxEntityRegistry : public FxNamedRegistry<FxEntity> {
  private:
    std::unordered_set<uint64_t> m_no_collision_pairs; // excluded collision pairs
    uint32_t m_next_entity_id = 0; // entity ID counter
    mutable FxAABBTree m_aabb_tree; // dynamic AABB tree
    // entity_id -> packed index. Ids are dense and issued in order from zero, so a vector is
    // both smaller and faster than the hash map this used to be, and the broad phase resolves
    // two of these per pair. -1 marks an id that has been removed.
    std::vector<int32_t> m_id_to_index;

    // Scratch for the tree pair query, reused across substeps.
    mutable std::vector<std::pair<int32_t, int32_t>> m_tree_pairs;

    void remove_entity_from_tree(FxEntity& entity) const {
        // Keep the tree free of stale leaves when an entity disappears or loses collision geometry.
        const int32_t node = entity.broad_phase_node();
        if (node < 0) return;
        m_aabb_tree.remove(node);
        entity.set_broad_phase_node(-1);
    }

    int32_t index_of_id(uint32_t entity_id) const {
        if (entity_id >= m_id_to_index.size()) return -1;
        return m_id_to_index[entity_id];
    }

    void erase_collision_pairs_for(uint32_t entity_id) {
        // Collision exclusions are keyed by entity id, so purge every pair that references the
        // removed entity.
        for (auto it = m_no_collision_pairs.begin(); it != m_no_collision_pairs.end();) {
            uint32_t a = static_cast<uint32_t>(*it >> 32);
            uint32_t b = static_cast<uint32_t>(*it & 0xffffffffULL);
            if (a == entity_id || b == entity_id) {
                it = m_no_collision_pairs.erase(it);
            } else {
                ++it;
            }
        }
    }

  public:
    FxEntityRegistry() = default;
    explicit FxEntityRegistry(size_t max_size) : FxNamedRegistry<FxEntity>() {
        m_size_limit = std::numeric_limits<uint32_t>::max();
        set_max_size(max_size);
        // A lower load factor keeps collision-pair lookups closer to O(1) under load.
        m_no_collision_pairs.max_load_factor(0.7f);
    }

    bool add(const std::shared_ptr<FxEntity>& entity) override {
        if (m_next_entity_id == std::numeric_limits<uint32_t>::max()) {
            std::cerr << "FxEntityRegistry: Entity ID limit exceeded.\n";
            return false;
        }

        // Entity ids stay stable even if packed indices move after swap-pop removal.
        entity->set_entity_id(m_next_entity_id);
        bool success = _add(entity);
        if (success) {
            // Broad-phase queries translate tree entity ids back to packed indices through this
            // map.
            const int32_t packed = static_cast<int32_t>(m_items_vec.size() - 1);
            if (m_next_entity_id >= m_id_to_index.size()) {
                m_id_to_index.resize(static_cast<size_t>(m_next_entity_id) + 1, -1);
            }
            m_id_to_index[m_next_entity_id] = packed;
            entity->set_packed_index(packed);
            m_next_entity_id++;
        }
        return success;
    }

    bool remove(const std::string& name) override {
        auto it = m_name_map.find(name);
        if (it == m_name_map.end()) {
            std::cerr << "FxEntityRegistry: Item '" << name << "' not found.\n";
            return false;
        }

        size_t idx = it->second;
        size_t last = m_items_vec.size() - 1;
        uint32_t removed_id = m_items_vec[idx]->get_entity_id();
        bool moved_last = (idx != last);
        uint32_t moved_id = moved_last ? m_items_vec[last]->get_entity_id() : 0;

        // Clean broad-phase and collision-filter state before the packed storage changes underneath
        // us.
        remove_entity_from_tree(*m_items_vec[idx]);
        erase_collision_pairs_for(removed_id);
        m_items_vec[idx]->set_packed_index(-1);
        m_id_to_index[removed_id] = -1;

        bool success = _remove(name);
        if (success && moved_last) {
            // Swap-pop removal can move the last entity into idx, so refresh its cached packed
            // index.
            m_id_to_index[moved_id] = static_cast<int32_t>(idx);
            m_items_vec[idx]->set_packed_index(static_cast<int32_t>(idx));
        }
        return success;
    }

    void clear() {
        // Entities outlive the registry that held them, so their cached indices have to be
        // invalidated here rather than left pointing into storage that no longer exists.
        for (const auto& e : m_items_vec) {
            if (e) {
                e->set_broad_phase_node(-1);
                e->set_packed_index(-1);
            }
        }
        FxNamedRegistry<FxEntity>::clear();
        m_no_collision_pairs.clear();
        m_id_to_index.clear();
        // Rebuild the tree pool from scratch so old node indices cannot leak across clear().
        m_aabb_tree = FxAABBTree{};
        m_next_entity_id = 0;
    }

    void enable_collision(const std::string& entity1_name, const std::string& entity2_name) {
        auto e1 = get_rawptr(entity1_name);
        auto e2 = get_rawptr(entity2_name);
        if (e1 && e2 && e1 != e2) {
            uint64_t pair_id = pack_id_pair(e1->get_entity_id(), e2->get_entity_id());
            m_no_collision_pairs.erase(pair_id);
        }
    }

    void disable_collision(const std::string& entity1_name, const std::string& entity2_name) {
        auto e1 = get_rawptr(entity1_name);
        auto e2 = get_rawptr(entity2_name);
        if (e1 && e2 && e1 != e2) {
            uint64_t pair_id = pack_id_pair(e1->get_entity_id(), e2->get_entity_id());
            m_no_collision_pairs.insert(pair_id);
        }
    }

    // Sync the tree from current entity state, then collect overlapping pairs. Convenience
    // overload; FxScene uses the out-parameter form so the pair buffer is allocated once.
    std::vector<std::pair<size_t, size_t>> get_broad_phase_pairs(float sweep_dt = 0.0f) const {
        std::vector<std::pair<size_t, size_t>> pairs;
        get_broad_phase_pairs(pairs, sweep_dt);
        return pairs;
    }

    // sweep_all_movers extends every mover's box along its velocity, not just CCD bodies, so
    // the list stays valid for the whole of sweep_dt. skip_sleeping_pairs must be false for a
    // list reused across a step, or a body woken partway through loses its pairs.
    void get_broad_phase_pairs(std::vector<std::pair<size_t, size_t>>& pairs, float sweep_dt = 0.0f,
                               bool sweep_all_movers = false,
                               bool skip_sleeping_pairs = true) const {
        sync_broad_phase(sweep_dt, sweep_all_movers);
        collect_broad_phase_pairs(pairs, skip_sleeping_pairs);
    }

    // Bring every proxy up to date and report whether any leaf was inserted, removed or
    // reinserted. That return value is what lets FxScene skip the pair walk: an unchanged tree
    // would rebuild the identical list. Syncing is a linear sweep; the walk is a descent.
    bool sync_broad_phase(float sweep_dt = 0.0f, bool sweep_all_movers = false) const {
        bool tree_changed = false;
        for (size_t i = 0; i < m_items_vec.size(); ++i) {
            const auto& e = m_items_vec[i];
            const int32_t node = e->broad_phase_node();
            const bool in_tree = (node >= 0);

            if (!e->enabled || !e->collision_geometry()) {
                // Disabled or geometry-less entities should never leave a broad-phase proxy behind.
                if (in_tree) {
                    remove_entity_from_tree(*e);
                    tree_changed = true;
                }
                continue;
            }

            const auto& bb = e->bounding_box();
            FxAABB tight{bb[0], bb[1], bb[2], bb[3]};
            // An axis-aligned edge or chain is zero-thickness on one axis, which is_valid()
            // rejects, so the entity would never enter the tree and never collide. Give the
            // proxy a hair of thickness; the shape's own AABB is left alone.
            constexpr float kMinProxyExtent = 1e-3f;
            if (tight.maxX - tight.minX < kMinProxyExtent) {
                tight.minX -= kMinProxyExtent;
                tight.maxX += kMinProxyExtent;
            }
            if (tight.maxY - tight.minY < kMinProxyExtent) {
                tight.minY -= kMinProxyExtent;
                tight.maxY += kMinProxyExtent;
            }
            if (!tight.is_valid()) continue;

            // Sweeping every mover was measured to flood the tree with false pairs when many
            // bodies fall together, so only CCD bodies extend their box by default -- they are
            // the ones whose speculative contacts need to see a partner before they reach it.
            FxAABB query_aabb = tight;
            if ((sweep_all_movers || e->enable_ccd) && sweep_dt > 0.0f) {
                float dx = e->velocity.x() * sweep_dt;
                float dy = e->velocity.y() * sweep_dt;
                if (dx != 0.0f || dy != 0.0f) {
                    FxAABB swept{tight.minX + dx, tight.minY + dy, tight.maxX + dx,
                                 tight.maxY + dy};
                    query_aabb = FxAABB::combine(tight, swept);
                }
            }

            if (!in_tree) {
                // New or re-enabled entities lazily create their tree leaf on demand.
                e->set_broad_phase_node(
                    m_aabb_tree.insert(static_cast<int32_t>(e->get_entity_id()), query_aabb));
                tree_changed = true;
            } else if (m_aabb_tree.update(node, query_aabb)) {
                tree_changed = true;
            }
        }
        return tree_changed;
    }

    // Walk the synced tree for overlapping proxies and translate them into packed indices,
    // applying the registry-level filters. Assumes sync_broad_phase has already run.
    void collect_broad_phase_pairs(std::vector<std::pair<size_t, size_t>>& pairs,
                                   bool skip_sleeping_pairs = true) const {
        // The scratch buffer is a member so its capacity carries across calls; query_pairs
        // clears it before filling.
        m_aabb_tree.query_pairs(m_tree_pairs);

        pairs.clear();
        pairs.reserve(m_tree_pairs.size());
        for (const auto& [eid_a, eid_b] : m_tree_pairs) {
            const int32_t ia = index_of_id(static_cast<uint32_t>(eid_a));
            const int32_t ib = index_of_id(static_cast<uint32_t>(eid_b));
            if (ia < 0 || ib < 0) continue;

            size_t i = static_cast<size_t>(ia);
            size_t j = static_cast<size_t>(ib);

            if (skip_sleeping_pairs && m_items_vec[i]->is_sleeping() &&
                m_items_vec[j]->is_sleeping() && !m_items_vec[i]->enable_ccd &&
                !m_items_vec[j]->enable_ccd)
                continue;
            if (!is_collision_pair(static_cast<uint32_t>(eid_a), static_cast<uint32_t>(eid_b)))
                continue;
            // One integer per body replaces O(N^2) pair exclusions for intra-group filtering.
            if (m_items_vec[i]->collision_group != 0 &&
                m_items_vec[i]->collision_group == m_items_vec[j]->collision_group &&
                m_items_vec[i]->collision_group < 0)
                continue;
            if (!m_items_vec[i]->collision_geometry() || !m_items_vec[j]->collision_geometry())
                continue;

            if (i > j) std::swap(i, j);
            pairs.emplace_back(i, j);
        }
    }

  private:
    static uint64_t pack_id_pair(uint32_t a, uint32_t b) {
        if (a > b) std::swap(a, b);
        return static_cast<uint64_t>(a) << 32 | static_cast<uint64_t>(b);
    }

    bool is_collision_pair(uint32_t entity1_id, uint32_t entity2_id) const {
        uint64_t pair_id = pack_id_pair(entity1_id, entity2_id);
        return m_no_collision_pairs.find(pair_id) == m_no_collision_pairs.end();
    }
};
