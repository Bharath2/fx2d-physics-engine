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
    std::vector<std::shared_ptr<T>> m_items_vec;         // packed storage for cache-friendly iteration
    std::unordered_map<std::string, size_t> m_name_map;  // name -> packed index in m_items_vec
    size_t m_size_limit = std::numeric_limits<size_t>::max();  // Hard ceiling imposed by the derived registry
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

    template <typename ExecPolicy, typename Func>
    void for_each(ExecPolicy&& policy, Func&& func) {
        // Snapshot raw pointers first so parallel algorithms iterate a simple contiguous array.
        std::vector<T*> raw_items_vec;
        raw_items_vec.reserve(m_items_vec.size());
        for (const auto& item : m_items_vec) {
            raw_items_vec.push_back(item.get());
        }

        std::for_each(std::forward<ExecPolicy>(policy),
                      raw_items_vec.begin(),
                      raw_items_vec.end(),
                      std::forward<Func>(func));
    }

    template <typename ExecPolicy, typename Func>
    void transform(ExecPolicy&& policy, Func&& func,
        std::vector<std::invoke_result_t<Func, std::shared_ptr<T>>>& results) {
        // Pre-size the output so std::transform can write by index in one pass.
        results.resize(m_items_vec.size());

        // Reuse the same raw-pointer snapshot pattern as for_each for lighter parallel dispatch.
        std::vector<T*> raw_items_vec;
        raw_items_vec.reserve(m_items_vec.size());
        for (const auto& item : m_items_vec) {
            raw_items_vec.push_back(item.get());
        }

        std::transform(std::forward<ExecPolicy>(policy),
                       raw_items_vec.begin(),
                       raw_items_vec.end(),
                       results.begin(),
                       std::forward<Func>(func));
    }
};

class FxEntity;

// Specialized registry for entities that handles collision pair exclusion.
class FxEntityRegistry : public FxNamedRegistry<FxEntity> {
  private:
    std::unordered_set<uint64_t> m_no_collision_pairs;  // excluded collision pairs
    uint32_t m_next_entity_id = 0;  // entity ID counter
    mutable FxAABBTree m_aabb_tree;  // dynamic AABB tree
    mutable std::unordered_map<uint32_t, int32_t> m_entity_node_map;  // entity_id -> tree node idx
    std::unordered_map<uint32_t, size_t> m_entity_idx_map;  // entity_id -> packed entity index

    void remove_entity_from_tree(uint32_t entity_id) const {
        // Keep the tree free of stale leaves when an entity disappears or loses collision geometry.
        auto it = m_entity_node_map.find(entity_id);
        if (it == m_entity_node_map.end()) return;
        m_aabb_tree.remove(it->second);
        m_entity_node_map.erase(it);
    }

    void erase_collision_pairs_for(uint32_t entity_id) {
        // Collision exclusions are keyed by entity id, so purge every pair that references the removed entity.
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
            // Broad-phase queries translate tree entity ids back to packed indices through this map.
            m_entity_idx_map[m_next_entity_id] = m_items_vec.size() - 1;
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

        // Clean broad-phase and collision-filter state before the packed storage changes underneath us.
        remove_entity_from_tree(removed_id);
        erase_collision_pairs_for(removed_id);
        m_entity_idx_map.erase(removed_id);

        bool success = _remove(name);
        if (success && moved_last) {
            // Swap-pop removal can move the last entity into idx, so refresh its cached packed index.
            m_entity_idx_map[moved_id] = idx;
        }
        return success;
    }

    void clear() {
        FxNamedRegistry<FxEntity>::clear();
        m_no_collision_pairs.clear();
        m_entity_node_map.clear();
        m_entity_idx_map.clear();
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

    // Get broad phase pairs for collision detection using the dynamic AABB tree.
    std::vector<std::pair<size_t, size_t>> get_broad_phase_pairs() const {
        // First sync the tree with the latest entity transforms and enable/disable state.
        for (size_t i = 0; i < m_items_vec.size(); ++i) {
            const auto& e = m_items_vec[i];
            uint32_t eid = e->get_entity_id();
            bool in_tree = (m_entity_node_map.count(eid) > 0);

            if (!e->enabled || !e->collision_geometry()) {
                // Disabled or geometry-less entities should never leave a broad-phase proxy behind.
                if (in_tree) {
                    remove_entity_from_tree(eid);
                }
                continue;
            }

            const auto& bb = e->bounding_box();
            if (bb[0] < 0.0f && bb[1] < 0.0f && bb[2] < 0.0f) continue;

            FxAABB tight { bb[0], bb[1], bb[2], bb[3] };
            if (!in_tree) {
                // New or re-enabled entities lazily create their tree leaf on demand.
                int32_t node = m_aabb_tree.insert(static_cast<int32_t>(eid), tight);
                m_entity_node_map[eid] = node;
            } else {
                m_aabb_tree.update(m_entity_node_map.at(eid), tight);
            }
        }

        // Then ask the tree for raw entity-id pairs.
        std::vector<std::pair<int32_t, int32_t>> tree_pairs;
        m_aabb_tree.query_pairs(tree_pairs);

        // Finally translate ids back to packed indices and apply registry-level filters.
        std::vector<std::pair<size_t, size_t>> pairs;
        pairs.reserve(tree_pairs.size());
        for (const auto& [eid_a, eid_b] : tree_pairs) {
            auto ia = m_entity_idx_map.find(static_cast<uint32_t>(eid_a));
            auto ib = m_entity_idx_map.find(static_cast<uint32_t>(eid_b));
            if (ia == m_entity_idx_map.end() || ib == m_entity_idx_map.end()) continue;

            size_t i = ia->second;
            size_t j = ib->second;

            if (m_items_vec[i]->is_sleeping() && m_items_vec[j]->is_sleeping()) continue;
            if (!is_collision_pair(static_cast<uint32_t>(eid_a), static_cast<uint32_t>(eid_b))) continue;
            if (!m_items_vec[i]->collision_geometry() || !m_items_vec[j]->collision_geometry()) continue;

            if (i > j) std::swap(i, j);
            pairs.emplace_back(i, j);
        }
        return pairs;
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
