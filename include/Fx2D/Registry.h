#pragma once
#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <iostream>
#include <limits>
#include <execution>
#include <algorithm>
#include <functional>
#include <cstdint>

template<typename T>
class FxNamedRegistry {
  protected:
    std::vector<std::shared_ptr<T>> m_items_vec;         // packed storage
    std::unordered_map<std::string, size_t> m_name_map;  // name -> index
    size_t m_max_size = std::numeric_limits<size_t>::max();

  public:
    FxNamedRegistry() = default;
    explicit FxNamedRegistry(size_t max_size) : m_max_size(max_size) {}

    void reserve(size_t n) {
        if (n > m_max_size) m_max_size = n;
        m_items_vec.reserve(n);
        m_name_map.reserve(n);
    }

    size_t size()  const noexcept { return m_items_vec.size(); }
    bool   empty() const noexcept { return m_items_vec.empty(); }
    void   set_max_size(size_t n) { m_max_size = n; }

    // Add (1 refcount inc)
    bool add(const std::shared_ptr<T>& item) {
        if (!item) { std::cerr << "FxNamedRegistry: Cannot add null item.\n"; return false; }
        if (m_items_vec.size() >= m_max_size) { 
            std::cerr << "FxNamedRegistry: Items limit exceeded.\n"; return false; 
        }
        const std::string& name = item->get_name();
        if (m_name_map.find(name) != m_name_map.end()) {
            std::cerr << "FxNamedRegistry: Item '" << name << "' already exists.\n"; return false;
        }
        m_items_vec.push_back(item);                   // refcount++
        m_name_map.emplace(name, m_items_vec.size()-1);
        return true;
    }

    // Remove by name (swap-pop fixup)
    bool remove(const std::string& name) {
        auto it = m_name_map.find(name);
        if (it == m_name_map.end()) { 
            std::cerr << "FxNamedRegistry: Item '" << name << "' not found.\n"; return false; 
        }
        size_t idx = it->second;
        size_t last = m_items_vec.size() - 1;
        if (idx != last) {
            m_items_vec[idx] = std::move(m_items_vec[last]);
            const std::string& moved_name = m_items_vec[idx]->get_name();
            m_name_map[moved_name] = idx;
        }
        m_items_vec.pop_back();
        m_name_map.erase(it);
        return true;
    }

    // Get shared ownership (1 refcount inc)
    std::shared_ptr<T> get(const std::string& name) const {
        auto it = m_name_map.find(name);
        if (it == m_name_map.end()) { std::cerr << "FxNamedRegistry: Item '" << name << "' not found.\n"; return nullptr; }
        return m_items_vec[it->second];
    }

    // Get raw pointer (no refcount inc)
    T* get_rawptr(const std::string& name) const noexcept {
        auto it = m_name_map.find(name);
        return (it == m_name_map.end()) ? nullptr : m_items_vec[it->second].get();
    }

    const std::vector<std::shared_ptr<T>>& items() const noexcept { return m_items_vec; }
    std::vector<std::shared_ptr<T>>&       items()       noexcept { return m_items_vec; }

    void clear() { m_items_vec.clear(); m_name_map.clear(); }
    void shrink_to_fit() { m_items_vec.shrink_to_fit(); m_name_map.rehash(0); }

    // for_each applies the given function on each item in a given execution mode
    template <typename ExecPolicy, typename Func>
    void for_each(ExecPolicy&& policy, Func&& func) {
        // copy to a new vector of raw pointers
        std::vector<T*> raw_items_vec;
        raw_items_vec.reserve(m_items_vec.size());
        for (const auto& item : m_items_vec) {
            raw_items_vec.push_back(item.get());
        }
        // pass to std::for_each to do the required optimization
        std::for_each(std::forward<ExecPolicy>(policy),
                      raw_items_vec.begin(),
                      raw_items_vec.end(),
                      std::forward<Func>(func));
    }

    // transform collects return values vector in a given execution mode
    template <typename ExecPolicy, typename Func>
    void transform(ExecPolicy&& policy, Func&& func,
        std::vector<std::invoke_result_t<Func, std::shared_ptr<T>>>& results){
        // set results vector size
        results.resize(m_items_vec.size()); 
        // copy to a new vector of raw pointers
        std::vector<T*> raw_items_vec;
        raw_items_vec.reserve(m_items_vec.size());
        for (const auto& item : m_items_vec) {
            raw_items_vec.push_back(item.get());
        }
        // pass to std::transform to do the required optimization
        std::transform(std::forward<ExecPolicy>(policy),
                    raw_items_vec.begin(),
                    raw_items_vec.end(),
                    results.begin(),
                    std::forward<Func>(func));
    }
};

// Forward declaration for FxEntity
class FxEntity;

// Specialized registry for entities that handles collision pair exclusion
class FxEntityRegistry : public FxNamedRegistry<FxEntity> {
  private:
    std::unordered_set<uint32_t> m_no_collision_pairs;  // excluded collision pairs
    size_t m_next_entity_id = 0;  // entity ID counter

  public:
    FxEntityRegistry() = default;
    explicit FxEntityRegistry(size_t max_size) : FxNamedRegistry<FxEntity>(max_size) {}

    // Override add method to set entity ID
    bool add(const std::shared_ptr<FxEntity>& entity) {
        if (!entity) { std::cerr << "FxEntityRegistry: Cannot add null entity.\n"; return false; }
        if (m_items_vec.size() >= m_max_size) { 
            std::cerr << "FxEntityRegistry: Entities limit exceeded.\n"; return false; 
        }
        const std::string& name = entity->get_name();
        if (m_name_map.find(name) != m_name_map.end()) {
            std::cerr << "FxEntityRegistry: Entity '" << name << "' already exists.\n"; return false;
        }
        // Set entity ID before adding
        entity->set_entity_id(m_next_entity_id++);
        m_items_vec.push_back(entity);                   // refcount++
        m_name_map.emplace(name, m_items_vec.size()-1);
        return true;
    }

    // Enable collision between two entities by name
    void enable_collision(const std::string& entity1_name, const std::string& entity2_name) {
        auto e1 = get_rawptr(entity1_name);
        auto e2 = get_rawptr(entity2_name);
        if (e1 && e2 && e1 != e2) {
            uint32_t pair_id = pack_id_pair(e1->get_entity_id(), e2->get_entity_id());
            m_no_collision_pairs.erase(pair_id);
        }
    }

    // Disable collision between two entities by name
    void disable_collision(const std::string& entity1_name, const std::string& entity2_name) {
        auto e1 = get_rawptr(entity1_name);
        auto e2 = get_rawptr(entity2_name);
        if (e1 && e2 && e1 != e2) {
            uint32_t pair_id = pack_id_pair(e1->get_entity_id(), e2->get_entity_id());
            m_no_collision_pairs.insert(pair_id);
        }
    }

    // Get broad phase pairs for collision detection
    std::vector<std::pair<size_t, size_t>> get_broad_phase_pairs() const {
        std::vector<std::pair<size_t, size_t>> pairs;
        pairs.reserve(m_items_vec.size() * 2); // small guess
        for (size_t i = 0; i < m_items_vec.size(); ++i) {
            if (!m_items_vec[i]->enabled) continue;  // Skip disabled entities
            for (size_t j = i + 1; j < m_items_vec.size(); ++j) {
                if (!m_items_vec[j]->enabled) continue;  // Skip disabled entities
                // Only include pairs that are allowed to collide
                if (is_collision_pair(m_items_vec[i]->get_entity_id(), 
                                      m_items_vec[j]->get_entity_id())) {
                    pairs.emplace_back(i, j);
                }
            }
        }
        return pairs;
    }

  private:

    // Helper function to pack two entity IDs into a single uint32_t
    // Need to improve this
    static uint32_t pack_id_pair(size_t a, size_t b) {
        if (a > b) std::swap(a, b);
        return static_cast<uint32_t>((a << 12) | b);
    }

    // Check if collision is enabled between two entities
    bool is_collision_pair(size_t entity1_id, size_t entity2_id) const {
        uint32_t pair_id = pack_id_pair(entity1_id, entity2_id);
        return m_no_collision_pairs.find(pair_id) == m_no_collision_pairs.end();
    }

};
