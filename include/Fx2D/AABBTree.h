#pragma once
#include "Fx2D/Math.h"
#include <vector>
#include <utility>
#include <limits>
#include <cstdint>

// ============================================================
// Dynamic AABB tree (index-based pool, SAH-guided insertion)
// ============================================================

// 20% half-extent margin used to fatten leaf AABBs so that
// small position changes don't trigger constant reinsertion.
static constexpr float AABB_TREE_MARGIN = 0.20f;

struct FxAABBTreeNode {
    FxAABB  fat_aabb;          // fattened AABB (stored in tree)
    FxAABB  tight_aabb;        // unfattened AABB (used for reinsertion check)
    int32_t parent     = -1;
    int32_t left       = -1;
    int32_t right      = -1;
    int32_t entity_id  = -1;   // >= 0 for leaf nodes only
    int32_t next_free  = -1;   // free-list chain (only valid when node is free)

    bool is_leaf() const { return entity_id >= 0; }
};

class FxAABBTree {
  public:
    FxAABBTree() = default;

    // --- Insert a leaf for the given entity, returns its node index.
    int32_t insert(int32_t entity_id, const FxAABB& tight) {
        int32_t leaf = alloc_node();
        m_nodes[leaf].tight_aabb = tight;
        m_nodes[leaf].fat_aabb   = fatten(tight);
        m_nodes[leaf].entity_id  = entity_id;
        insert_leaf(leaf);
        return leaf;
    }

    // --- Remove the leaf at node_idx and free the node.
    void remove(int32_t node_idx) {
        remove_leaf(node_idx);
        free_node(node_idx);
    }

    // --- Update a leaf's AABB.  Returns true if the fat box was rebuilt
    //     (i.e. entity moved far enough to escape its current fat box).
    bool update(int32_t node_idx, const FxAABB& new_tight) {
        FxAABBTreeNode& n = m_nodes[node_idx];
        if (n.fat_aabb.contains(new_tight)) {
            n.tight_aabb = new_tight;
            return false;            // still inside fat box, no reinsertion
        }
        remove_leaf(node_idx);
        n.tight_aabb = new_tight;
        n.fat_aabb   = fatten(new_tight);
        insert_leaf(node_idx);
        return true;
    }

    // --- Collect every overlapping leaf-pair (entity_id_a < entity_id_b).
    void query_pairs(std::vector<std::pair<int32_t, int32_t>>& out) const {
        out.clear();
        if (m_root < 0) return;
        collect_pairs(m_root, m_root, out, /*same_node=*/true);
    }

    // --- Accessors
    const FxAABBTreeNode& node(int32_t idx) const { return m_nodes[idx]; }
    FxAABBTreeNode&       node(int32_t idx)       { return m_nodes[idx]; }
    int32_t root()  const { return m_root; }
    bool    empty() const { return m_root < 0; }

  private:
    std::vector<FxAABBTreeNode> m_nodes;
    int32_t m_root      = -1;
    int32_t m_free_head = -1;

    // -------- node allocation pool --------
    int32_t alloc_node() {
        if (m_free_head >= 0) {
            int32_t idx = m_free_head;
            m_free_head = m_nodes[idx].next_free;
            m_nodes[idx] = FxAABBTreeNode{};
            return idx;
        }
        m_nodes.emplace_back();
        return static_cast<int32_t>(m_nodes.size() - 1);
    }

    void free_node(int32_t idx) {
        m_nodes[idx]           = FxAABBTreeNode{};
        m_nodes[idx].next_free = m_free_head;
        m_free_head            = idx;
    }

    // -------- AABB fattening --------
    static FxAABB fatten(const FxAABB& b) {
        float w  = b.maxX - b.minX;
        float h  = b.maxY - b.minY;
        // margin = 20% of each half-extent, with a minimum of 0.01 world units
        float mx = std::max(w * AABB_TREE_MARGIN, 0.01f);
        float my = std::max(h * AABB_TREE_MARGIN, 0.01f);
        return { b.minX - mx, b.minY - my, b.maxX + mx, b.maxY + my };
    }

    // -------- SAH-guided insertion --------
    void insert_leaf(int32_t leaf) {
        if (m_root < 0) {
            m_root = leaf;
            m_nodes[leaf].parent = -1;
            return;
        }

        int32_t best   = find_best_sibling(leaf);
        int32_t old_gp = m_nodes[best].parent;

        int32_t np          = alloc_node();       // new internal node
        m_nodes[np].fat_aabb = FxAABB::combine(m_nodes[leaf].fat_aabb, m_nodes[best].fat_aabb);
        m_nodes[np].entity_id = -1;
        m_nodes[np].parent    = old_gp;

        if (old_gp >= 0) {
            if (m_nodes[old_gp].left == best) m_nodes[old_gp].left  = np;
            else                              m_nodes[old_gp].right = np;
        } else {
            m_root = np;
        }

        m_nodes[np].left   = best;
        m_nodes[np].right  = leaf;
        m_nodes[best].parent = np;
        m_nodes[leaf].parent = np;

        refit_from(np);
    }

    // Best-sibling search using SAH lower-bound pruning
    int32_t find_best_sibling(int32_t leaf) const {
        const FxAABB& la = m_nodes[leaf].fat_aabb;
        float best_cost  = std::numeric_limits<float>::max();
        int32_t best     = m_root;

        struct Entry { int32_t idx; float inherited_cost; };
        std::vector<Entry> stack;
        stack.reserve(32);
        stack.push_back({ m_root, 0.0f });

        while (!stack.empty()) {
            auto [idx, inh] = stack.back();
            stack.pop_back();

            FxAABB combined  = FxAABB::combine(la, m_nodes[idx].fat_aabb);
            float direct     = combined.perimeter();
            float total      = direct + inh;

            if (total < best_cost) {
                best_cost = total;
                best      = idx;
            }

            if (!m_nodes[idx].is_leaf()) {
                float child_inh  = inh + direct - m_nodes[idx].fat_aabb.perimeter();
                float lower_bound = la.perimeter() + child_inh;
                if (lower_bound < best_cost) {
                    stack.push_back({ m_nodes[idx].left,  child_inh });
                    stack.push_back({ m_nodes[idx].right, child_inh });
                }
            }
        }
        return best;
    }

    // -------- leaf removal --------
    void remove_leaf(int32_t leaf) {
        if (m_root == leaf) { m_root = -1; return; }

        int32_t parent  = m_nodes[leaf].parent;
        int32_t grandpa = m_nodes[parent].parent;
        int32_t sibling = (m_nodes[parent].left == leaf)
                        ? m_nodes[parent].right
                        : m_nodes[parent].left;

        if (grandpa >= 0) {
            if (m_nodes[grandpa].left  == parent) m_nodes[grandpa].left  = sibling;
            else                                   m_nodes[grandpa].right = sibling;
            m_nodes[sibling].parent = grandpa;
            free_node(parent);
            refit_from(grandpa);
        } else {
            // parent was the root
            m_root                  = sibling;
            m_nodes[sibling].parent = -1;
            free_node(parent);
        }
        m_nodes[leaf].parent = -1;
    }

    // Refit fat_aabb upward to the root after a structural change
    void refit_from(int32_t idx) {
        while (idx >= 0) {
            int32_t l = m_nodes[idx].left;
            int32_t r = m_nodes[idx].right;
            if      (l >= 0 && r >= 0) m_nodes[idx].fat_aabb = FxAABB::combine(m_nodes[l].fat_aabb, m_nodes[r].fat_aabb);
            else if (l >= 0)           m_nodes[idx].fat_aabb = m_nodes[l].fat_aabb;
            else if (r >= 0)           m_nodes[idx].fat_aabb = m_nodes[r].fat_aabb;
            idx = m_nodes[idx].parent;
        }
    }

    // -------- overlapping pair collection (descent-based) --------
    void collect_pairs(int32_t a, int32_t b,
                       std::vector<std::pair<int32_t, int32_t>>& out,
                       bool same_node) const {
        if (a < 0 || b < 0) return;
        const FxAABBTreeNode& na = m_nodes[a];
        const FxAABBTreeNode& nb = m_nodes[b];

        if (same_node) {
            if (na.is_leaf()) return;
            collect_pairs(na.left,  na.left,  out, true);
            collect_pairs(na.left,  na.right, out, false);
            collect_pairs(na.right, na.right, out, true);
        } else {
            if (!na.fat_aabb.overlaps(nb.fat_aabb)) return;
            if (na.is_leaf() && nb.is_leaf()) {
                int32_t x = na.entity_id, y = nb.entity_id;
                if (x != y) {
                    if (x > y) std::swap(x, y);
                    out.emplace_back(x, y);
                }
                return;
            }
            // Descend into the larger node
            if (nb.is_leaf() || (!na.is_leaf() && na.fat_aabb.perimeter() >= nb.fat_aabb.perimeter())) {
                collect_pairs(na.left,  b, out, false);
                collect_pairs(na.right, b, out, false);
            } else {
                collect_pairs(a, nb.left,  out, false);
                collect_pairs(a, nb.right, out, false);
            }
        }
    }
};
