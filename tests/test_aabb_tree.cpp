#include "Fx2D/AABBTree.h"

#include "test_harness.h"

#include <iostream>

namespace {

// Helper: build a square AABB centred at (cx,cy) with half-extent h
FxAABB box(float cx, float cy, float h) {
    return {cx - h, cy - h, cx + h, cy + h};
}

// 1. Single leaf → no pairs
void test_single_leaf_no_pairs() {
    FxAABBTree t;
    t.insert(0, box(1.0f, 1.0f, 0.5f));
    std::vector<std::pair<int32_t, int32_t>> pairs;
    t.query_pairs(pairs);
    require(pairs.empty(), "a single leaf must not produce any pair");
}

// 2. Two overlapping leaves → exactly one pair
void test_two_overlapping_one_pair() {
    FxAABBTree t;
    t.insert(0, box(0.0f, 0.0f, 1.0f));
    t.insert(1, box(1.0f, 0.0f, 1.0f)); // overlaps leaf 0
    std::vector<std::pair<int32_t, int32_t>> pairs;
    t.query_pairs(pairs);
    require(pairs.size() == 1, "two overlapping leaves must produce exactly one pair");
    require(pairs[0].first == 0 && pairs[0].second == 1,
            "the reported pair must carry both leaf ids in ascending order");
}

// 3. Two non-overlapping leaves → no pairs
void test_two_separated_no_pairs() {
    FxAABBTree t;
    t.insert(0, box(0.0f, 0.0f, 0.4f));
    t.insert(1, box(5.0f, 5.0f, 0.4f)); // far away
    std::vector<std::pair<int32_t, int32_t>> pairs;
    t.query_pairs(pairs);
    require(pairs.empty(), "separated leaves must not produce a pair");
}

// 4. Remove a leaf → tree back to no pairs
void test_remove_restores_empty_pairs() {
    FxAABBTree t;
    t.insert(0, box(0.0f, 0.0f, 1.0f));
    int32_t n1 = t.insert(1, box(0.5f, 0.0f, 1.0f));
    t.remove(n1);
    std::vector<std::pair<int32_t, int32_t>> pairs;
    t.query_pairs(pairs);
    require(pairs.empty(), "removing one of two overlapping leaves must clear the pair");
}

// 5. update() outside fat box → returns true, pair still detected
void test_update_outside_fat_box_returns_true() {
    FxAABBTree t;
    t.insert(0, box(0.0f, 0.0f, 1.0f));
    int32_t n1 = t.insert(1, box(20.0f, 20.0f, 0.1f)); // start far away
    // Move leaf 1 close enough to overlap leaf 0; this must escape its fat box
    bool reinserted = t.update(n1, box(0.5f, 0.0f, 1.0f));
    require(reinserted, "a leaf moved outside its fat box must be reinserted");
    std::vector<std::pair<int32_t, int32_t>> pairs;
    t.query_pairs(pairs);
    require(pairs.size() == 1, "the reinserted leaf must be found overlapping leaf 0");
}

// 6. update() inside fat box → returns false (no reinsertion needed)
void test_update_inside_fat_box_returns_false() {
    FxAABBTree t;
    int32_t n0 = t.insert(0, box(0.0f, 0.0f, 0.4f));
    // Tiny nudge; stays well within the 20% fat margin
    bool reinserted = t.update(n0, box(0.01f, 0.01f, 0.4f));
    require(!reinserted, "a leaf nudged inside its fat box must not be reinserted");
}

} // namespace

void run_aabb_tree_tests() {
    test_single_leaf_no_pairs();
    test_two_overlapping_one_pair();
    test_two_separated_no_pairs();
    test_remove_restores_empty_pairs();
    test_update_outside_fat_box_returns_true();
    test_update_inside_fat_box_returns_false();
    std::cout << "AABB tree tests passed." << std::endl;
}
