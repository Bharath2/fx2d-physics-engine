#include "Fx2D/AABBTree.h"
#include <cassert>
#include <iostream>

// Helper: build a square AABB centred at (cx,cy) with half-extent h
static FxAABB box(float cx, float cy, float h) {
    return {cx - h, cy - h, cx + h, cy + h};
}

// 1. Single leaf → no pairs
static void test_single_leaf_no_pairs() {
    FxAABBTree t;
    t.insert(0, box(1.0f, 1.0f, 0.5f));
    std::vector<std::pair<int32_t, int32_t>> pairs;
    t.query_pairs(pairs);
    assert(pairs.empty());
}

// 2. Two overlapping leaves → exactly one pair
static void test_two_overlapping_one_pair() {
    FxAABBTree t;
    t.insert(0, box(0.0f, 0.0f, 1.0f));
    t.insert(1, box(1.0f, 0.0f, 1.0f)); // overlaps leaf 0
    std::vector<std::pair<int32_t, int32_t>> pairs;
    t.query_pairs(pairs);
    assert(pairs.size() == 1);
    assert(pairs[0].first == 0 && pairs[0].second == 1);
}

// 3. Two non-overlapping leaves → no pairs
static void test_two_separated_no_pairs() {
    FxAABBTree t;
    t.insert(0, box(0.0f, 0.0f, 0.4f));
    t.insert(1, box(5.0f, 5.0f, 0.4f)); // far away
    std::vector<std::pair<int32_t, int32_t>> pairs;
    t.query_pairs(pairs);
    assert(pairs.empty());
}

// 4. Remove a leaf → tree back to no pairs
static void test_remove_restores_empty_pairs() {
    FxAABBTree t;
    int32_t n0 = t.insert(0, box(0.0f, 0.0f, 1.0f));
    int32_t n1 = t.insert(1, box(0.5f, 0.0f, 1.0f));
    t.remove(n1);
    std::vector<std::pair<int32_t, int32_t>> pairs;
    t.query_pairs(pairs);
    assert(pairs.empty());
    (void)n0;
}

// 5. update() outside fat box → returns true, pair still detected
static void test_update_outside_fat_box_returns_true() {
    FxAABBTree t;
    int32_t n0 = t.insert(0, box(0.0f, 0.0f, 1.0f));
    int32_t n1 = t.insert(1, box(20.0f, 20.0f, 0.1f)); // start far away
    // Move leaf 1 close enough to overlap leaf 0; this must escape its fat box
    bool reinserted = t.update(n1, box(0.5f, 0.0f, 1.0f));
    assert(reinserted == true);
    std::vector<std::pair<int32_t, int32_t>> pairs;
    t.query_pairs(pairs);
    assert(pairs.size() == 1);
    (void)n0;
}

// 6. update() inside fat box → returns false (no reinsertion needed)
static void test_update_inside_fat_box_returns_false() {
    FxAABBTree t;
    int32_t n0 = t.insert(0, box(0.0f, 0.0f, 0.4f));
    // Tiny nudge; stays well within the 20% fat margin
    bool reinserted = t.update(n0, box(0.01f, 0.01f, 0.4f));
    assert(reinserted == false);
}

void run_aabb_tree_tests() {
    test_single_leaf_no_pairs();
    test_two_overlapping_one_pair();
    test_two_separated_no_pairs();
    test_remove_restores_empty_pairs();
    test_update_outside_fat_box_returns_true();
    test_update_inside_fat_box_returns_false();
    std::cout << "AABB tree tests passed." << std::endl;
}
