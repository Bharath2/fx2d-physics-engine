// Contact graph coloring. The batched solve rests on one invariant: within a color, no two
// pairs touch the same movable body. Breaking it looks like a solver that quietly got worse at
// stacks, so this checks it directly on hand-built pair lists rather than through simulation.

#include "Fx2D/Entity.h"
#include "Fx2D/Solver.h"

#include "test_harness.h"
#include "test_scene_builders.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace {

using PairList = std::vector<std::pair<size_t, size_t>>;
using Entities = std::vector<std::shared_ptr<FxEntity>>;

// n bodies, all movable unless listed in `immovable`. Immovable here means what it means to the
// solver: zero inverse mass and zero inverse inertia, so no impulse can shift it.
Entities make_entities(size_t n, const std::set<size_t>& immovable = {}) {
    Entities entities;
    for (size_t i = 0; i < n; ++i) {
        auto e = std::make_shared<FxEntity>("e" + std::to_string(i));
        if (immovable.find(i) != immovable.end()) {
            e->set_mass(0.0f);
            e->set_inertia(0.0f);
            e->enable_external_forces(false);
        } else {
            e->set_mass(1.0f);
            e->set_inertia(1.0f);
        }
        entities.push_back(std::move(e));
    }
    return entities;
}

bool movable(const Entities& entities, size_t body) {
    return entities[body]->inv_mass() != 0.0f || entities[body]->inv_inertia() != 0.0f;
}

// The invariant, checked exhaustively over the assigned colors: two pairs may share a color
// only if they share no movable body. Overflow (kMaxColors) is exempt -- those are solved one
// at a time precisely because no color would take them.
void require_colors_independent(const FxContactGraph& graph, const PairList& pairs,
                                const Entities& entities, const std::string& scenario) {
    for (size_t i = 0; i < pairs.size(); ++i) {
        const uint32_t color = graph.color_of_pair(i);
        if (color >= FxContactGraph::kMaxColors) continue;
        for (size_t j = i + 1; j < pairs.size(); ++j) {
            if (graph.color_of_pair(j) != color) continue;
            for (size_t body : {pairs[i].first, pairs[i].second}) {
                if (!movable(entities, body)) continue;
                require(body != pairs[j].first && body != pairs[j].second,
                        scenario + ": pairs " + std::to_string(i) + " and " + std::to_string(j) +
                            " share movable body " + std::to_string(body) + " in color " +
                            std::to_string(color));
            }
        }
    }
}

// Every contact must end up in exactly one group, and the groups must tile the order.
void require_grouping_covers(FxContactGraph& graph, const std::vector<uint32_t>& contact_colors,
                             const std::string& scenario) {
    std::vector<uint32_t> order;
    graph.group_contacts(contact_colors, order);

    require(order.size() == contact_colors.size(),
            scenario + ": grouping dropped or duplicated contacts (" +
                std::to_string(order.size()) + " of " + std::to_string(contact_colors.size()) +
                ")");
    std::vector<int> seen(contact_colors.size(), 0);
    for (uint32_t index : order) {
        require(index < contact_colors.size(), scenario + ": grouped index out of range");
        ++seen[index];
    }
    for (size_t i = 0; i < seen.size(); ++i)
        require(seen[i] == 1, scenario + ": contact " + std::to_string(i) + " grouped " +
                                  std::to_string(seen[i]) + " times");

    require(graph.group_start(0) == 0, scenario + ": first group does not start at 0");
    require(graph.group_start(graph.group_count()) == order.size(),
            scenario + ": groups do not cover the whole order");

    // Every contact inside a group must carry the same color.
    for (size_t g = 0; g < graph.group_count(); ++g) {
        require(graph.group_start(g) < graph.group_start(g + 1),
                scenario + ": group " + std::to_string(g) + " is empty");
        const uint32_t first = contact_colors[order[graph.group_start(g)]];
        for (uint32_t k = graph.group_start(g); k < graph.group_start(g + 1); ++k) {
            require(contact_colors[order[k]] == first,
                    scenario + ": group " + std::to_string(g) + " mixes colors");
        }
    }
}

// Colors for a contact set that mirrors the pair set one-for-one, which is the easy case: every
// pair produced a contact.
std::vector<uint32_t> colors_for_all(const FxContactGraph& graph, size_t pair_count) {
    std::vector<uint32_t> colors;
    for (size_t i = 0; i < pair_count; ++i)
        colors.push_back(graph.color_of_pair(i));
    return colors;
}

// A chain of boxes, each touching the next: every pair shares a body with its neighbours, so a
// correct coloring needs exactly two colors and must not use one.
void test_chain_needs_two_colors() {
    const Entities entities = make_entities(8);
    PairList pairs;
    for (size_t i = 0; i + 1 < 8; ++i)
        pairs.emplace_back(i, i + 1);

    FxContactGraph graph;
    graph.color_pairs(pairs, entities);

    require_colors_independent(graph, pairs, entities, "chain");
    require(graph.overflow_count() == 0, "a chain should never overflow");
    // Groups only exist once contacts have been grouped: the pair list is a superset, so which
    // colors survive is not known until the narrow phase has said which pairs actually touched.
    require_grouping_covers(graph, colors_for_all(graph, pairs.size()), "chain");
    require(graph.group_count() == 2,
            "a chain of 7 pairs should color into exactly 2 groups, got " +
                std::to_string(graph.group_count()));
}

// Every box on one static ground. The ground is immovable, so nothing conflicts and the whole
// set is one color. The case that matters most: if the ground split colors, in a pile or a
// stack there would be nothing left to batch.
void test_shared_static_body_does_not_split_colors() {
    const Entities entities = make_entities(21, {0}); // body 0 is the ground
    PairList pairs;
    for (size_t i = 1; i < 21; ++i)
        pairs.emplace_back(0, i);

    FxContactGraph graph;
    graph.color_pairs(pairs, entities);

    require_colors_independent(graph, pairs, entities, "static ground");
    require_grouping_covers(graph, colors_for_all(graph, pairs.size()), "static ground");
    require(graph.group_count() == 1,
            "20 pairs against one immovable body should be a single color, got " +
                std::to_string(graph.group_count()));
}

// A sleeping body is still colored as movable, because the narrow phase can wake it partway
// through the step and the partition has to remain valid when it does.
void test_sleeping_body_is_treated_as_movable() {
    Entities entities = make_entities(4);
    entities[0]->sleep();
    const PairList pairs = {{0, 1}, {0, 2}, {0, 3}};

    FxContactGraph graph;
    graph.color_pairs(pairs, entities);
    std::vector<uint32_t> order;
    graph.group_contacts({graph.color_of_pair(0), graph.color_of_pair(1), graph.color_of_pair(2)},
                         order);

    require(graph.group_count() == 3,
            "three pairs on one sleeping-but-movable body need three colors, got " +
                std::to_string(graph.group_count()));
}

// More pairs on one movable body than there are colors. The excess cannot be colored and must
// be marked as overflow rather than silently sharing a color.
void test_overflow_is_captured_not_lost() {
    const size_t spokes = FxContactGraph::kMaxColors + 5;
    const Entities entities = make_entities(spokes + 1);
    PairList pairs;
    for (size_t i = 1; i <= spokes; ++i)
        pairs.emplace_back(0, i);

    FxContactGraph graph;
    graph.color_pairs(pairs, entities);

    require_colors_independent(graph, pairs, entities, "overflow");
    require(graph.colored_count() == FxContactGraph::kMaxColors,
            "every color should be filled before overflowing, got " +
                std::to_string(graph.colored_count()));
    require(graph.overflow_count() == 5,
            "5 pairs should overflow, got " + std::to_string(graph.overflow_count()));
    require_grouping_covers(graph, colors_for_all(graph, pairs.size()), "overflow");
    require(graph.is_overflow_group(graph.group_count() - 1), "the overflow group must be last");
}

// Only some pairs produce contacts, which is the normal case -- the pair list is a superset.
// The grouping must cope with an arbitrary subset in detection order.
void test_grouping_handles_a_contact_subset() {
    const Entities entities = make_entities(10);
    PairList pairs;
    for (size_t i = 0; i + 1 < 10; ++i)
        pairs.emplace_back(i, i + 1);

    FxContactGraph graph;
    graph.color_pairs(pairs, entities);

    // Every other pair "touched".
    std::vector<uint32_t> contact_colors;
    for (size_t i = 0; i < pairs.size(); i += 2)
        contact_colors.push_back(graph.color_of_pair(i));

    require_grouping_covers(graph, contact_colors, "subset");
}

// The partition feeds a solver whose results must be reproducible, so the coloring itself has
// to be a pure function of its inputs.
void test_coloring_is_deterministic() {
    const Entities entities = make_entities(12);
    PairList pairs;
    for (size_t i = 0; i + 1 < 12; ++i) {
        pairs.emplace_back(i, i + 1);
        if (i >= 2) pairs.emplace_back(i - 2, i);
    }

    FxContactGraph graph_a;
    FxContactGraph graph_b;
    graph_a.color_pairs(pairs, entities);
    graph_b.color_pairs(pairs, entities);

    for (size_t i = 0; i < pairs.size(); ++i) {
        require(graph_a.color_of_pair(i) == graph_b.color_of_pair(i),
                "coloring must be identical between builds, pair " + std::to_string(i));
    }
    require_colors_independent(graph_a, pairs, entities, "deterministic");

    // Reusing one graph object must not leak state from the previous call either.
    FxContactGraph reused;
    reused.color_pairs(pairs, entities);
    reused.color_pairs(pairs, entities);
    for (size_t i = 0; i < pairs.size(); ++i) {
        require(reused.color_of_pair(i) == graph_a.color_of_pair(i),
                "recoloring through the same object must not drift");
    }
}

// An empty step is a real case: every body asleep, or nothing near anything else.
void test_empty_input() {
    const Entities entities = make_entities(4);
    const PairList pairs;
    FxContactGraph graph;
    graph.color_pairs(pairs, entities);
    require(graph.group_count() == 0, "no pairs means no groups");

    std::vector<uint32_t> order;
    graph.group_contacts({}, order);
    require(order.empty(), "no contacts means an empty order");
}

// Drives the overflow colour through a real scene: this one puts five pairs into it, and no
// other suite reaches overflow at all. The assertion is momentum conservation across the
// collision, which any lost contact impulse would break.

// It does not prove the overflow group *needs* solving one contact at a time. No scene here
// produces two overflow contacts sharing a movable body, the case that would actually drop an
// impulse, so that handling is correct by construction rather than by reproduction.
void test_overflow_group_is_solved_without_losing_impulses() {
    FxScene scene = ::make_scene(FxVec2f{60.0f, 60.0f}, 0.0f);

    const FxVec2f centre{30.0f, 30.0f};
    const size_t spokes = FxContactGraph::kMaxColors + 8;
    std::vector<std::shared_ptr<FxEntity>> bodies;

    bodies.push_back(::add_circle(scene, "hub", centre, 2.0f, {.mass = 4.0f}));

    // Fired inward together, so every spoke is in contact with the hub at the same moment and
    // the hub's pair count runs past the colour cap.
    for (size_t i = 0; i < spokes; ++i) {
        const float a = 2.0f * FxPif * static_cast<float>(i) / static_cast<float>(spokes);
        const FxVec2f dir{std::cos(a), std::sin(a)};
        const FxVec2f at = centre + dir * 3.0f;
        // Speeds vary around the ring on purpose. Fired evenly, the impulses the hub should
        // receive cancel by symmetry, and impulses dropped from a symmetric sum are invisible.
        const float speed = 3.0f + 6.0f * static_cast<float>(i) / static_cast<float>(spokes);
        auto e = ::add_circle(scene, "s" + std::to_string(i), at, 0.6f, {.mass = 1.0f});
        e->velocity = FxVec3f{-dir.x() * speed, -dir.y() * speed, 0.0f};
        bodies.push_back(e);
    }

    auto momentum = [&]() {
        FxVec2f p{0.0f, 0.0f};
        for (const auto& e : bodies)
            p += e->velocity.get_xy() * e->mass();
        return p;
    };

    // Compared against the start, not against zero: the ring carries real momentum by design.
    const FxVec2f before = momentum();
    for (int i = 0; i < 90; ++i)
        scene.step(1.0 / 60.0);
    const FxVec2f after = momentum();

    const float drift = (after - before).norm();
    require(drift < 0.05f,
            "contact impulses must conserve momentum; total drifted by " + std::to_string(drift));
}

} // namespace

void run_contact_graph_tests() {
    test_empty_input();
    test_chain_needs_two_colors();
    test_shared_static_body_does_not_split_colors();
    test_sleeping_body_is_treated_as_movable();
    test_overflow_is_captured_not_lost();
    test_grouping_handles_a_contact_subset();
    test_coloring_is_deterministic();
    test_overflow_group_is_solved_without_losing_impulses();
    std::cout << "Contact graph tests passed." << std::endl;
}
