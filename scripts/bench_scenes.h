#pragma once

// The benchmark scenes. Fx2DBench times them, and benchmarks/box2d builds the same scenes in Box2D,
// so both measure the same thing. Kept apart from the test builders on purpose: the benchmark is
// a standalone tool and should not depend on test scaffolding.

#include "Fx2D/Scene.h"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace FxBench {

inline std::shared_ptr<FxEntity> add_box(FxScene& scene, const std::string& name, float cx,
                                         float cy, float w, float h, float mass) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_visual_geometry(FxVisualShape(FxVec2f{w, h}));
    e->set_collision_geometry(FxCollisionShape(FxVec2f{w, h}));
    e->set_init_pose(FxVec3f{cx, cy, 0.0f});
    e->set_mass(mass);
    e->set_inertia();
    e->static_friction = 0.5f;
    e->dynamic_friction = 0.4f;
    scene.add_entity(e);
    return e;
}

inline std::shared_ptr<FxEntity> add_circle(FxScene& scene, const std::string& name, float cx,
                                            float cy, float radius, float mass) {
    auto e = std::make_shared<FxEntity>(name);
    e->set_visual_geometry(FxVisualShape(radius));
    e->set_collision_geometry(FxCollisionShape(radius));
    e->set_init_pose(FxVec3f{cx, cy, 0.0f});
    e->set_mass(mass);
    e->set_inertia();
    e->static_friction = 0.4f;
    e->dynamic_friction = 0.3f;
    scene.add_entity(e);
    return e;
}

// Immovable scenery: zero mass, no external forces, no gravity.
inline void make_static(const std::shared_ptr<FxEntity>& e) {
    e->set_mass(0.0f);
    e->set_inertia(0.0f);
    e->enable_external_forces(false);
    e->gravity_scale = 0.0f;
}

// A loose grid of boxes settling onto the ground. Sparse contacts; dominated by per-body work.
inline FxScene settling_boxes(int body_count) {
    FxScene scene({400.0f, 400.0f});
    scene.set_gravity(FxVec2f{0.0f, -10.0f});

    make_static(add_box(scene, "ground", 200.0f, 1.0f, 380.0f, 1.0f, 1.0f));

    const int per_row = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(body_count))));
    for (int i = 0; i < body_count; ++i) {
        const float x = 50.0f + static_cast<float>(i % per_row) * 1.5f;
        const float y = 2.0f + static_cast<float>(i / per_row) * 1.5f;
        add_box(scene, "b" + std::to_string(i), x, y, 1.0f, 1.0f, 1.0f);
    }
    return scene;
}

// Circles packed into a walled container. This is the contact-dense case the solver profile was
// taken on: every body touches several neighbours, so the velocity passes carry the step.
inline FxScene pile(int body_count) {
    FxScene scene({200.0f, 200.0f});
    scene.set_gravity(FxVec2f{0.0f, -10.0f});

    const float floor_y = 5.0f;
    const float half_width = 14.0f;
    make_static(add_box(scene, "floor", 100.0f, floor_y, 2.0f * half_width, 1.0f, 1.0f));
    make_static(add_box(scene, "wall_l", 100.0f - half_width, floor_y + 15.0f, 1.0f, 30.0f, 1.0f));
    make_static(add_box(scene, "wall_r", 100.0f + half_width, floor_y + 15.0f, 1.0f, 30.0f, 1.0f));

    // Staggered rows so the pack settles into a dense pile rather than a lattice that never
    // rearranges. Radius 0.5 against a 26-wide floor gives roughly 24 per row.
    const int per_row = 24;
    for (int i = 0; i < body_count; ++i) {
        const int row = i / per_row;
        const int col = i % per_row;
        const float stagger = (row % 2 == 0) ? 0.0f : 0.5f;
        const float x = 87.5f + (static_cast<float>(col) + stagger) * 1.05f;
        const float y = floor_y + 1.5f + static_cast<float>(row) * 1.05f;
        add_circle(scene, "p" + std::to_string(i), x, y, 0.5f, 1.0f);
    }
    return scene;
}

// Free-standing columns of boxes. Deep contact chains with few contacts per body -- the case
// where solver convergence, not contact volume, decides the cost.
inline FxScene stacks(int body_count) {
    FxScene scene({400.0f, 200.0f});
    scene.set_gravity(FxVec2f{0.0f, -10.0f});

    make_static(add_box(scene, "ground", 200.0f, 1.0f, 380.0f, 1.0f, 1.0f));

    const int column_height = 10;
    const int columns = (body_count + column_height - 1) / column_height;
    int made = 0;
    for (int c = 0; c < columns && made < body_count; ++c) {
        const float x = 30.0f + static_cast<float>(c) * 3.0f;
        for (int r = 0; r < column_height && made < body_count; ++r, ++made) {
            add_box(scene, "s" + std::to_string(made), x, 2.0f + static_cast<float>(r) * 1.01f,
                    1.0f, 1.0f, 1.0f);
        }
    }
    return scene;
}

struct Scene {
    const char* name;
    FxScene (*build)(int body_count);
    int warmup_steps; // untimed; the pile needs longer, so its contact count is representative
    std::vector<int> sizes;
};

inline const std::vector<Scene>& scenes() {
    static const std::vector<Scene> all = {
        {"settling_boxes", &settling_boxes, 20, {10, 50, 200, 400, 800, 1600, 3000}},
        {"pile", &pile, 120, {50, 150, 300, 600, 1200}},
        {"stacks", &stacks, 60, {50, 200, 600}},
    };
    return all;
}

} // namespace FxBench
