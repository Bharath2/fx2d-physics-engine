// Fx2D against Box2D v3 on the scenes in scripts/bench_scenes.h. Both run single-threaded at their
// default solver settings, alternating in one process so both see the same CPU clock. Usage and
// what is and is not compared: benchmarks/box2d/README.md.

#include "Fx2D/Scene.h"
#include "bench_scenes.h"

#include <box2d/box2d.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

constexpr float kDt = 1.0f / 60.0f;

struct Options {
    int rounds = 7;
    int steps = 60;
    bool sleep = true; // --no-sleep turns sleeping off in both engines
    int box2d_substeps = 4; // Box2D's default; --box2d-substeps=N changes it
};

bool g_box2d_sleep = true; // the Box2D scene builders take only a body count

// Box2D has one friction coefficient where Fx2D has two; it gets Fx2D's static value. Fx2D's
// default elasticity is 0.1, so Box2D gets restitution 0.1. Density gives each body Fx2D's mass.
void b2_add_box(b2WorldId world, float cx, float cy, float w, float h, bool dynamic) {
    b2BodyDef body = b2DefaultBodyDef();
    body.type = dynamic ? b2_dynamicBody : b2_staticBody;
    body.position = b2ToPos(b2Vec2{cx, cy});
    const b2BodyId id = b2CreateBody(world, &body);
    b2ShapeDef shape = b2DefaultShapeDef();
    shape.density = 1.0f / (w * h);
    shape.material.friction = 0.5f;
    shape.material.restitution = 0.1f;
    const b2Polygon box = b2MakeBox(0.5f * w, 0.5f * h);
    b2CreatePolygonShape(id, &shape, &box);
}

void b2_add_circle(b2WorldId world, float cx, float cy, float radius) {
    b2BodyDef body = b2DefaultBodyDef();
    body.type = b2_dynamicBody;
    body.position = b2ToPos(b2Vec2{cx, cy});
    const b2BodyId id = b2CreateBody(world, &body);
    b2ShapeDef shape = b2DefaultShapeDef();
    shape.density = 1.0f / (3.14159265f * radius * radius);
    shape.material.friction = 0.4f;
    shape.material.restitution = 0.1f;
    const b2Circle circle = {b2Vec2{0.0f, 0.0f}, radius};
    b2CreateCircleShape(id, &shape, &circle);
}

b2WorldId b2_world() {
    b2WorldDef def = b2DefaultWorldDef();
    def.gravity = b2Vec2{0.0f, -10.0f};
    def.enableSleep = g_box2d_sleep;
    return b2CreateWorld(&def);
}

// The same three scenes as FxBench, body for body.
b2WorldId b2_settling_boxes(int body_count) {
    const b2WorldId world = b2_world();
    b2_add_box(world, 200.0f, 1.0f, 380.0f, 1.0f, false);
    const int per_row = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(body_count))));
    for (int i = 0; i < body_count; ++i) {
        b2_add_box(world, 50.0f + static_cast<float>(i % per_row) * 1.5f,
                   2.0f + static_cast<float>(i / per_row) * 1.5f, 1.0f, 1.0f, true);
    }
    return world;
}

b2WorldId b2_pile(int body_count) {
    const b2WorldId world = b2_world();
    const float floor_y = 5.0f;
    const float half_width = 14.0f;
    b2_add_box(world, 100.0f, floor_y, 2.0f * half_width, 1.0f, false);
    b2_add_box(world, 100.0f - half_width, floor_y + 15.0f, 1.0f, 30.0f, false);
    b2_add_box(world, 100.0f + half_width, floor_y + 15.0f, 1.0f, 30.0f, false);
    const int per_row = 24;
    for (int i = 0; i < body_count; ++i) {
        const int row = i / per_row;
        const int col = i % per_row;
        const float stagger = (row % 2 == 0) ? 0.0f : 0.5f;
        b2_add_circle(world, 87.5f + (static_cast<float>(col) + stagger) * 1.05f,
                      floor_y + 1.5f + static_cast<float>(row) * 1.05f, 0.5f);
    }
    return world;
}

b2WorldId b2_stacks(int body_count) {
    const b2WorldId world = b2_world();
    b2_add_box(world, 200.0f, 1.0f, 380.0f, 1.0f, false);
    const int column_height = 10;
    const int columns = (body_count + column_height - 1) / column_height;
    int made = 0;
    for (int c = 0; c < columns && made < body_count; ++c) {
        for (int r = 0; r < column_height && made < body_count; ++r, ++made) {
            b2_add_box(world, 30.0f + static_cast<float>(c) * 3.0f,
                       2.0f + static_cast<float>(r) * 1.01f, 1.0f, 1.0f, true);
        }
    }
    return world;
}

struct Timing {
    double ms_per_step = 0.0;
    double mean_contacts = 0.0;
};

double seconds_since(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

Timing time_fx2d(const FxBench::Scene& scene, int bodies, int steps, bool sleep) {
    FxScene world = scene.build(bodies);
    if (!sleep) {
        world.for_each_entity(std::execution::seq,
                              [](const auto& e) { e->sleep_time_required = FxInfinityf; });
    }
    for (int i = 0; i < scene.warmup_steps; ++i)
        world.step(kDt);
    double contacts = 0.0;
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < steps; ++i) {
        world.step(kDt);
        contacts += static_cast<double>(world.contacts().size());
    }
    return {seconds_since(start) * 1e3 / steps, contacts / steps};
}

Timing time_box2d(b2WorldId (*build)(int), int warmup, int bodies, int steps, int substeps) {
    const b2WorldId world = build(bodies);
    for (int i = 0; i < warmup; ++i)
        b2World_Step(world, kDt, substeps);
    double contacts = 0.0;
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < steps; ++i) {
        b2World_Step(world, kDt, substeps);
        contacts += static_cast<double>(b2World_GetCounters(world).contactCount);
    }
    const double ms = seconds_since(start) * 1e3 / steps;
    b2DestroyWorld(world);
    return {ms, contacts / steps};
}

double median(std::vector<double> v) {
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

} // namespace

int main(int argc, char** argv) {
    Options opt;
    int positional = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--no-sleep") opt.sleep = false;
        else if (arg.rfind("--box2d-substeps=", 0) == 0)
            opt.box2d_substeps = std::atoi(argv[i] + 17);
        else if (positional++ == 0) opt.rounds = std::atoi(argv[i]);
        else opt.steps = std::atoi(argv[i]);
    }
    if (opt.rounds <= 0 || opt.steps <= 0 || opt.box2d_substeps <= 0) {
        std::fprintf(stderr,
                     "usage: fx2d_vs_box2d [rounds] [steps] [--no-sleep] [--box2d-substeps=N]\n");
        return 1;
    }
    g_box2d_sleep = opt.sleep;

    b2WorldId (*const b2_scenes[])(int) = {b2_settling_boxes, b2_pile, b2_stacks};
    const auto& scenes = FxBench::scenes();

    std::printf("Fx2D vs Box2D %d.%d.%d, %d rounds of %d steps, median ms/step, one thread each\n",
                b2GetVersion().major, b2GetVersion().minor, b2GetVersion().revision, opt.rounds,
                opt.steps);
    std::printf("sleeping %s; Box2D at %d substeps, Fx2D at its default\n",
                opt.sleep ? "on" : "off", opt.box2d_substeps);
    std::printf("contacts: Fx2D counts touching pairs, Box2D its contact list (AABB overlaps)\n\n");
    std::printf("%-15s %6s %11s %11s %9s %13s %13s\n", "scene", "bodies", "Fx2D", "Box2D",
                "Fx2D/B2D", "Fx2D contacts", "B2D contacts");
    for (std::size_t s = 0; s < scenes.size(); ++s) {
        for (int bodies : scenes[s].sizes) {
            if (bodies < 200) continue; // too small to time against Box2D's per-step overhead
            std::vector<double> fx, b2, ratio;
            Timing last_fx, last_b2;
            for (int r = 0; r < opt.rounds; ++r) {
                // Alternate which engine goes first, so ordering cannot favour either.
                if (r % 2 == 0) {
                    last_fx = time_fx2d(scenes[s], bodies, opt.steps, opt.sleep);
                    last_b2 = time_box2d(b2_scenes[s], scenes[s].warmup_steps, bodies, opt.steps,
                                         opt.box2d_substeps);
                } else {
                    last_b2 = time_box2d(b2_scenes[s], scenes[s].warmup_steps, bodies, opt.steps,
                                         opt.box2d_substeps);
                    last_fx = time_fx2d(scenes[s], bodies, opt.steps, opt.sleep);
                }
                fx.push_back(last_fx.ms_per_step);
                b2.push_back(last_b2.ms_per_step);
                ratio.push_back(last_fx.ms_per_step / last_b2.ms_per_step);
            }
            std::printf("%-15s %6d %8.3f ms %8.3f ms %8.2fx %13.1f %13.1f\n", scenes[s].name,
                        bodies, median(fx), median(b2), median(ratio), last_fx.mean_contacts,
                        last_b2.mean_contacts);
            std::fflush(stdout);
        }
    }
    return 0;
}
