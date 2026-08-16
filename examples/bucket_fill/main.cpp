// A floating bucket, balls raining until it fills and overflows. Space pauses the rain, C
// clears, R resets. Build with -DFX2D_BUILD_EXAMPLES=ON and run from the repo root.

#include "Fx2D/Core.h"

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr int kMaxBalls = 300;
constexpr int kStepsPerSpawn = 6;

const FxVec4ui8 kColours[] = {{232, 122, 92, 255},
                              {110, 176, 232, 255},
                              {236, 200, 104, 255},
                              {156, 208, 132, 255},
                              {196, 140, 224, 255}};

} // namespace

int main(int, char**) {
    FxScene scene = FxYAML::buildScene("examples/bucket_fill/Scene.yml");
    if (!scene.get_entity("bucket_bottom")) {
        std::fprintf(stderr, "bucket_fill: Scene.yml did not build the bucket\n");
        return 1;
    }

    // Runtime-spawned balls live in a group: one delete on clear, and reset removes them
    // wholesale because they are not part of the captured composition.
    auto balls = scene.create_group("balls", /*self_collide=*/true);

    int spawned = 0;
    int in_bucket = 0;
    int step_count = 0;
    bool raining = true;

    scene.set_reset_callback([&](FxScene& s) {
        // reset() restores the group from the snapshot, so re-fetch it rather than re-create:
        // create_group would find the name taken, return nullptr, and kill the rain.
        balls = s.get_group("balls");
        if (!balls) balls = s.create_group("balls", true);
        spawned = 0;
        step_count = 0;
        raining = true;
    });

    scene.set_step_callback([&](FxScene& s, double) {
        if (s.input().key_pressed(FxKey::R)) {
            s.reset();
            return; // the reset callback re-armed everything; nothing valid to do this step
        }
        if (s.input().key_pressed(FxKey::Space)) raining = !raining;
        if (s.input().key_pressed(FxKey::C)) {
            s.delete_group("balls");
            balls = s.create_group("balls", true);
            spawned = 0;
        }

        ++step_count;
        if (raining && spawned < kMaxBalls && step_count % kStepsPerSpawn == 0) {
            // Deterministic spread, a little wider than the bucket mouth so overflow spills
            // down both sides once the bucket is full.
            const float jitter = 1.7f * std::sin(static_cast<float>(spawned) * 2.399f);
            const float radius = 0.16f + 0.05f * static_cast<float>(spawned % 3);

            auto ball =
                std::make_shared<FxEntity>(s.unique_entity_name("ball_" + std::to_string(spawned)));
            FxVisualShape visual(radius);
            visual.set_fillColor(kColours[spawned % 5]);
            visual.set_outlineColor(FxVec4ui8{18, 20, 28, 255});
            visual.set_outlineThickness(1.5f);
            ball->set_visual_geometry(visual);
            ball->set_collision_geometry(FxCollisionShape(radius));
            ball->set_init_pose(FxVec3f{8.0f + jitter, 11.4f, 0.0f});
            ball->set_mass(0.6f);
            ball->set_inertia();
            ball->elasticity = 0.1f;
            ball->static_friction = 0.4f;
            ball->dynamic_friction = 0.3f;
            if (s.add_to_group(balls, ball)) {
                ball->reset();
                ++spawned;
            }
        }

        // The bucket interior, asked for with an overlap query rather than tracked by hand.
        std::vector<std::shared_ptr<FxEntity>> inside;
        s.overlap_box(FxVec2f{8.0f, 6.6f}, FxVec2f{2.4f, 2.8f}, inside);
        int count = 0;
        for (const auto& e : inside)
            if (e && e->get_name().rfind("ball_", 0) == 0) ++count;
        in_bucket = count;
    });

    FxRylbRenderer renderer(scene, 60, 80);

    renderer.set_draw_callback([&](FxRylbRenderer& r) {
        const std::string hud =
            "balls: " + std::to_string(spawned) + "/" + std::to_string(kMaxBalls) +
            "    in bucket: " + std::to_string(in_bucket) +
            "    overflowed: " + std::to_string(spawned - in_bucket) +
            (raining ? "" : "    [paused]") + "    [space] rain  [C] clear  [R] reset";
        DrawText(hud.c_str(), 12, 12, 18, to_rl_color(FxVec4ui8{226, 232, 240, 255}));
        (void)r;
    });

    renderer.run(true);
    return 0;
}
