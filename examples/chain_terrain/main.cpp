// Terrain built from one chain entity; click to drop balls on it, C clears, R resets.
// Build with -DFX2D_BUILD_EXAMPLES=ON and run from the repo root.

#include "Fx2D/Core.h"

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr int kMaxBalls = 40;

const FxVec4ui8 kBallColours[] = {{232, 122, 92, 255},
                                  {110, 176, 232, 255},
                                  {236, 200, 104, 255},
                                  {156, 208, 132, 255},
                                  {196, 140, 224, 255}};

Color to_rl(const FxVec4ui8& c) {
    return Color{c[0], c[1], c[2], c[3]};
}

} // namespace

int main(int, char**) {
    FxScene scene = FxYAML::buildScene("examples/chain_terrain/Scene.yml");

    auto terrain = scene.get_entity("terrain");
    if (!terrain || !terrain->collision_geometry()->is_chain()) {
        std::fprintf(stderr, "chain_terrain: Scene.yml did not yield a chain collider\n");
        return 1;
    }

    int spawned = 0;
    int resting = 0;

    // Dropped balls are added at runtime, so a scene reset removes them: reset() restores the
    // composition captured on the first step, which is the terrain and the floor alone.
    scene.set_reset_callback([&](FxScene&) { spawned = 0; });

    scene.set_step_callback([&](FxScene& s, double) {
        const FxInput& in = s.input();

        if (in.mouse_pressed(FxMouseButton::Left) && spawned < kMaxBalls) {
            const FxVec2f at = in.mouse_position();
            auto ball = std::make_shared<FxEntity>("ball_" + std::to_string(spawned));
            const float r = 0.18f + 0.10f * static_cast<float>(spawned % 3);
            FxVisualShape visual(r);
            visual.set_fillColor(kBallColours[spawned % 5]);
            visual.set_outlineColor(FxVec4ui8{20, 24, 34, 255});
            visual.set_outlineThickness(2.0f);
            ball->set_visual_geometry(visual);
            ball->set_collision_geometry(FxCollisionShape(r));
            ball->set_init_pose(FxVec3f{at.x(), at.y(), 0.0f});
            ball->set_mass(1.0f);
            ball->set_inertia();
            ball->elasticity = 0.25f;
            ball->static_friction = 0.4f;
            ball->dynamic_friction = 0.3f;
            if (s.add_entity(ball)) {
                ball->reset();
                ++spawned;
            }
        }

        if (in.key_pressed(FxKey::R)) {
            s.reset();
            return;
        }
        if (in.key_pressed(FxKey::C)) {
            for (int i = 0; i < spawned; ++i)
                s.delete_entity("ball_" + std::to_string(i));
            spawned = 0;
        }

        int asleep = 0;
        for (int i = 0; i < spawned; ++i) {
            if (auto b = s.get_entity("ball_" + std::to_string(i))) {
                if (b->is_sleeping()) ++asleep;
            }
        }
        resting = asleep;
    });

    FxRylbRenderer renderer(scene, 60, 70);

    renderer.set_draw_callback([&](FxRylbRenderer& r) {
        // Mark the chain's vertices, so the segment joints the collider is built from are
        // visible rather than implied.
        const auto& verts = terrain->collision_geometry()->vertices();
        for (size_t i = 0; i < verts.size(); ++i) {
            const FxVec2f s = r.world_to_screen(verts[i]);
            DrawCircleV({s.x(), s.y()}, 3.0f, Color{240, 240, 245, 200});
        }

        const std::string hud =
            "chain: " + std::to_string(verts.size()) + " points, " +
            std::to_string(terrain->collision_geometry()->segment_count()) + " segments" +
            "    balls: " + std::to_string(spawned) + "/" + std::to_string(kMaxBalls) +
            "  resting: " + std::to_string(resting) + "    [click] drop  [C] clear  [R] reset";
        DrawText(hud.c_str(), 12, 12, 18, to_rl(FxVec4ui8{226, 232, 240, 255}));
    });

    renderer.run(true);
    return 0;
}
