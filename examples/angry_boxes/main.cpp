// Angry-birds style demo: drag the ball back with the mouse, release, knock the tower down.
//
//   left mouse   grab the ball near the slingshot, drag to aim, release to fire
//   R            reset the shot
//
// Run from the repo root so the Scene.yml path resolves:
//   cmake -S . -B build -DFX2D_BUILD_EXAMPLES=ON && cmake --build build --target
//   example_angry_boxes
//   ./build/example_angry_boxes

#include "Fx2D/Core.h"

#include "slingshot.h"

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

namespace {

// Tower pieces, in the order they are stacked. Used only to report what got knocked over.
const char* kTowerPieces[] = {"box_l1", "box_r1",  "lintel1", "box_l2",
                              "box_r2", "lintel2", "target"};

Color to_color(const FxVec4ui8& c) {
    return Color{c[0], c[1], c[2], c[3]};
}

} // namespace

int main(int, char**) {
    FxScene scene = FxYAML::buildScene("examples/angry_boxes/Scene.yml");

    auto ball = scene.get_entity("ball");
    if (!ball) {
        std::fprintf(stderr, "angry_boxes: Scene.yml has no 'ball' entity\n");
        return 1;
    }

    FxSlingshot slingshot;
    slingshot.anchor = FxVec2f{3.0f, 3.0f};
    slingshot.reset(ball);

    // Resting heights, so a piece that has dropped or rolled can be counted as knocked over.
    std::vector<std::pair<std::string, float>> resting;
    for (const char* name : kTowerPieces) {
        if (auto piece = scene.get_entity(name)) resting.emplace_back(name, piece->pose.y());
    }

    int knocked_over = 0;
    float best_hit = 0.0f;

    scene.set_step_callback([&](FxScene& s, double) {
        slingshot.update(s, ball);

        // Score: a piece counts once it has fallen a body-height or tipped past ~35 degrees.
        int down = 0;
        for (const auto& [name, start_y] : resting) {
            auto piece = s.get_entity(name);
            if (!piece) continue;
            const bool fell = (start_y - piece->pose.y()) > 0.35f;
            const bool tipped = std::fabs(piece->pose.theta()) > 0.6f;
            if (fell || tipped) ++down;
        }
        knocked_over = down;

        // Impact strength: the largest normal impulse the ball delivered this step.
        for (const auto& contact : s.contacts()) {
            const bool ball_involved = (contact.entity1 && contact.entity1->get_name() == "ball") ||
                                       (contact.entity2 && contact.entity2->get_name() == "ball");
            if (!ball_involved) continue;
            best_hit = std::max(best_hit, std::fabs(contact.jn_accumulated[0]) +
                                              std::fabs(contact.jn_accumulated[1]));
        }
    });

    FxRylbRenderer renderer(scene, 60, 90);

    renderer.set_draw_callback([&](FxRylbRenderer& r) {
        // The slingshot post, and the band while aiming.
        const FxVec2f anchor_screen = r.world_to_screen(slingshot.anchor);
        DrawCircleV({anchor_screen.x(), anchor_screen.y()}, 5.0f, Color{90, 62, 34, 255});

        if (!slingshot.launched) {
            const FxVec2f ball_screen = r.world_to_screen(ball->pose.xy());
            DrawLineEx({anchor_screen.x(), anchor_screen.y()}, {ball_screen.x(), ball_screen.y()},
                       4.0f, Color{60, 40, 24, 255});

            // Dotted arc previewing where the shot would go, sampled from the launch velocity.
            const FxVec2f v = slingshot.launch_velocity(ball);
            const FxVec2f p0 = ball->pose.xy();
            for (int i = 1; i <= 26; ++i) {
                const float t = static_cast<float>(i) * 0.045f;
                const FxVec2f p{p0.x() + v.x() * t, p0.y() + v.y() * t - 0.5f * 10.0f * t * t};
                if (p.y() < 0.0f) break;
                const FxVec2f s = r.world_to_screen(p);
                DrawCircleV({s.x(), s.y()}, 2.0f, Color{255, 255, 255, 170});
            }
        }

        const std::string hud = "knocked over: " + std::to_string(knocked_over) + " / " +
                                std::to_string(resting.size()) +
                                "    hardest hit: " + std::to_string(static_cast<int>(best_hit)) +
                                (slingshot.launched ? "    [R] reset" : "    drag the ball");
        DrawText(hud.c_str(), 12, 12, 20, to_color(FxVec4ui8{30, 30, 40, 255}));
    });

    renderer.run(true);
    return 0;
}
