// The playground: drag anything with the left mouse button. Right-click or Space spawns a shape
// at the cursor, C clears what was spawned, R resets. The same file builds for the browser
// (scripts/build_web.sh) and for the desktop (-DFX2D_BUILD_EXAMPLES=ON, run from the repo root).

#include "Fx2D/Core.h"

#include <memory>
#include <string>

namespace {

constexpr int kMaxSpawned = 60;

const FxVec4ui8 kColours[] = {{232, 122, 92, 255},
                              {110, 176, 232, 255},
                              {236, 200, 104, 255},
                              {156, 208, 132, 255},
                              {196, 140, 224, 255}};

// Cycles box, ball, capsule, rounded triangle, so a few clicks give a mixed pile.
std::shared_ptr<FxEntity> make_spawned(int index, const FxVec2f& at) {
    auto e = std::make_shared<FxEntity>("spawned_" + std::to_string(index));
    const FxVec4ui8 colour = kColours[index % 5];
    const float size = 0.35f + 0.1f * static_cast<float>(index % 3);

    FxShape shape;
    switch (index % 4) {
    case 0:
        shape = FxShape(FxVec2f{2.0f * size, 2.0f * size});
        break;
    case 1:
        shape = FxShape(size);
        break;
    case 2:
        shape = FxShape(2.2f * size, 0.5f * size);
        break;
    default: {
        FxVec2fArray tri(3);
        tri[0] = FxVec2f{0.0f, 1.2f * size};
        tri[1] = FxVec2f{-size, -0.7f * size};
        tri[2] = FxVec2f{size, -0.7f * size};
        shape = FxShape(tri, 0.08f);
        break;
    }
    }

    FxVisualShape visual(shape);
    visual.set_fillColor(colour);
    visual.set_outlineColor(FxVec4ui8{20, 24, 34, 255});
    visual.set_outlineThickness(2.0f);
    e->set_visual_geometry(visual);
    e->set_collision_geometry(FxCollisionShape(shape));
    e->set_init_pose(FxVec3f{at.x(), at.y(), 0.0f});
    e->set_mass(1.0f);
    e->set_inertia();
    e->elasticity = 0.2f;
    e->static_friction = 0.5f;
    e->dynamic_friction = 0.4f;
    return e;
}

} // namespace

int main(int, char**) {
    FxScene scene = FxYAML::buildScene("examples/playground/Scene.yml");

    int spawned = 0;
    scene.set_reset_callback([&](FxScene&) { spawned = 0; });

    scene.set_step_callback([&](FxScene& s, double) {
        const FxInput& in = s.input();

        const bool spawn = in.mouse_pressed(FxMouseButton::Right) || in.key_pressed(FxKey::Space);
        if (spawn && spawned < kMaxSpawned) {
            auto e = make_spawned(spawned, in.mouse_position());
            if (s.add_entity(e)) {
                e->reset();
                ++spawned;
            }
        }
        if (in.key_pressed(FxKey::R)) {
            s.reset();
            return;
        }
        if (in.key_pressed(FxKey::C)) {
            for (int i = 0; i < spawned; ++i)
                s.delete_entity("spawned_" + std::to_string(i));
            spawned = 0;
        }
    });

    FxRylbRenderer renderer(scene, 60, 60);

    renderer.set_draw_callback([&](FxRylbRenderer&) {
        const std::string hud = scene.mouse_joint().attached() ?
                                    "holding " + scene.mouse_joint().entity()->get_name() :
                                    "[drag] move anything   [right-click / space] spawn   "
                                    "[C] clear   [R] reset";
        DrawText(hud.c_str(), 12, 12, 18, to_rl_color(FxVec4ui8{226, 232, 240, 255}));
    });

    renderer.run(true);
    return 0;
}
