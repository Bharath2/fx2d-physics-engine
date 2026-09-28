// The playground: drag anything with the left mouse button. Right-click or Space spawns a shape
// at the cursor, C clears what was spawned, R resets. The same file builds for the browser
// (scripts/build_web.sh) and for the desktop (-DFX2D_BUILD_EXAMPLES=ON, run from the repo root).

#include "Fx2D/Core.h"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

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

// ---------------------------------------------------------------- rope and bridge builders

// One link body: a small circle for a rope, a plank for a bridge deck.
std::shared_ptr<FxEntity> make_link(FxScene& scene, const std::shared_ptr<FxEntityGroup>& group,
                                    const std::string& name, const FxVec3f& pose,
                                    const FxShape& shape, float mass, const FxVec4ui8& colour,
                                    float damping = 0.0f) {
    auto e = std::make_shared<FxEntity>(name);
    FxVisualShape visual(shape);
    visual.set_fillColor(colour);
    visual.set_outlineColor(FxVec4ui8{20, 24, 34, 255});
    visual.set_outlineThickness(2.0f);
    e->set_visual_geometry(visual);
    e->set_collision_geometry(FxCollisionShape(shape));
    e->set_init_pose(pose);
    e->set_mass(mass);
    e->set_inertia();
    e->static_friction = 0.6f;
    e->dynamic_friction = 0.5f;
    // A little drag, so a knocked bridge or a swung ball settles instead of ringing forever.
    e->vel_damping = damping;
    // Rope and bridge links swing through one another constantly; a group keeps them from
    // colliding without excluding them from anything else.
    scene.add_to_group(group, e);
    e->reset();
    return e;
}

// Joins two bodies at a point on each, at whatever distance the scene was authored with.
// Compliance is metres of stretch per newton of tension: zero is a rigid link.
void link_together(FxScene& scene, const std::string& name, const std::shared_ptr<FxEntity>& a,
                   const std::shared_ptr<FxEntity>& b, const FxVec2f& anchor_a,
                   const FxVec2f& anchor_b, double compliance = 0.0) {
    auto joint = std::make_shared<FxDistanceJoint>(name, a, b, anchor_a, anchor_b);
    if (compliance > 0.0) joint->set_compliance(compliance);
    scene.add_joint(joint);
}

// A rope of round links hanging from `from`, with a heavy ball on the end. Drag the ball and it
// swings like one, because every link is a distance joint and nothing locks the angles.
void build_wrecking_ball(FxScene& scene, const std::shared_ptr<FxEntity>& from) {
    constexpr int kLinks = 12;
    constexpr float kPitch = 0.32f;
    constexpr float kLinkRadius = 0.09f;

    auto group = scene.create_group("wrecking_ball");
    const FxVec2f top = from->pose.xy() - FxVec2f{0.0f, 0.12f};

    auto previous = from;
    FxVec2f previous_anchor{0.0f, -0.12f};
    for (int i = 0; i < kLinks; ++i) {
        const float y = top.y() - kPitch * static_cast<float>(i + 1);
        auto link = make_link(scene, group, "rope_" + std::to_string(i), FxVec3f{top.x(), y, 0.0f},
                              FxShape(kLinkRadius), 0.3f, FxVec4ui8{150, 158, 176, 255}, 0.2f);
        link_together(scene, "rope_j" + std::to_string(i), previous, link, previous_anchor,
                      FxVec2f{0.0f, 0.0f});
        previous = link;
        previous_anchor = FxVec2f{0.0f, 0.0f};
    }

    auto ball = make_link(scene, group, "wrecking_ball",
                          FxVec3f{top.x(), top.y() - kPitch * (kLinks + 1), 0.0f}, FxShape(0.32f),
                          5.0f, FxVec4ui8{226, 96, 84, 255}, 0.2f);
    link_together(scene, "rope_ball", previous, ball, FxVec2f{0.0f, 0.0f}, FxVec2f{0.0f, 0.0f});
}

// A plank bridge slung between two towers. The planks are laid along a shallow sag, so the deck
// has slack to deepen when something lands on it rather than being a rigid beam.
void build_bridge(FxScene& scene, const std::shared_ptr<FxEntity>& left,
                  const std::shared_ptr<FxEntity>& right) {
    constexpr int kPlanks = 10;
    constexpr float kSag = 0.55f;
    constexpr float kDeckY = 6.9f;
    // Ropes stretch. A rigid deck laid along an arc can only deepen that arc by the slack it
    // was drawn with, which is a few centimetres; giving each link some give is what lets the
    // span dip under a load and spring back when it comes off.
    constexpr double kLinkCompliance = 5e-4;

    const float x0 = left->pose.x() + 0.15f; // inner face of each tower
    const float x1 = right->pose.x() - 0.15f;
    const float pitch = (x1 - x0) / static_cast<float>(kPlanks);
    const float width = pitch * 0.86f;

    auto group = scene.create_group("bridge");
    scene.add_to_group(group, left);
    scene.add_to_group(group, right);

    auto previous = left;
    FxVec2f previous_anchor{0.15f, 0.6f}; // top inner corner of the left tower
    for (int i = 0; i < kPlanks; ++i) {
        const float t = (static_cast<float>(i) + 0.5f) / static_cast<float>(kPlanks);
        const float x = x0 + (x1 - x0) * t;
        const float y = kDeckY - kSag * std::sin(FxPif * t);
        // Lie along the sag rather than across it, so the deck reads as one curve.
        const float slope = -kSag * FxPif * std::cos(FxPif * t) / (x1 - x0);
        auto plank =
            make_link(scene, group, "plank_" + std::to_string(i), FxVec3f{x, y, std::atan(slope)},
                      FxShape(FxVec2f{width, 0.12f}), 0.25f, FxVec4ui8{196, 150, 96, 255}, 0.6f);
        link_together(scene, "plank_j" + std::to_string(i), previous, plank, previous_anchor,
                      FxVec2f{-width * 0.5f, 0.0f}, kLinkCompliance);
        previous = plank;
        previous_anchor = FxVec2f{width * 0.5f, 0.0f};
    }
    link_together(scene, "plank_jend", previous, right, previous_anchor, FxVec2f{-0.15f, 0.6f},
                  kLinkCompliance);
}

} // namespace

int main(int, char**) {
    FxScene scene = FxYAML::buildScene("examples/playground/Scene.yml");

    if (auto gantry = scene.get_entity("gantry")) build_wrecking_ball(scene, gantry);
    if (auto left = scene.get_entity("tower_left")) {
        if (auto right = scene.get_entity("tower_right")) build_bridge(scene, left, right);
    }

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
                                    "[drag] swing the wrecking ball, sag the bridge, move "
                                    "anything   [right-click / space] spawn   [C] clear   "
                                    "[R] reset";
        DrawText(hud.c_str(), 12, 12, 18, to_rl_color(FxVec4ui8{226, 232, 240, 255}));
    });

    renderer.run(true);
    return 0;
}
