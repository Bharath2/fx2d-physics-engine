// Input state machine: edge detection, mouse state, and headless injection. The same interface
// exists with or without a window; both paths go through the producer API exercised here.

#include "Fx2D/Physics.h"

#include "test_harness.h"
#include "test_scene_builders.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>

namespace {

constexpr double kFrame = 1.0 / 60.0;

// A scene with no producer feeding it must report no input at all.
void test_unfed_input_is_inert() {
    FxInput input;

    require(!input.available(), "input nobody has fed must report itself unavailable");
    require(!input.key_down(FxKey::Space), "no key may read as down");
    require(!input.key_pressed(FxKey::Space), "no key may read as pressed");
    require(!input.key_released(FxKey::Space), "no key may read as released");
    require(!input.mouse_down(FxMouseButton::Left), "no mouse button may read as down");
    require(input.wheel_delta() == 0.0f, "wheel delta must start at zero");
    require(input.mouse_position().x() == 0.0f && input.mouse_position().y() == 0.0f,
            "mouse position must start at the origin");
}

// A key held across frames reports pressed once, down throughout, released once.
void test_key_edges_fire_once() {
    FxInput input;

    // Frame 1: key goes down.
    input.begin_frame();
    input.set_key(FxKey::Right, true);
    require(input.available(), "feeding input must mark it available");
    require(input.key_down(FxKey::Right), "a key set down must read as down");
    require(input.key_pressed(FxKey::Right), "the first frame down must report pressed");
    require(!input.key_released(FxKey::Right), "a key going down must not report released");

    // Frame 2: still held. Down stays true, the edge does not repeat.
    input.begin_frame();
    input.set_key(FxKey::Right, true);
    require(input.key_down(FxKey::Right), "a held key must stay down");
    require(!input.key_pressed(FxKey::Right), "pressed must not repeat while a key is held");

    // Frame 3: released.
    input.begin_frame();
    input.set_key(FxKey::Right, false);
    require(!input.key_down(FxKey::Right), "a released key must not read as down");
    require(input.key_released(FxKey::Right), "the frame a key comes up must report released");
    require(!input.key_pressed(FxKey::Right), "a key coming up must not report pressed");

    // Frame 4: still up, no edge.
    input.begin_frame();
    input.set_key(FxKey::Right, false);
    require(!input.key_released(FxKey::Right), "released must not repeat once the key is up");
}

// Keys are independent of one another.
void test_keys_are_independent() {
    FxInput input;
    input.begin_frame();
    input.set_key(FxKey::A, true);
    input.set_key(FxKey::W, true);

    require(input.key_down(FxKey::A), "A must be down");
    require(input.key_down(FxKey::W), "W must be down");
    require(!input.key_down(FxKey::D), "D must not be down");
    require(!input.key_down(FxKey::Space), "Space must not be down");
}

void test_mouse_button_edges() {
    FxInput input;

    input.begin_frame();
    input.set_mouse_button(FxMouseButton::Left, true);
    require(input.mouse_down(FxMouseButton::Left), "left button must read as down");
    require(input.mouse_pressed(FxMouseButton::Left), "first frame down must report pressed");
    require(!input.mouse_down(FxMouseButton::Right), "right button must be untouched");

    input.begin_frame();
    input.set_mouse_button(FxMouseButton::Left, true);
    require(!input.mouse_pressed(FxMouseButton::Left), "pressed must not repeat while held");

    input.begin_frame();
    input.set_mouse_button(FxMouseButton::Left, false);
    require(input.mouse_released(FxMouseButton::Left), "releasing must report released once");
}

// Mouse delta is measured in world units between frames; wheel delta is per frame.
void test_mouse_motion_and_wheel() {
    FxInput input;

    input.begin_frame();
    input.set_mouse_position(FxVec2f{3.0f, 4.0f}, FxVec2f{300.0f, 400.0f});
    input.set_wheel_delta(2.0f);
    require_near(input.mouse_position().x(), 3.0f, 1e-5f, "world mouse x");
    require_near(input.mouse_screen_position().y(), 400.0f, 1e-5f, "screen mouse y");
    require_near(input.wheel_delta(), 2.0f, 1e-5f, "wheel delta must survive within the frame");

    input.begin_frame();
    input.set_mouse_position(FxVec2f{5.0f, 4.5f}, FxVec2f{500.0f, 350.0f});
    require_near(input.mouse_delta().x(), 2.0f, 1e-5f, "world mouse dx across a frame");
    require_near(input.mouse_delta().y(), 0.5f, 1e-5f, "world mouse dy across a frame");
    require(input.wheel_delta() == 0.0f,
            "wheel delta must reset each frame, it is a per-frame impulse not a state");
}

// release_all drops held keys without pretending input went away; clear resets everything.
void test_release_all_and_clear() {
    FxInput input;
    input.begin_frame();
    input.set_key(FxKey::Space, true);
    input.set_mouse_button(FxMouseButton::Left, true);

    input.release_all();
    require(!input.key_down(FxKey::Space), "release_all must drop held keys");
    require(!input.mouse_down(FxMouseButton::Left), "release_all must drop held buttons");
    require(input.available(), "release_all must not make input look unavailable");

    input.clear();
    require(!input.available(), "clear must reset availability");
}

// Out-of-range enum values must not index out of bounds.
void test_out_of_range_key_is_safe() {
    FxInput input;
    input.begin_frame();
    const FxKey bogus = static_cast<FxKey>(250);
    input.set_key(bogus, true);
    require(input.key_down(bogus), "an out-of-range key must fold onto a valid slot, not crash");
    require(!input.key_down(FxKey::A), "folding must not disturb real keys");
}

// A scene exposes input, and a headless one starts with none.
void test_scene_starts_without_input() {
    FxScene scene({10.0f, 10.0f});
    require(!scene.input().available(),
            "a headless scene must report no input until something feeds it");
}

// Injected input reaches the step callback: this is the headless trigger path.
void test_headless_injection_drives_step_callback() {
    FxScene scene({20.0f, 20.0f});
    scene.set_gravity(FxVec2f{0.0f, 0.0f});

    auto body = add_box(scene, "body", {10.0f, 10.0f}, {1.0f, 1.0f});

    int callback_saw_key = 0;
    scene.set_step_callback([&](FxScene& s, double) {
        if (s.input().key_down(FxKey::Right)) {
            ++callback_saw_key;
            s.get_entity("body")->velocity.x() = 2.0f;
        }
    });

    // No input yet: the body must not move.
    for (int i = 0; i < 10; ++i)
        scene.step(kFrame);
    require(callback_saw_key == 0, "the callback must not see a key nobody pressed");

    // Inject a held key, as a headless trigger would.
    const float x_before = body->pose.x();
    for (int i = 0; i < 30; ++i) {
        scene.input().begin_frame();
        scene.input().set_key(FxKey::Right, true);
        scene.step(kFrame);
    }

    require(callback_saw_key == 30, "the callback must see the injected key on every step, saw " +
                                        std::to_string(callback_saw_key));
    require(body->pose.x() > x_before + 0.5f,
            "the injected key must actually drive the body, x moved from " +
                std::to_string(x_before) + " to " + std::to_string(body->pose.x()));
}

// A one-shot trigger fires once even while the key stays held.
void test_headless_edge_trigger_fires_once() {
    FxScene scene({20.0f, 20.0f});
    scene.set_gravity(FxVec2f{0.0f, 0.0f});

    int fired = 0;
    scene.set_step_callback([&](FxScene& s, double) {
        if (s.input().key_pressed(FxKey::Space)) ++fired;
    });

    for (int i = 0; i < 20; ++i) {
        scene.input().begin_frame();
        scene.input().set_key(FxKey::Space, true); // held the whole time
        scene.step(kFrame);
    }

    require(fired == 1, "an edge trigger must fire once for a held key, fired " +
                            std::to_string(fired) + " times");
}

// reset() clears input along with the rest of the scene state.
void test_scene_reset_clears_input() {
    FxScene scene({10.0f, 10.0f});
    scene.input().begin_frame();
    scene.input().set_key(FxKey::A, true);
    require(scene.input().available(), "input must be available after being fed");

    scene.reset();
    require(!scene.input().available(), "reset must clear input availability");
    require(!scene.input().key_down(FxKey::A), "reset must drop held keys");
}

// reset() must give user code a chance to restore state the scene does not own.
void test_reset_callback_fires_after_entities_are_restored() {
    FxScene scene({20.0f, 20.0f});

    auto body = add_box(scene, "body", {5.0f, 5.0f}, {1.0f, 1.0f});

    int fired = 0;
    float body_x_seen_by_callback = -1.0f;
    scene.set_reset_callback([&](FxScene& s) {
        ++fired;
        // The callback must run after entities are back, so it can build on a clean scene.
        body_x_seen_by_callback = s.get_entity("body")->pose.x();
    });

    require(fired == 0, "the reset callback must not fire before reset() is called");

    // Move the body away, then reset.
    body->pose.x() = 17.0f;
    scene.reset();

    require(fired == 1, "reset() must fire the reset callback exactly once");
    require_near(body_x_seen_by_callback, 5.0f, 1e-4f,
                 "the callback must see entities already restored to their initial pose");

    scene.reset();
    require(fired == 2, "each reset() must fire the callback again");
}

// reset() restores the captured composition: additions go, deletions come back.
void test_reset_restores_scene_composition() {
    FxScene scene({20.0f, 20.0f});
    scene.set_gravity(FxVec2f{0.0f, 0.0f});

    auto keeper = add_box(scene, "keeper", {5.0f, 5.0f}, {1.0f, 1.0f});

    auto doomed = add_box(scene, "doomed", {9.0f, 5.0f}, {1.0f, 1.0f});

    scene.step(kFrame); // captures the initial composition
    require(scene.entity_count() == 2, "the scene must start with both entities");

    // Mutate the scene the way a running game would.
    scene.delete_entity("doomed");
    auto spawned = add_box(scene, "spawned", {12.0f, 5.0f}, {1.0f, 1.0f});
    scene.step(kFrame);

    require(scene.entity_count() == 2, "after the edit there should still be two entities");
    require(!scene.entity_exists("doomed"), "doomed must be gone before the reset");
    require(scene.entity_exists("spawned"), "spawned must be present before the reset");

    scene.reset();

    require(scene.entity_count() == 2, "reset must restore the original entity count");
    require(scene.entity_exists("keeper"), "the original entity must survive the reset");
    require(scene.entity_exists("doomed"), "a deleted entity must come back on reset");
    require(!scene.entity_exists("spawned"), "an entity added at runtime must be gone on reset");
}

// A sleeping body must wake on reset, or the reloaded scene starts frozen.
void test_reset_wakes_sleeping_bodies() {
    FxScene scene({20.0f, 20.0f});
    scene.set_gravity(FxVec2f{0.0f, -10.0f});

    make_static(add_box(scene, "ground", {10.0f, 1.0f}, {18.0f, 1.0f}));

    auto box = add_box(scene, "box", {10.0f, 2.0f}, {1.0f, 1.0f}, {.elasticity = 0.0f});

    for (int i = 0; i < 240 && !box->is_sleeping(); ++i)
        scene.step(kFrame);
    require(box->is_sleeping(), "the box must fall asleep before the reset is meaningful");

    scene.reset();
    require(!box->is_sleeping(), "reset must wake every body, or the scene reloads frozen");
}

// Time, contacts and input are all dropped by a reset.
void test_reset_clears_accumulated_scene_state() {
    FxScene scene({20.0f, 20.0f});
    scene.set_gravity(FxVec2f{0.0f, -10.0f});

    make_static(add_box(scene, "ground", {10.0f, 1.0f}, {18.0f, 1.0f}));

    auto box = add_box(scene, "box", {10.0f, 2.0f}, {1.0f, 1.0f});

    for (int i = 0; i < 60; ++i)
        scene.step(kFrame);
    require(scene.time_elapsed() > 0.5, "time must have advanced before the reset");
    require(!scene.contacts().empty(), "the box must be resting on the ground before the reset");

    scene.reset();

    require(scene.time_elapsed() == 0.0, "reset must put the clock back to zero");
    require(scene.contacts().empty(), "reset must drop the contact buffer");
    require(scene.begin_contact_events().empty(), "reset must drop begin events");
    require(!scene.input().available(), "reset must drop input");
    require_near(box->pose.y(), 2.0f, 1e-4f, "reset must put the box back at its initial pose");
}

} // namespace

void run_input_tests() {
    test_reset_callback_fires_after_entities_are_restored();
    test_reset_restores_scene_composition();
    test_reset_wakes_sleeping_bodies();
    test_reset_clears_accumulated_scene_state();
    test_unfed_input_is_inert();
    test_key_edges_fire_once();
    test_keys_are_independent();
    test_mouse_button_edges();
    test_mouse_motion_and_wheel();
    test_release_all_and_clear();
    test_out_of_range_key_is_safe();
    test_scene_starts_without_input();
    test_headless_injection_drives_step_callback();
    test_headless_edge_trigger_fires_once();
    test_scene_reset_clears_input();
    std::cout << "Input tests passed." << std::endl;
}
