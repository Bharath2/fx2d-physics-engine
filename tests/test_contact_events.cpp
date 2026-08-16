// Buffered contacts, begin/end contact events, and sensor (trigger) entities.

#include "Fx2D/Scene.h"

#include "test_harness.h"
#include "test_scene_builders.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>

namespace {

constexpr double kFrame = 1.0 / 60.0;

std::shared_ptr<FxEntity> make_ground(FxScene& scene, const std::string& name, float cx, float cy,
                                      float w, float h) {
    return make_static(add_box(scene, name, {cx, cy}, {w, h}));
}

std::shared_ptr<FxEntity> make_box(FxScene& scene, const std::string& name, float cx, float cy,
                                   float w, float h) {
    return add_box(scene, name, {cx, cy}, {w, h});
}

FxScene make_scene() {
    return ::make_scene(FxVec2f{12.0f, 10.0f});
}

// Returns true if the two named entities appear as a pair in the event list.
bool has_pair(const std::vector<FxContactEvent>& events, const std::string& a,
              const std::string& b) {
    for (const auto& e : events) {
        if (!e.entity1 || !e.entity2) continue;
        const std::string& n1 = e.entity1->get_name();
        const std::string& n2 = e.entity2->get_name();
        if ((n1 == a && n2 == b) || (n1 == b && n2 == a)) return true;
    }
    return false;
}

bool contacts_include(const FxScene& scene, const std::string& a, const std::string& b) {
    for (const auto& c : scene.contacts()) {
        if (!c.entity1 || !c.entity2) continue;
        const std::string& n1 = c.entity1->get_name();
        const std::string& n2 = c.entity2->get_name();
        if ((n1 == a && n2 == b) || (n1 == b && n2 == a)) return true;
    }
    return false;
}

// A scene with nothing touching must expose no contacts and no events.
void test_no_contacts_when_separated() {
    FxScene scene = make_scene();
    make_ground(scene, "ground", 6.0f, 1.0f, 8.0f, 0.5f);
    make_box(scene, "box", 6.0f, 6.0f, 0.5f, 0.5f);

    scene.step(kFrame);

    require(scene.contacts().empty(), "a box in free fall must produce no contacts");
    require(scene.begin_contact_events().empty(), "free fall must produce no begin events");
    require(scene.end_contact_events().empty(), "free fall must produce no end events");
}

// Landing produces exactly one begin event, and it must not repeat while resting.
void test_begin_event_fires_once_on_landing() {
    FxScene scene = make_scene();
    make_ground(scene, "ground", 6.0f, 1.0f, 8.0f, 0.5f);
    make_box(scene, "box", 6.0f, 2.0f, 0.5f, 0.5f);

    int begin_count = 0;
    int resting_steps_with_contact = 0;
    for (int i = 0; i < 180; ++i) {
        scene.step(kFrame);
        if (has_pair(scene.begin_contact_events(), "box", "ground")) ++begin_count;
        if (contacts_include(scene, "box", "ground")) ++resting_steps_with_contact;
    }

    require(begin_count == 1, "a box landing and staying put must report begin exactly once, got " +
                                  std::to_string(begin_count));
    require(resting_steps_with_contact > 100,
            "a resting box must keep reporting its contact every step, got " +
                std::to_string(resting_steps_with_contact));
}

// Separating a resting pair produces an end event naming both entities.
void test_end_event_fires_on_separation() {
    FxScene scene = make_scene();
    make_ground(scene, "ground", 6.0f, 1.0f, 8.0f, 0.5f);
    auto box = make_box(scene, "box", 6.0f, 2.0f, 0.5f, 0.5f);

    for (int i = 0; i < 120; ++i) {
        scene.step(kFrame);
    }
    require(contacts_include(scene, "box", "ground"), "the box must be resting on the ground");

    // Teleport the box far away; the pair must report an end event.
    box->wake();
    box->pose = FxVec3f{6.0f, 8.0f, 0.0f};
    box->prev_pose = box->pose;
    box->velocity = FxVec3f{0.0f, 0.0f, 0.0f};

    bool saw_end = false;
    for (int i = 0; i < 5 && !saw_end; ++i) {
        scene.step(kFrame);
        if (has_pair(scene.end_contact_events(), "box", "ground")) saw_end = true;
    }

    require(saw_end, "separating a touching pair must report an end event");
}

// A solved contact carries valid geometry every step. The impulse is the velocity-pass one, so
// it decays to zero once settled: what holds the box up after that is the position solve.
void test_contact_carries_geometry_and_impact_impulse() {
    FxScene scene = make_scene();
    make_ground(scene, "ground", 6.0f, 1.0f, 8.0f, 0.5f);
    make_box(scene, "box", 6.0f, 2.0f, 0.5f, 0.5f);

    float max_normal_impulse = 0.0f;
    int steps_with_contact = 0;

    for (int i = 0; i < 120; ++i) {
        scene.step(kFrame);
        for (const auto& c : scene.contacts()) {
            require(c.entity1 != nullptr && c.entity2 != nullptr,
                    "a buffered contact must name both entities");
            require(c.count > 0, "a buffered contact must carry at least one contact point");
            require(c.normal.norm() > 0.9f, "a buffered contact must carry a unit normal");
            require(std::isfinite(c.penetration_depth),
                    "a buffered contact must carry a finite penetration depth");
            ++steps_with_contact;
            max_normal_impulse = std::max(max_normal_impulse, std::fabs(c.jn_accumulated[0]) +
                                                                  std::fabs(c.jn_accumulated[1]));
        }
    }

    require(steps_with_contact > 0, "a landing box must appear in the contact buffer");
    require(max_normal_impulse > 0.0f,
            "arresting the fall must report a non-zero normal impulse at impact");
}

// A sensor detects the overlap but must not deflect anything passing through it.
void test_sensor_does_not_apply_impulse() {
    FxScene scene = make_scene();
    auto sensor = make_box(scene, "trigger", 6.0f, 5.0f, 1.0f, 1.0f);
    sensor->is_sensor = true;
    sensor->set_mass(0.0f);
    sensor->set_inertia(0.0f);
    sensor->enable_external_forces(false);
    sensor->gravity_scale = 0.0f;

    auto faller = make_box(scene, "faller", 6.0f, 8.0f, 0.4f, 0.4f);

    bool saw_begin = false;
    bool saw_overlap = false;
    bool saw_end = false;
    float min_speed_during_overlap = 1e9f;

    for (int i = 0; i < 200; ++i) {
        scene.step(kFrame);
        if (has_pair(scene.begin_contact_events(), "faller", "trigger")) saw_begin = true;
        if (has_pair(scene.end_contact_events(), "faller", "trigger")) saw_end = true;
        if (contacts_include(scene, "faller", "trigger")) {
            saw_overlap = true;
            min_speed_during_overlap =
                std::min(min_speed_during_overlap, std::fabs(faller->velocity.y()));
        }
        if (faller->pose.y() < 1.0f) break;
    }

    require(saw_begin, "entering a sensor must report a begin event");
    require(saw_overlap, "a body inside a sensor must appear in the contact buffer");
    require(saw_end, "leaving a sensor must report an end event");
    require(faller->pose.y() < 4.0f, "a sensor must not block a falling body, it stopped at y=" +
                                         std::to_string(faller->pose.y()));
    require(min_speed_during_overlap > 0.5f,
            "a sensor must not slow a body passing through it, min speed was " +
                std::to_string(min_speed_during_overlap));
}

// Sensor contacts are reported as triggers and carry no impulse.
void test_sensor_contact_is_flagged_and_impulse_free() {
    FxScene scene = make_scene();
    auto sensor = make_box(scene, "trigger", 6.0f, 5.0f, 1.5f, 1.5f);
    sensor->is_sensor = true;
    sensor->set_mass(0.0f);
    sensor->set_inertia(0.0f);
    sensor->enable_external_forces(false);
    sensor->gravity_scale = 0.0f;

    make_box(scene, "faller", 6.0f, 7.0f, 0.4f, 0.4f);

    bool checked_event = false;
    bool checked_contact = false;
    for (int i = 0; i < 200; ++i) {
        scene.step(kFrame);

        for (const auto& e : scene.begin_contact_events()) {
            require(e.is_trigger, "a begin event involving a sensor must be flagged as a trigger");
            checked_event = true;
        }
        for (const auto& c : scene.contacts()) {
            float total = std::fabs(c.jn_accumulated[0]) + std::fabs(c.jn_accumulated[1]) +
                          std::fabs(c.jt_accumulated[0]) + std::fabs(c.jt_accumulated[1]);
            require(total == 0.0f, "a sensor contact must carry no impulse");
            checked_contact = true;
        }
        if (checked_contact) break;
    }

    require(checked_event, "the sensor overlap must have produced a begin event to inspect");
    require(checked_contact, "the sensor overlap must have produced a contact to inspect");
}

// A sensor must not wake a sleeping body, since it applies no force to disturb one.
void test_sensor_does_not_wake_sleeper() {
    FxScene scene = make_scene();
    make_ground(scene, "ground", 6.0f, 1.0f, 8.0f, 0.5f);
    auto box = make_box(scene, "box", 6.0f, 1.75f, 0.5f, 0.5f);

    for (int i = 0; i < 240 && !box->is_sleeping(); ++i) {
        scene.step(kFrame);
    }
    require(box->is_sleeping(), "the box must fall asleep before the sensor is introduced");

    // Drop a sensor directly over the sleeping box.
    auto sensor = make_box(scene, "trigger", 6.0f, 1.75f, 1.0f, 1.0f);
    sensor->is_sensor = true;
    sensor->set_mass(0.0f);
    sensor->set_inertia(0.0f);
    sensor->enable_external_forces(false);
    sensor->gravity_scale = 0.0f;

    for (int i = 0; i < 30; ++i) {
        scene.step(kFrame);
    }

    require(box->is_sleeping(), "a sensor overlap must not wake a sleeping body");
    require(contacts_include(scene, "box", "trigger"),
            "a sensor overlapping a sleeping body must still report the contact");
}

// Contacts and events must be readable from inside the step callback.
void test_events_visible_in_step_callback() {
    FxScene scene = make_scene();
    make_ground(scene, "ground", 6.0f, 1.0f, 8.0f, 0.5f);
    make_box(scene, "box", 6.0f, 2.0f, 0.5f, 0.5f);

    int begin_seen_in_callback = 0;
    scene.set_step_callback([&](FxScene& s, double) {
        if (has_pair(s.begin_contact_events(), "box", "ground")) ++begin_seen_in_callback;
    });

    for (int i = 0; i < 120; ++i) {
        scene.step(kFrame);
    }

    require(begin_seen_in_callback == 1,
            "the step callback must observe the landing begin event exactly once, got " +
                std::to_string(begin_seen_in_callback));
}

// reset() must clear buffered contacts so a replayed scene does not see stale events.
void test_reset_clears_contacts() {
    FxScene scene = make_scene();
    make_ground(scene, "ground", 6.0f, 1.0f, 8.0f, 0.5f);
    make_box(scene, "box", 6.0f, 2.0f, 0.5f, 0.5f);

    for (int i = 0; i < 120; ++i) {
        scene.step(kFrame);
    }
    require(!scene.contacts().empty(), "the box must be resting before reset");

    scene.reset();
    require(scene.contacts().empty(), "reset must clear the buffered contacts");
    require(scene.begin_contact_events().empty(), "reset must clear begin events");
    require(scene.end_contact_events().empty(), "reset must clear end events");
}

} // namespace

void run_contact_event_tests() {
    test_no_contacts_when_separated();
    test_begin_event_fires_once_on_landing();
    test_end_event_fires_on_separation();
    test_contact_carries_geometry_and_impact_impulse();
    test_sensor_does_not_apply_impulse();
    test_sensor_contact_is_flagged_and_impulse_free();
    test_sensor_does_not_wake_sleeper();
    test_events_visible_in_step_callback();
    test_reset_clears_contacts();
    std::cout << "Contact event and sensor tests passed." << std::endl;
}
