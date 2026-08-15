#pragma once

#include "Fx2D/Physics.h"

#include <algorithm>
#include <cmath>
#include <memory>

// Mouse-driven slingshot: grab the ball, pull away from the anchor, release to launch.
//
// Kept in a header rather than inside main.cpp so the mechanic can be exercised headlessly by
// tests/test_slingshot.cpp, which injects mouse state instead of a real cursor. That is the
// same producer API the renderer uses, so the tested path is the played path.
struct FxSlingshot {
    // --- configuration -------------------------------------------------------------
    FxVec2f anchor{3.0f, 3.0f}; // where the ball rests before launch, in scene units
    float max_pull = 2.2f; // furthest the ball can be drawn back
    float grab_radius = 0.9f; // how close the cursor must be to pick the ball up
    float power = 9.0f; // launch speed per unit of pull

    // --- state ---------------------------------------------------------------------
    bool dragging = false;
    bool launched = false;

    // Parks the ball at the anchor and arms the slingshot again.
    void reset(const std::shared_ptr<FxEntity>& ball) {
        dragging = false;
        launched = false;
        if (!ball) return;
        hold_at(ball, anchor);
    }

    // Where the band should be drawn from and to. Only meaningful before launch.
    FxVec2f band_start() const { return anchor; }
    FxVec2f band_end(const std::shared_ptr<FxEntity>& ball) const {
        return ball ? ball->pose.xy() : anchor;
    }

    // How hard a release right now would throw the ball.
    FxVec2f launch_velocity(const std::shared_ptr<FxEntity>& ball) const {
        if (!ball) return {0.0f, 0.0f};
        return (anchor - ball->pose.xy()) * power;
    }

    // Call once per step, from the scene's step callback.
    void update(FxScene& scene, const std::shared_ptr<FxEntity>& ball) {
        if (!ball) return;
        const FxInput& in = scene.input();

        if (launched) {
            if (in.key_pressed(FxKey::R)) reset(ball);
            return;
        }

        if (!dragging) {
            // Hold the ball at the anchor so it does not fall while waiting to be fired.
            hold_at(ball, anchor);
            const FxVec2f cursor = in.mouse_position();
            if (in.mouse_pressed(FxMouseButton::Left) && (cursor - anchor).norm() <= grab_radius) {
                dragging = true;
            }
            if (in.key_pressed(FxKey::R)) reset(ball);
            return;
        }

        // Dragging: follow the cursor, but never further than max_pull from the anchor.
        FxVec2f pull = in.mouse_position() - anchor;
        const float distance = pull.norm();
        if (distance > max_pull) pull = pull * (max_pull / distance);
        hold_at(ball, anchor + pull);

        if (in.mouse_released(FxMouseButton::Left)) {
            const FxVec2f velocity = launch_velocity(ball);
            ball->gravity_scale = 1.0f;
            ball->velocity = FxVec3f{velocity.x(), velocity.y(), 0.0f};
            ball->wake();
            dragging = false;
            launched = true;
        }
    }

  private:
    // Park the ball at a position with no residual motion.
    //
    // prev_pose is written alongside pose deliberately. FxScene::step() derives velocity as
    // (pose - prev_pose) / dt, so moving pose alone would have the solver reconstruct a huge
    // velocity from what is really a teleport, and the ball would fire itself the moment it
    // was picked up.
    static void hold_at(const std::shared_ptr<FxEntity>& ball, const FxVec2f& position) {
        ball->gravity_scale = 0.0f;
        ball->pose.x() = position.x();
        ball->pose.y() = position.y();
        ball->prev_pose = ball->pose;
        ball->velocity = FxVec3f{0.0f, 0.0f, 0.0f};
        ball->wake();
    }
};
