#pragma once

#include <memory>
#include <vector>

#include "Fx2D/Math.h"

class FxEntity;

// What a ray struck.
struct FxRayHit {
    std::shared_ptr<FxEntity> entity = nullptr;
    FxVec2f point{0.0f, 0.0f}; // world-space point of impact
    FxVec2f normal{0.0f, 0.0f}; // outward surface normal there, facing back along the ray
    float distance = 0.0f; // travel along the ray from its origin

    bool hit() const { return entity != nullptr; }
};
