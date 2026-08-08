#pragma once

// Physics-only aggregate header: everything needed to build and step a scene,
// with no raylib / Dear ImGui / rlImGui dependency. Include this for headless
// builds (testing, data collection, batch simulation, CI without a GL stack).
//
// Include "Fx2D/Core.h" instead when the renderer is also needed.

#include "Fx2D/Entity.h"
#include "Fx2D/Joints.h"
#include "Fx2D/Math.h"
#include "Fx2D/Registry.h"
#include "Fx2D/Scene.h"
#include "Fx2D/Solver.h"
#include "Fx2D/YamlUtils.h"
