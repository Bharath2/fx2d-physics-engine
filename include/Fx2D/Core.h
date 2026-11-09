#pragma once

// Everything, renderer included. Headless code should include Fx2D/Scene.h instead, which
// pulls in no windowing dependency.

#include "Fx2D/Renderer.h"
#include "Fx2D/Scene.h"
#ifndef FX2D_NO_YAML
#include "Fx2D/YamlUtils.h"
#endif
