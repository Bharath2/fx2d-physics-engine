---
title: Your first scene
---

# Your first scene

A scene describes simulation settings and named entities. Save the following as `Scene.yml` in the repository root.

```yaml
scene:
  size: [16, 9]
  gravity: [0, -9.81]

entities:
  floor:
    pose: [8, 0.5, 0]
    physics:
      mass: 0
      gravity_scale: 0
    collision:
      geometry:
        rectangle: [16, 1]

  ball:
    pose: [8, 7, 0]
    physics:
      mass: 1.0
      elasticity: 0.5
    collision:
      geometry:
        circle: 0.25
```

Load it with the standard renderer entry point:

```cpp
#include "Fx2D/Core.h"

int main() {
    auto scene = FxYAML::buildScene("Scene.yml");
    FxRylbRenderer renderer(scene, 60);
    renderer.run();
}
```

That is the entire loop: Fx2D advances the scene with a fixed timestep while the renderer presents it. The [Scene YAML reference](/reference/scene-yaml) documents every field, shape, and joint.
