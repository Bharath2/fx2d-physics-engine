---
title: Your first scene
---

# Your first scene

A scene describes simulation settings and named entities. Save the following as `Scene.yml` in the repository root.

```yaml
scene:
  gravity: [0, 9.81]
  dt: 0.0166667

entities:
  floor:
    pose: [0, 500, 0]
    physics:
      static: true
    collision:
      type: rectangle
      size: [960, 40]

  ball:
    pose: [480, 80, 0]
    physics:
      density: 1.0
      restitution: 0.5
    collision:
      type: circle
      radius: 24
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
