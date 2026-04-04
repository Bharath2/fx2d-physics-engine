# Raylib Renderer — `FxRylbRenderer`

The `FxRylbRenderer` class drives the real-time render loop using [raylib](https://www.raylib.com/) and [Dear ImGui](https://github.com/ocornut/imgui) (via rlImGui). Include via:

```cpp
#include "Fx2D/Core.h"
```

---

## Construction

```cpp
FxRylbRenderer(FxScene& scene, int fps = 60, unsigned int scale = 100);
```

| Parameter | Description |
|---|---|
| `scene` | Reference to a `FxScene` to render and simulate |
| `fps` | Target frames per second (default: 60) |
| `scale` | Pixels per physics unit (default: 100) |

```cpp
auto scene = FxYAML::buildScene("./Scene.yml");
FxRylbRenderer renderer(scene, 60, 100);
```

---

## Running the Simulation

```cpp
void run(bool play = true);
```

Starts the window and enters the render loop. Returns when the window is closed.

- If `play = false`, the simulation starts paused — useful for inspecting the initial state.

```cpp
renderer.run();         // start playing immediately
renderer.run(false);    // start paused
```

---

## Background

```cpp
void set_background(const FxVec4ui8& color);       // solid RGBA color
void set_background(const std::string& filepath);  // image/texture file
```

```cpp
renderer.set_background(FxVec4ui8(30, 30, 30, 255));   // dark grey
renderer.set_background("./assets/background.png");
```

---

## Real-Time Factor

Controls how fast the simulation runs relative to wall-clock time.

```cpp
void   set_real_time_factor(const double& rt_factor);
double get_real_time_factor() const;
```

- `1.0` — real time (default)
- `0.5` — half speed (slow motion)
- `2.0` — double speed

The value is clamped to `[0.01, 10.0]`; values outside this range are rejected with a warning and clamped.

```cpp
renderer.set_real_time_factor(0.5);
```

---

## ImGui Panel

While running, an ImGui overlay is displayed with:
- Play / Pause toggle
- Real-time factor control
- Simulation time

---

## Fixed Timestep Integration

The renderer uses a **fixed-timestep accumulator** pattern:

1. Wall-clock elapsed time × real-time factor is accumulated each frame (raw frame time is capped at 0.2 s to prevent spiral-of-death).
2. Each drain step uses `dt = clamp(m_fixed_dt × real_time_factor, 1e-3, 0.06)` — so the effective step size scales with the real-time factor.
3. `scene.step(dt)` is called repeatedly until the accumulator is drained.

This decouples rendering frame rate from physics stability.

---

## Texture Caching

Entity textures are loaded lazily on first draw and cached in an internal `std::unordered_map<std::string, Texture2D>`. Repeated references to the same file path incur no extra GPU uploads.
