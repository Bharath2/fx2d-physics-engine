# Keyboard and Mouse Input

`FxScene::input()` exposes keyboard and mouse state to gameplay code. The interface is
renderer-agnostic — `Fx2D/Input.h` includes no raylib — so the same code compiles and runs in
both windowed and headless builds.

Who fills it in differs:

| Build | Producer | `available()` |
|---|---|---|
| Windowed (`FxRylbRenderer`) | the renderer polls raylib once per rendered frame | `true` |
| Headless | nobody, unless you inject | `false` until you do |

A headless scene has no window, so it has no keyboard or mouse. Every query reads `false` or
zero. That is not an error — it is the honest answer, and `available()` lets shared code tell
the difference between "no key is held" and "there is no keyboard here". Headless scenes drive
themselves by *injecting* input instead, which is covered below.

## Reading input

Read it from the step callback, which runs once per `step()`:

```cpp
scene.set_step_callback([](FxScene& scene, double dt) {
    auto cart = scene.get_entity("cart");
    if (!cart) return;

    // Continuous control: act while the key is held.
    if (scene.input().key_down(FxKey::Right)) cart->apply_force({40.0f, 0.0f});
    if (scene.input().key_down(FxKey::Left))  cart->apply_force({-40.0f, 0.0f});

    // One-shot: fires on the frame the key goes down, not while it is held.
    if (scene.input().key_pressed(FxKey::Space)) cart->apply_impulse({0.0f, 8.0f});
});
```

### Queries

```cpp
bool  key_down(FxKey);              // held right now
bool  key_pressed(FxKey);           // went down this frame
bool  key_released(FxKey);          // came up this frame

bool  mouse_down(FxMouseButton);    // Left, Right, Middle
bool  mouse_pressed(FxMouseButton);
bool  mouse_released(FxMouseButton);

FxVec2f mouse_position();           // scene (world) units — compare with entity poses
FxVec2f mouse_screen_position();    // window pixels, origin top-left
FxVec2f mouse_delta();              // world-unit movement since the previous frame
float   wheel_delta();              // this frame's scroll; positive is away from the user

bool  available();                  // has anything fed this input?
```

`mouse_position()` is in **scene units**, the same frame entity poses live in, so picking works
without any conversion:

```cpp
const FxVec2f cursor = scene.input().mouse_position();
if (scene.input().mouse_pressed(FxMouseButton::Left)) {
    auto ball = scene.get_entity("ball");
    const FxVec2f toward = cursor - ball->pose.xy();
    ball->apply_impulse(toward * 2.0f);
}
```

`FxRylbRenderer` also exposes `screen_to_world()` and `world_to_screen()` if you need the
conversion elsewhere.

### Edge events last a whole frame

Input is polled once per **rendered frame**, but the renderer may run several physics **steps**
per frame to consume its accumulator. `key_pressed()` stays true for every step in that frame.
For anything that must not be applied twice — spawning, firing, jumping — either use
`key_down()` and track your own state, or latch the event yourself:

```cpp
bool jump_used = false;
scene.set_step_callback([&](FxScene& scene, double) {
    if (scene.input().key_pressed(FxKey::Space) && !jump_used) {
        jump_used = true;
        scene.get_entity("player")->apply_impulse({0.0f, 8.0f});
    }
    if (scene.input().key_released(FxKey::Space)) jump_used = false;
});
```

Continuous control via `key_down()` has no such hazard: applying a force on each of several
steps within one frame is correct, since each step integrates its own slice of time.

### The UI gets first refusal

While an ImGui panel has keyboard focus, or the cursor is over one, the renderer reports every
key and button as released. Typing a number into a control panel field will not also drive the
scene. The mouse *position* is still reported, since knowing where the cursor is remains useful.

## Headless: injecting input

A headless scene drives itself by writing the same state the renderer would. This is the
event-trigger path — scripted demos, replays, and RL agents all use it.

Call `begin_frame()` once per frame, then set the state for that frame:

```cpp
for (int frame = 0; frame < 600; ++frame) {
    scene.input().begin_frame();                     // rolls state so edges work
    scene.input().set_key(FxKey::Right, frame > 100); // held from frame 100
    if (frame == 200) scene.input().set_mouse_button(FxMouseButton::Left, true);
    scene.step(1.0 / 60.0);
}
```

`begin_frame()` is what makes `key_pressed()` and `key_released()` meaningful: it moves the
current state into the previous one, so the edges describe the frame you are about to specify.
It also resets `wheel_delta()`, which is a per-frame impulse rather than a state, and marks the
input available.

The producer API in full:

```cpp
void begin_frame();                                        // call first, once per frame
void set_key(FxKey, bool down);
void set_mouse_button(FxMouseButton, bool down);
void set_mouse_position(const FxVec2f& world, const FxVec2f& screen);
void set_wheel_delta(float);
void release_all();   // drop held keys/buttons, keep availability
void clear();         // full reset, including availability
```

`FxScene::reset()` calls `clear()`, so a replayed scene does not inherit stale key state.

An agent stepping the scene directly uses exactly the same shape — set the action as key state,
step, read [contacts and events](./events) for the reward.

## Keys

`FxKey` covers `A`–`Z`, `Num0`–`Num9`, the four arrows, `Space`, `Enter`, `Escape`, `Tab`,
`Backspace`, `Delete`, the left/right `Shift`, `Ctrl` and `Alt` modifiers, and `F1`–`F12`.
`FxMouseButton` covers `Left`, `Right` and `Middle`. Unmapped values fold onto `FxKey::Unknown`
rather than indexing out of bounds.

To add a key, extend the `FxKey` enum in `include/Fx2D/Input.h` and add the matching row to
`kKeyBindings` in `src/Renderer.cpp`. Nothing else needs to change.
