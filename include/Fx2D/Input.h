#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "Fx2D/Math.h"

// Renderer-agnostic input state.
//
// This header deliberately knows nothing about raylib, so headless builds compile against the
// same interface a windowed build uses. Gameplay code reads FxScene::input() from inside the
// step callback and never reaches into a windowing library directly.
//
// Who fills it in:
//   - Windowed: FxRylbRenderer polls the keyboard and mouse once per rendered frame.
//   - Headless: nothing does, so every query reads false/zero and available() is false. A
//     headless scene that wants scripted or event-driven control injects state itself through
//     the producer API below, which is the headless equivalent of a key press.

enum class FxKey : uint8_t {
    Unknown = 0,
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L,
    M,
    N,
    O,
    P,
    Q,
    R,
    S,
    T,
    U,
    V,
    W,
    X,
    Y,
    Z,
    Num0,
    Num1,
    Num2,
    Num3,
    Num4,
    Num5,
    Num6,
    Num7,
    Num8,
    Num9,
    Left,
    Right,
    Up,
    Down,
    Space,
    Enter,
    Escape,
    Tab,
    Backspace,
    Delete,
    LeftShift,
    RightShift,
    LeftCtrl,
    RightCtrl,
    LeftAlt,
    RightAlt,
    F1,
    F2,
    F3,
    F4,
    F5,
    F6,
    F7,
    F8,
    F9,
    F10,
    F11,
    F12,
    Count
};

enum class FxMouseButton : uint8_t { Left = 0, Right, Middle, Count };

class FxInput {
  public:
    static constexpr size_t kKeyCount = static_cast<size_t>(FxKey::Count);
    static constexpr size_t kButtonCount = static_cast<size_t>(FxMouseButton::Count);

    // ---------------------------------------------------------------- consumer API

    // Held down right now. This is what continuous control wants — driving a motor while a
    // key is held, for instance.
    bool key_down(FxKey key) const { return m_keys[index(key)]; }
    // Went down since the last frame. Edge events last for the whole frame, which may span
    // several physics steps when the renderer runs more than one step per frame; use
    // key_down() for anything that must not be applied twice.
    bool key_pressed(FxKey key) const { return m_keys[index(key)] && !m_prev_keys[index(key)]; }
    bool key_released(FxKey key) const { return !m_keys[index(key)] && m_prev_keys[index(key)]; }

    bool mouse_down(FxMouseButton button) const { return m_buttons[index(button)]; }
    bool mouse_pressed(FxMouseButton button) const {
        return m_buttons[index(button)] && !m_prev_buttons[index(button)];
    }
    bool mouse_released(FxMouseButton button) const {
        return !m_buttons[index(button)] && m_prev_buttons[index(button)];
    }

    // Cursor in scene (world) units — the frame entity poses live in, so this can be compared
    // against entity positions directly.
    const FxVec2f& mouse_position() const { return m_mouse_world; }
    // Cursor in pixels, origin at the top-left of the window.
    const FxVec2f& mouse_screen_position() const { return m_mouse_screen; }
    // World-unit movement since the previous frame.
    FxVec2f mouse_delta() const { return m_mouse_world - m_prev_mouse_world; }
    // Scroll wheel movement this frame; positive is away from the user.
    float wheel_delta() const { return m_wheel_delta; }

    // True once a producer has fed this input at least once. False for a headless scene that
    // nobody drives, which lets shared gameplay code branch instead of silently doing nothing.
    bool available() const { return m_available; }

    // ---------------------------------------------------------------- producer API
    // Called by the renderer each frame, or by user code to script a headless scene.

    // Rolls the current state into the previous one so the pressed/released edges refer to the
    // frame about to be described. Call once, before the set_* calls for that frame.
    void begin_frame() {
        m_prev_keys = m_keys;
        m_prev_buttons = m_buttons;
        m_prev_mouse_world = m_mouse_world;
        m_wheel_delta = 0.0f;
        m_available = true;
    }

    void set_key(FxKey key, bool down) { m_keys[index(key)] = down; }
    void set_mouse_button(FxMouseButton button, bool down) { m_buttons[index(button)] = down; }
    void set_mouse_position(const FxVec2f& world, const FxVec2f& screen) {
        m_mouse_world = world;
        m_mouse_screen = screen;
    }
    void set_wheel_delta(float delta) { m_wheel_delta = delta; }

    // Drops every key and button without touching availability. Used when the UI takes focus,
    // so gameplay sees a clean release rather than a key stuck down.
    void release_all() {
        m_keys.fill(false);
        m_buttons.fill(false);
    }

    // Full reset, including availability. Called by FxScene::reset().
    void clear() {
        m_keys.fill(false);
        m_prev_keys.fill(false);
        m_buttons.fill(false);
        m_prev_buttons.fill(false);
        m_mouse_world = {0.0f, 0.0f};
        m_prev_mouse_world = {0.0f, 0.0f};
        m_mouse_screen = {0.0f, 0.0f};
        m_wheel_delta = 0.0f;
        m_available = false;
    }

  private:
    static size_t index(FxKey key) {
        const size_t i = static_cast<size_t>(key);
        return i < kKeyCount ? i : 0; // out-of-range folds onto Unknown, never out of bounds
    }
    static size_t index(FxMouseButton button) {
        const size_t i = static_cast<size_t>(button);
        return i < kButtonCount ? i : 0;
    }

    std::array<bool, kKeyCount> m_keys{};
    std::array<bool, kKeyCount> m_prev_keys{};
    std::array<bool, kButtonCount> m_buttons{};
    std::array<bool, kButtonCount> m_prev_buttons{};
    FxVec2f m_mouse_world{0.0f, 0.0f};
    FxVec2f m_prev_mouse_world{0.0f, 0.0f};
    FxVec2f m_mouse_screen{0.0f, 0.0f};
    float m_wheel_delta = 0.0f;
    bool m_available = false;
};
