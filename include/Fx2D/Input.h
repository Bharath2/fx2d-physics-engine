#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "Fx2D/Math.h"

// Renderer-agnostic input state, so headless and windowed builds share one interface.
// A renderer fills it each frame; headless callers inject through the producer API.

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

    // Held down right now; what continuous control wants.
    bool key_down(FxKey key) const { return m_keys[index(key)]; }
    // Went down since the last frame. Edges last a whole frame, which may span several
    // physics steps, so key_down() is safer for anything that must not repeat.
    bool key_pressed(FxKey key) const { return m_keys[index(key)] && !m_prev_keys[index(key)]; }
    bool key_released(FxKey key) const { return !m_keys[index(key)] && m_prev_keys[index(key)]; }

    bool mouse_down(FxMouseButton button) const { return m_buttons[index(button)]; }
    bool mouse_pressed(FxMouseButton button) const {
        return m_buttons[index(button)] && !m_prev_buttons[index(button)];
    }
    bool mouse_released(FxMouseButton button) const {
        return !m_buttons[index(button)] && m_prev_buttons[index(button)];
    }

    // Cursor in scene units, the same frame entity poses live in.
    const FxVec2f& mouse_position() const { return m_mouse_world; }
    // Cursor in pixels, origin at the top-left of the window.
    const FxVec2f& mouse_screen_position() const { return m_mouse_screen; }
    // World-unit movement since the previous frame.
    FxVec2f mouse_delta() const { return m_mouse_world - m_prev_mouse_world; }
    // Scroll wheel movement this frame; positive is away from the user.
    float wheel_delta() const { return m_wheel_delta; }

    // False until something feeds this, distinguishing "no key held" from "no keyboard".
    bool available() const { return m_available; }

    // ---------------------------------------------------------------- producer API

    // Rolls current state into previous so edges describe the frame about to be set.
    // Call once per frame, before the set_* calls.
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

    // Drops held keys and buttons but stays available; for when the UI takes focus.
    void release_all() {
        m_keys.fill(false);
        m_buttons.fill(false);
    }

    // Full reset, including availability.
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
