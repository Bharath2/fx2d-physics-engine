#pragma once

// Raylib + rlImGui (Dear ImGui binding for raylib)
#define RAYLIB_QUIET // Suppress raylib logging
#include <raylib.h>

#define IMGUI_DEFINE_MATH_OPERATORS
#include "imgui.h"
#include "rlImGui.h"
#include <rlgl.h>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "Fx2D/Math.h"
#include "Fx2D/Physics.h"

class FxRylbRenderer {
  protected:
    FxScene& m_scene;
    unsigned int m_scale;
    unsigned int m_display_w, m_display_h;
    // Fixed timestep variables
    double m_fixed_dt = 1.0 / 60.0;
    double m_dt_accumulator = 0.0;
    double m_simulation_time = 0.0; // total time elaspsed from start in seconds
    // Real-time factor variables
    double m_real_time_factor = 1.0;
    double m_max_time_step = 0.06;
    double m_min_time_step = 1e-3;
    bool m_play = true; // run-state
    // background settings
    Color m_backgroundColor{50, 50, 50, 255};
    Texture2D m_backgroundTexture{0, 0, 0, 0, 0};
    bool m_hasBackgroundTexture{false};
    // texture cache for entity textures
    std::unordered_map<std::string, Texture2D> m_textureCache;
    Texture2D get_or_load_texture(const std::string& path);
    // user overlay drawn after the scene, before the UI
    std::function<void(FxRylbRenderer&)> m_func_draw_callback;
    // helpers
    void init(int fps = 60);
    void draw_ui(double curr_rt_factor);
    void draw_scene();
    // Polls keyboard and mouse into m_scene.input(), once per rendered frame.
    void poll_input();

  public:
    FxRylbRenderer(FxScene& scene, int fps = 60, unsigned int scale = 100);
    ~FxRylbRenderer();

    void run(bool play = true);
    void set_background(const FxVec4ui8& color);
    void set_background(const std::string& filepath);
    void set_real_time_factor(const double& rt_factor);
    double get_real_time_factor() const { return m_real_time_factor; }

    // Convert between window pixels and scene units.
    FxVec2f screen_to_world(const FxVec2f& screen) const;
    FxVec2f world_to_screen(const FxVec2f& world) const;

    // Custom drawing over the scene, before the UI panels. Use world_to_screen() to place
    // things in scene coordinates.
    void set_draw_callback(const std::function<void(FxRylbRenderer&)>& callback) {
        m_func_draw_callback = callback;
    }
};
