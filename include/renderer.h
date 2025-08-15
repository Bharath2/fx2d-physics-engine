#pragma once

#define IMGUI_DEFINE_MATH_OPERATORS
#include "imgui.h"

// Raylib + rlImGui (Dear ImGui binding for raylib)
#define RAYLIB_QUIET  // Suppress raylib logging
#include <raylib.h>
#include <rlgl.h>

// Dear ImGui binding for raylib
#include "rlImGui.h"

#include <memory>
#include <string>
#include <unordered_map>

#include "Core.h"
#include "MathTypes.h"

class Renderer {
private:
	FxScene &scene;
	unsigned int m_scale;
	unsigned int m_display_w, m_display_h;
	// Fixed timestep variables
	float m_fixed_dt = 1.0f / 60.0f;
	float m_dt_accumulator = 0.0f;
	
	// Real-time factor variables
	float m_real_time_factor = 1.0f;
	float m_max_time_step = 0.5f;
	static constexpr float m_min_time_step = 1e-4f;
	
	// background settings
	Color m_backgroundColor { 50, 50, 50, 255 };
	Texture2D m_backgroundTexture { 0, 0, 0, 0, 0 };
	bool m_hasBackgroundTexture { false };

	// texture cache for entity textures
	std::unordered_map<std::string, Texture2D> m_textureCache;
	Texture2D get_or_load_texture(const std::string& path);

	// run-state
	bool m_play = true;

	// helpers
	void init(int fps = 60);
	void draw_ui(float curr_rt_factor);
	void draw_scene();
public:
	Renderer(FxScene &scene, int fps = 60, unsigned int scale = 100);
	~Renderer();

	void run();
	void set_background(const FxVec4ui8 &color);
	void set_background(const std::string &filepath);
	void set_real_time_factor(const float& rt_factor);
	float get_real_time_factor() const { return m_real_time_factor; }
};
