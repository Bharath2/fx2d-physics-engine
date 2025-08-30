#include "Renderer.h"
#include "MathTypes.h"

// Simple helpers to convert between our colors and raylib
static inline Color to_rl_color(const FxVec4ui8& c) {
	return Color{ c[0], c[1], c[2], c[3] };
}

Renderer::Renderer(FxScene &scene, int fps, unsigned int scale)
	: scene(scene), m_scale(scale), m_display_w(0), m_display_h(0) {
	init(fps);
}

Renderer::~Renderer() {
	// unload textures
	for (auto &entry : m_textureCache) {
		if (entry.second.id != 0) UnloadTexture(entry.second);
	}
	if (m_hasBackgroundTexture && m_backgroundTexture.id != 0) {
		UnloadTexture(m_backgroundTexture);
	}
	rlImGuiShutdown();
	CloseWindow();
}

void Renderer::init(int fps) {
	// Clamp FPS to reasonable range and warn if outside bounds
	if (fps < 10 || fps > 240) {
		std::cerr << "Warning: Clamping FPS to valid range [10, 240]." << std::endl;
		fps = std::clamp(fps, 10, 240);
	}
	
	// Calculate FIXED_DT from the provided FPS
	m_fixed_dt = 1.0 / static_cast<double>(fps);

	// Calculate window size from scene dimensions
	m_display_w = static_cast<unsigned int>(m_scale * scene.size.x());
	m_display_h = static_cast<unsigned int>(m_scale * scene.size.y());

	SetTraceLogLevel(LOG_NONE); 
	SetConfigFlags(FLAG_WINDOW_RESIZABLE);
	InitWindow(m_display_w, m_display_h, "Fx2D");
	SetTargetFPS(fps);
	rlImGuiSetup(true);

	// Background
	if (!scene.fillTexturePath.empty()) {
		set_background(scene.fillTexturePath);
	} else {
		set_background(scene.fillColor);
	}
}

void Renderer::run() {
	double curr_rt_factor = 0.0;
	
	while (!WindowShouldClose()) {
		double org_frame_dt = static_cast<double>(GetFrameTime());
		double frame_dt = std::min(org_frame_dt, 0.2);
		double sim_in_dt = frame_dt * m_real_time_factor;
		if (m_play) m_dt_accumulator += sim_in_dt;
		// UI: actual applied real-time factor this frame
		curr_rt_factor = frame_dt > 0 ? (sim_in_dt / frame_dt) : 1.0;
		// Consume accumulator 
		double curr_step_dt = std::clamp(m_fixed_dt*m_real_time_factor, m_min_time_step, m_max_time_step);
		while (m_dt_accumulator >= curr_step_dt) {
			scene.step(curr_step_dt);
			m_dt_accumulator -= curr_step_dt;
		}

		BeginDrawing();
		if (m_hasBackgroundTexture) {
			DrawTexturePro(
				m_backgroundTexture,
				{0, 0, static_cast<float>(m_backgroundTexture.width), static_cast<float>(m_backgroundTexture.height)},
				{0, 0, static_cast<float>(m_display_w), static_cast<float>(m_display_h)}, {0, 0}, 0.0f, WHITE
			);
		} else { ClearBackground(m_backgroundColor); }

		draw_scene();
		rlImGuiBegin();
		draw_ui(curr_rt_factor);
		rlImGuiEnd();
		EndDrawing();
	}
}
void Renderer::set_real_time_factor(const double& rt_factor) {
	if (rt_factor < 0.01) {
		std::cerr << "Renderer: real time factor must be greater than 0.01" << std::endl;
		m_real_time_factor = 0.01;
	} else if (rt_factor > 10.0) {
		std::cerr << "Renderer: real time factor must be less than or equal to 10" << std::endl;
		m_real_time_factor = 10.0;
	} else {
		m_real_time_factor = rt_factor;
	}
}

void Renderer::set_background(const FxVec4ui8 &color) {
	m_backgroundColor = to_rl_color(color);
	m_hasBackgroundTexture = false;
}

void Renderer::set_background(const std::string &filepath) {
	Image img = LoadImage(filepath.c_str());
	if (img.data == nullptr) {
		// fallback to color
		m_hasBackgroundTexture = false;
		m_backgroundColor = to_rl_color(scene.fillColor);
		return;
	}
	if (m_backgroundTexture.id != 0) UnloadTexture(m_backgroundTexture);
	m_backgroundTexture = LoadTextureFromImage(img);
	UnloadImage(img);
	m_hasBackgroundTexture = (m_backgroundTexture.id != 0);
}

Texture2D Renderer::get_or_load_texture(const std::string& path) {
	auto it = m_textureCache.find(path);
	if (it != m_textureCache.end()) return it->second;
	Image img = LoadImage(path.c_str());
	if (img.data == nullptr) return Texture2D{}; // invalid texture
	Texture2D tex = LoadTextureFromImage(img);
	UnloadImage(img);
	m_textureCache[path] = tex;
	return tex;
}

void Renderer::draw_scene() {
	// Draw all entities
	scene.for_each_entity(std::execution::seq, [&](FxEntity* e){ 
		const auto& visual = e->visual_geometry();
		if (!visual) return;
		const FxVec3f pose = visual->world_pose();

		// screen mapping helpers
		auto sx = [&](float wx) { return m_scale * wx; };
		auto sy = [&](float wy) { return static_cast<float>(m_display_h) - m_scale * wy; };
		const float x = sx(pose.x());
		const float y = sy(pose.y());

		if (visual->is_circle()) {
			const float r = m_scale * visual->radius();
			bool drewTexture = false;
			if (!visual->fillTexture().empty()) {
				Texture2D tex = get_or_load_texture(visual->fillTexture());
				if (tex.id != 0) {
					// Center the rotation at the circle center
					Rectangle src{0, 0, (float)tex.width, (float)tex.height};
					Rectangle dst{x, y, 2.0f * r, 2.0f * r};
					// rotate about the center of the circle (origin at dst center)
					Vector2 origin{r, r};
					// The earlier sign looked right; keep if it matches your convention.
					const float deg = -pose.theta() * 180.0f / FxPif;
					DrawTexturePro(tex, src, dst, origin, deg, WHITE);
					drewTexture = true;
				}
			}
			if (!drewTexture) {
				DrawCircleV(Vector2{x, y}, r, to_rl_color(visual->fillColor()));
			}
			// outline on top (whether textured or not)
			DrawCircleLinesV(Vector2{x, y}, r, to_rl_color(visual->outlineColor()));
		} else {
			const FxVec2fArray& verts = visual->vertices();
			const size_t n = verts.size();
			bool drewTexture = false;
			if (!visual->fillTexture().empty()) {
				auto bounds = (visual->__vertices()).bounds();
				float minx = bounds[0], miny = bounds[1], maxx = bounds[2], maxy = bounds[3];
				Texture2D tex = get_or_load_texture(visual->fillTexture());
				if (tex.id != 0) {
					// Use DrawTexturePro for the polygon with default values
					Rectangle src{0, 0, (float)tex.width, (float)tex.height};
					Rectangle dst{x, y, m_scale * (maxx - minx), m_scale * (maxy - miny)};
					Vector2 origin{m_scale * (maxx - minx)/2, m_scale * (maxy - miny)/2};
					const float deg = -pose.theta() * 180.0f / FxPif;
					DrawTexturePro(tex, src, dst, origin, deg, WHITE);
					drewTexture = true;
				}
			}
			// screen-space vertices
			std::vector<Vector2> pts;
			pts.reserve(n);
			pts.push_back(Vector2{ sx(verts[0].x()), sy(verts[0].y()) });
			for (size_t i = 1; i < n; ++i) {
				pts.push_back(Vector2{ sx(verts[i].x()), sy(verts[i].y()) });
				// Draw triangle fan for solid fill (skip first two vertices)
				if (!drewTexture && i >= 2)
					DrawTriangle(pts[0], pts[i-1], pts[i], to_rl_color(visual->fillColor()));
				DrawLineV(pts[i-1], pts[i], to_rl_color(visual->outlineColor()));
			}
			// Draw final outline edge from last vertex to first
			DrawLineV(pts[n-1], pts[0], to_rl_color(visual->outlineColor()));
		}
	});
}

void Renderer::draw_ui(double curr_rt_factor) {
	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8, 8));
	ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10, 10));
	ImGui::Begin("Simulation Controls");
	ImGui::SetWindowFocus("Simulation Controls");
	char buf[32];
	// grab the current gravity so we can edit it locally
	static float gravity_x = scene.gravity.x();
	static float gravity_y = scene.gravity.y();
	// ---- GRAVITY X ----
	ImGui::Text("Gravity X:");
	ImGui::SetNextItemWidth(100.0f);
	if (ImGui::InputFloat("##GravityX", &gravity_x, 0.0f, 0.0f, "%.1f")) {
		// clamp into range
		gravity_x = std::clamp(gravity_x, -20.0f, 20.0f);
		scene.gravity.set_x(gravity_x);
	}
	// ---- GRAVITY Y ----
	ImGui::Text("Gravity Y:");
	ImGui::SetNextItemWidth(100.0f);
	if (ImGui::InputFloat("##GravityY", &gravity_y, 0.0f, 0.0f, "%.1f")) {
		gravity_y = std::clamp(gravity_y, -20.0f, 20.0f);
		scene.gravity.set_y(gravity_y);
	}
	ImGui::Dummy(ImVec2(0, 5));
	ImGui::Separator();

	ImGui::Text("Real Time Factor:");
	static float rt_factor = 1.0f;
	std::snprintf(buf, sizeof(buf), "%.3f", static_cast<float>(curr_rt_factor));
	ImGui::SetNextItemWidth(100.0f);
	if (ImGui::InputFloat("##RTFactor", &rt_factor, 0.0f, 0.0f, "%.3f")) {
		set_real_time_factor(static_cast<double>(rt_factor));
	}
	ImGui::SameLine();
	ImGui::Text(" : %.3f", static_cast<float>(curr_rt_factor));
	ImGui::Dummy(ImVec2(0, 5));
	ImGui::Separator();

	// Pause / Play toggle button
	const char* label = m_play ? "Pause" : "Play";
	if (ImGui::Button(label, ImVec2(150, 0))) {
		m_play = !m_play;
	}
	if (ImGui::Button("Reset Simulation", ImVec2(150, 0))) {
		scene.reset();
		scene.step(m_min_time_step);
	}
	ImGui::Dummy(ImVec2(0, 1));
	ImGui::Separator();

	float frame_rate = ImGui::GetIO().Framerate;
	ImGui::Text("Performance:\n %.3f ms/frame\n (%.1f FPS)\n",
				1000.0f / frame_rate, frame_rate);
	ImGui::End();
	ImGui::PopStyleVar(2);
}



//   // Policy: keep min <= base <= max, prefer 1× base most of the time
//   constexpr float kMinMult = 0.5f;  // allow small steps when accumulator is low
//   constexpr float kMaxMult = 3.0f;  // allow fewer, larger steps when catching up

//   m_min_time_step  = kMinMult * m_fixed_dt;  // e.g. 0.5× base
//   m_max_time_step  = kMaxMult * m_fixed_dt;  // e.g. 3×   base


// m_fixed_dt = 1.0f / float(fps);
// float curr_rt_factor = 0.0f;

// constexpr float kHitchCap = 0.25f; // cap huge pauses

// while (!WindowShouldClose()) {
//     float org_frame_dt = GetFrameTime();
// 	float frame_dt =  std::min(org_frame_dt, 0.2f);
//     float sim_in_dt = frame_dt * m_real_time_factor;
//     if (m_play) m_dt_accumulator += sim_in_dt;
//     // UI: actual applied real-time factor this frame
//     curr_rt_factor = frame_dt > 0 ? (sim_in_dt / frame_dt) : 1.0f;
//     // Consume accumulator 
//     while (m_dt_accumulator >= m_min_time_step) {
//         float k = std::min(3.0f, std::floor(m_dt_accumulator / m_fixed_dt));
//         step_dt = std::clamp(k * m_fixed_dt, m_min_time_step, m_max_time_step);
//         scene.step(step_dt);
//         m_dt_accumulator -= step_dt;
//     }