#include "Fx2D/Renderer.h"
#include "Fx2D/Math.h"

// Helper to convert between our colors and raylib
static inline Color to_rl_color(const FxVec4ui8& c) {
    return Color{c[0], c[1], c[2], c[3]};
}

FxRylbRenderer::FxRylbRenderer(FxScene& scene, int fps, unsigned int scale) :
    m_scene(scene), m_scale(scale), m_display_w(0), m_display_h(0) {
    init(fps);
}

FxRylbRenderer::~FxRylbRenderer() {
    // unload textures
    for (auto& entry : m_textureCache) {
        if (entry.second.id != 0) UnloadTexture(entry.second);
    }
    if (m_hasBackgroundTexture && m_backgroundTexture.id != 0) {
        UnloadTexture(m_backgroundTexture);
    }
    rlImGuiShutdown();
    CloseWindow();
}

void FxRylbRenderer::init(int fps) {
    // Clamp FPS to reasonable range and warn if outside bounds
    if (fps < 10 || fps > 240) {
        std::cerr << "Warning: Clamping FPS to valid range [10, 240]." << std::endl;
        fps = std::clamp(fps, 10, 240);
    }

    // Calculate FIXED_DT from the provided FPS
    m_fixed_dt = 1.0 / static_cast<double>(fps);

    // Calculate window size from scene dimensions
    m_display_w = static_cast<unsigned int>(m_scale * m_scene.size.x());
    m_display_h = static_cast<unsigned int>(m_scale * m_scene.size.y());

    SetTraceLogLevel(LOG_NONE);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(static_cast<int>(m_display_w), static_cast<int>(m_display_h), "Fx2D");
    SetTargetFPS(fps);
    rlImGuiSetup(true);

    // Background
    if (!m_scene.fillTexturePath.empty()) {
        set_background(m_scene.fillTexturePath);
    } else {
        set_background(m_scene.fillColor);
    }
}

void FxRylbRenderer::run(bool play) {
    double curr_rt_factor = 0.0;
    m_play = play;
    m_scene.step(m_min_time_step);
    // Main loop
    while (!WindowShouldClose()) {
        double org_frame_dt = static_cast<double>(GetFrameTime());
        double frame_dt = std::min(org_frame_dt, 0.2);
        double sim_in_dt = frame_dt * m_real_time_factor;
        if (m_play) m_dt_accumulator += sim_in_dt;
        // UI: actual applied real-time factor this frame
        curr_rt_factor = frame_dt > 0 ? (sim_in_dt / frame_dt) : 1.0;
        // Consume accumulator
        double curr_step_dt =
            std::clamp(m_fixed_dt * m_real_time_factor, m_min_time_step, m_max_time_step);
        while (m_dt_accumulator >= curr_step_dt) {
            m_scene.step(curr_step_dt);
            m_dt_accumulator -= curr_step_dt;
        }

        BeginDrawing();
        if (m_hasBackgroundTexture) {
            DrawTexturePro(m_backgroundTexture,
                           {0, 0, static_cast<float>(m_backgroundTexture.width),
                            static_cast<float>(m_backgroundTexture.height)},
                           {0, 0, static_cast<float>(m_display_w), static_cast<float>(m_display_h)},
                           {0, 0}, 0.0f, WHITE);
        } else {
            ClearBackground(m_backgroundColor);
        }

        draw_scene();
        rlImGuiBegin();
        draw_ui(curr_rt_factor);
        rlImGuiEnd();
        EndDrawing();
    }
}

void FxRylbRenderer::set_real_time_factor(const double& rt_factor) {
    if (rt_factor < 0.01) {
        std::cerr << "FxRylbRenderer: real time factor must be greater than 0.01" << std::endl;
        m_real_time_factor = 0.01;
    } else if (rt_factor > 10.0) {
        std::cerr << "FxRylbRenderer: real time factor must be less than or equal to 10"
                  << std::endl;
        m_real_time_factor = 10.0;
    } else {
        m_real_time_factor = rt_factor;
    }
}

void FxRylbRenderer::set_background(const FxVec4ui8& color) {
    m_backgroundColor = to_rl_color(color);
    m_hasBackgroundTexture = false;
}

void FxRylbRenderer::set_background(const std::string& filepath) {
    Image img = LoadImage(filepath.c_str());
    if (img.data == nullptr) {
        // fallback to color
        m_hasBackgroundTexture = false;
        m_backgroundColor = to_rl_color(m_scene.fillColor);
        return;
    }
    if (m_backgroundTexture.id != 0) UnloadTexture(m_backgroundTexture);
    m_backgroundTexture = LoadTextureFromImage(img);
    UnloadImage(img);
    m_hasBackgroundTexture = (m_backgroundTexture.id != 0);
}

Texture2D FxRylbRenderer::get_or_load_texture(const std::string& path) {
    auto it = m_textureCache.find(path);
    if (it != m_textureCache.end()) return it->second;
    Image img = LoadImage(path.c_str());
    if (img.data == nullptr) {
        // Cache the failure too, otherwise every frame retries the missing file.
        m_textureCache[path] = Texture2D{};
        TraceLog(LOG_WARNING, "Fx2D: texture '%s' failed to load; falling back to fill color",
                 path.c_str());
        return Texture2D{};
    }
    Texture2D tex = LoadTextureFromImage(img);
    UnloadImage(img);
    m_textureCache[path] = tex;
    return tex;
}

void FxRylbRenderer::draw_scene() {
    // Helper for nice ring tessellation
    auto ring_segments = [](float radius_px) {
        int seg = (int)ceilf((2.0f * PI * radius_px) / 6.0f);
        if (seg < 16) seg = 16;
        if (seg > 256) seg = 256;
        return seg;
    };

    // Draw all entities - skip disabled entities
    m_scene.for_each_entity(std::execution::seq, [&](FxEntity* e) {
        if (!e->enabled) return; // Skip disabled entities

        const auto& visual = e->visual_geometry();
        if (!visual) return;

        const FxVec3f pose = visual->world_pose();
        // screen mapping helpers
        auto sx = [&](float wx) { return static_cast<float>(m_scale) * wx; };
        auto sy = [&](float wy) {
            return static_cast<float>(m_display_h) - static_cast<float>(m_scale) * wy;
        };
        const float x = sx(pose.x());
        const float y = sy(pose.y());

        if (visual->is_circle()) {
            const float r = static_cast<float>(m_scale) * visual->skin_radius();
            bool textured = false;
            if (!visual->fillTexture().empty()) {
                Texture2D tex = get_or_load_texture(visual->fillTexture());
                if (tex.id != 0) {
                    Rectangle src{0, 0, (float)tex.width, (float)tex.height};
                    Rectangle dst{x, y, 2.0f * r, 2.0f * r};
                    Vector2 origin{r, r};
                    const float deg = -pose.theta() * 180.0f / FxPif;
                    DrawTexturePro(tex, src, dst, origin, deg, to_rl_color(visual->fillColor()));
                    textured = true;
                }
            }
            if (!textured) {
                DrawCircleV(Vector2{x, y}, r, to_rl_color(visual->fillColor()));
            }
            const float outline_px = std::max(0.0f, visual->outlineThickness());
            if (outline_px > 0.0f) {
                const float inner = std::max(0.0f, r - 0.5f * outline_px);
                const float outer = r + 0.5f * outline_px;
                DrawRing(Vector2{x, y}, inner, outer, 0.0f, 360.0f, ring_segments(r),
                         to_rl_color(visual->outlineColor()));
            }
        } else if (visual->is_capsule()) {
            // Capsule = central rectangle (length x 2r) + two end-cap discs at the segment
            // endpoints.
            const FxVec2fArray& local = visual->__vertices();
            const float skin_w = static_cast<float>(m_scale) * visual->skin_radius();
            const float cosT = cosf(pose.theta()), sinT = sinf(pose.theta());
            const FxVec2f C = pose.get_xy();
            auto to_screen = [&](const FxVec2f& v) {
                float wx = C.x() + cosT * v.x() - sinT * v.y();
                float wy = C.y() + sinT * v.x() + cosT * v.y();
                return Vector2{sx(wx), sy(wy)};
            };
            const Vector2 p0 = to_screen(local[0]);
            const Vector2 p1 = to_screen(local[1]);
            const Color fill = to_rl_color(visual->fillColor());
            if (visual->is_edge()) {
                // Zero-thickness segment: nothing to fill, so draw the outline as the shape itself.
                const float thickness =
                    (visual->outlineThickness() > 0.0f) ? visual->outlineThickness() : 2.0f;
                const Color oc = to_rl_color(visual->outlineColor());
                DrawLineEx(p0, p1, thickness, oc);
                DrawCircleV(p0, 0.5f * thickness, oc);
                DrawCircleV(p1, 0.5f * thickness, oc);
                return;
            }
            // Body: line segment of thickness 2*skin_w covers the central rectangle exactly.
            if (skin_w > 0.0f) DrawLineEx(p0, p1, 2.0f * skin_w, fill);
            // End caps: discs at each segment endpoint.
            if (skin_w > 0.0f) {
                DrawCircleV(p0, skin_w, fill);
                DrawCircleV(p1, skin_w, fill);
            }
            // Outline: offset edges along the segment normal + arcs at each end.
            const float outline_px = std::max(0.0f, visual->outlineThickness());
            if (outline_px > 0.0f) {
                const Color oc = to_rl_color(visual->outlineColor());
                // Direction along the segment in screen space (note y is flipped).
                float dx = p1.x - p0.x, dy = p1.y - p0.y;
                float len = std::sqrt(dx * dx + dy * dy);
                if (len > 1e-6f) {
                    float ux = dx / len, uy = dy / len;
                    float nx = -uy, ny = ux; // CCW perpendicular in screen space
                    Vector2 a0{p0.x + nx * skin_w, p0.y + ny * skin_w};
                    Vector2 a1{p1.x + nx * skin_w, p1.y + ny * skin_w};
                    Vector2 b0{p0.x - nx * skin_w, p0.y - ny * skin_w};
                    Vector2 b1{p1.x - nx * skin_w, p1.y - ny * skin_w};
                    DrawLineEx(a0, a1, outline_px, oc);
                    DrawLineEx(b0, b1, outline_px, oc);
                    // End-cap rings
                    const float inner = std::max(0.0f, skin_w - 0.5f * outline_px);
                    const float outer = skin_w + 0.5f * outline_px;
                    // atan2 returns [-pi, pi]; raylib DrawRing uses degrees, sweep CW.
                    const float ang0_deg = atan2f(ny, nx) * 180.0f / PI;
                    DrawRing(p0, inner, outer, ang0_deg, ang0_deg + 180.0f, ring_segments(skin_w),
                             oc);
                    DrawRing(p1, inner, outer, ang0_deg + 180.0f, ang0_deg + 360.0f,
                             ring_segments(skin_w), oc);
                } else {
                    // Degenerate (zero-length) capsule: just draw a ring like a circle.
                    const float inner = std::max(0.0f, skin_w - 0.5f * outline_px);
                    const float outer = skin_w + 0.5f * outline_px;
                    DrawRing(p0, inner, outer, 0.0f, 360.0f, ring_segments(skin_w), oc);
                }
            }
        } else {
            const FxVec2fArray& local = visual->__vertices();
            const size_t n = local.size();
            if (n < 3) return; // A polygon needs at least 3 vertices.

            //  build screen-space vertices by applying pose (T·R) to LOCAL verts ---
            std::vector<Vector2> pts;
            pts.reserve(n);
            const float c = cosf(pose.theta()), s = sinf(pose.theta());
            const FxVec2f C = pose.get_xy(); // world center
            for (size_t i = 0; i < n; ++i) {
                float wx = C.x() + c * local[i].x() - s * local[i].y();
                float wy = C.y() + s * local[i].x() + c * local[i].y();
                pts.push_back(Vector2{sx(wx), sy(wy)});
            }

            const float skin_w = static_cast<float>(m_scale) * visual->skin_radius();
            const Color fill = to_rl_color(visual->fillColor());

            // textured fill
            rlDisableBackfaceCulling();
            bool textured = false;
            if (!visual->fillTexture().empty()) {
                Texture2D tex = get_or_load_texture(visual->fillTexture());
                if (tex.id != 0) {
                    textured = true;
                    auto b = local.bounds(); // {minx,miny,maxx,maxy} in LOCAL space
                    const float minx = b[0], miny = b[1], maxx = b[2], maxy = b[3];
                    const float invW = (maxx > minx) ? 1.0f / (maxx - minx) : 0.0f;
                    const float invH = (maxy > miny) ? 1.0f / (maxy - miny) : 0.0f;
                    rlBegin(RL_TRIANGLES);
                    rlColor4ub(255, 255, 255, 255);
                    rlSetTexture(tex.id);
                    auto emit_vertex = [&](size_t i) {
                        float u = (local[i].x() - minx) * invW;
                        // Local y grows upward but texture v grows downward, so v must be
                        // measured from maxy - otherwise the image renders upside down.
                        float v = (maxy - local[i].y()) * invH;
                        rlTexCoord2f(u, v);
                        rlVertex2f(pts[i].x, pts[i].y); // Use screen-space position
                    };
                    // Create a triangle fan from the vertices, anchored at vertex 0
                    for (size_t i = 1; i < n - 1; ++i) {
                        emit_vertex(0);
                        emit_vertex(i);
                        emit_vertex(i + 1);
                    }
                    rlEnd();
                    rlSetTexture(0);
                }
            }
            if (!textured) {
                DrawTriangleFan(pts.data(), (int)n, fill);
            }

            // Rounded-polygon skin: thick edges (2*skin_w wide) + corner discs cover the
            // Minkowski-sum band.
            if (skin_w > 0.0f) {
                for (size_t i = 0; i < n; ++i) {
                    DrawLineEx(pts[i], pts[(i + 1) % n], 2.0f * skin_w, fill);
                }
                for (const Vector2& p : pts) {
                    DrawCircleV(p, skin_w, fill);
                }
            }

            // outline (thickness with rounded joins)
            const float outline_px = std::max(0.0f, visual->outlineThickness());
            if (outline_px > 0.0f) {
                const Color oc = to_rl_color(visual->outlineColor());
                if (skin_w > 0.0f) {
                    // Rounded-polygon outline: offset each edge outward by skin and stitch with
                    // arcs.
                    std::vector<Vector2> oA(n), oB(n);
                    for (size_t i = 0; i < n; ++i) {
                        Vector2 a = pts[i];
                        Vector2 b = pts[(i + 1) % n];
                        float dx = b.x - a.x, dy = b.y - a.y;
                        float len = std::sqrt(dx * dx + dy * dy);
                        if (len < 1e-6f) {
                            oA[i] = a;
                            oB[i] = b;
                            continue;
                        }
                        // Outward normal (polygon stored CCW in world but screen-space y is
                        // flipped, so the outward direction in screen space is the CW perpendicular
                        // of edge dir).
                        float nx = dy / len, ny = -dx / len;
                        oA[i] = Vector2{a.x + nx * skin_w, a.y + ny * skin_w};
                        oB[i] = Vector2{b.x + nx * skin_w, b.y + ny * skin_w};
                    }
                    const float inner = std::max(0.0f, skin_w - 0.5f * outline_px);
                    const float outer = skin_w + 0.5f * outline_px;
                    for (size_t i = 0; i < n; ++i) {
                        DrawLineEx(oA[i], oB[i], outline_px, oc);
                        // Corner arc at pts[i] joining edge (i-1) end to edge i start.
                        const size_t i_prev = (i + n - 1) % n;
                        float ang0 =
                            atan2f(oB[i_prev].y - pts[i].y, oB[i_prev].x - pts[i].x) * 180.0f / PI;
                        float ang1 = atan2f(oA[i].y - pts[i].y, oA[i].x - pts[i].x) * 180.0f / PI;
                        while (ang1 < ang0)
                            ang1 += 360.0f;
                        DrawRing(pts[i], inner, outer, ang0, ang1, ring_segments(skin_w), oc);
                    }
                } else {
                    for (size_t i = 0; i < n; ++i) {
                        const Vector2 a = pts[i];
                        const Vector2 b = pts[(i + 1) % n];
                        DrawLineEx(a, b, outline_px, oc);
                    }
                    // rounded joins
                    const float jr = 0.5f * outline_px;
                    for (const Vector2& p : pts) {
                        DrawCircleV(p, jr, oc);
                    }
                }
            }
        }
    });
}

void FxRylbRenderer::draw_ui(double curr_rt_factor) {
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8, 8));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10, 10));
    ImGui::Begin("Simulation Controls");
    ImGui::SetWindowFocus("Simulation Controls");
    char buf[32];
    // grab the current gravity so we can edit it locally
    static float gravity_x = m_scene.gravity.x();
    static float gravity_y = m_scene.gravity.y();
    // ---- GRAVITY X ----
    ImGui::Text("Gravity X:");
    ImGui::SetNextItemWidth(100.0f);
    if (ImGui::InputFloat("##GravityX", &gravity_x, 0.0f, 0.0f, "%.1f")) {
        // clamp into range
        gravity_x = std::clamp(gravity_x, -20.0f, 20.0f);
        m_scene.gravity.set_x(gravity_x);
    }
    // ---- GRAVITY Y ----
    ImGui::Text("Gravity Y:");
    ImGui::SetNextItemWidth(100.0f);
    if (ImGui::InputFloat("##GravityY", &gravity_y, 0.0f, 0.0f, "%.1f")) {
        gravity_y = std::clamp(gravity_y, -20.0f, 20.0f);
        m_scene.gravity.set_y(gravity_y);
    }
    ImGui::Dummy(ImVec2(0, 1));
    ImGui::Separator();

    ImGui::Text("Real Time Factor");
    static float rt_factor = 1.0f;
    std::snprintf(buf, sizeof(buf), "%.3f", static_cast<float>(curr_rt_factor));
    ImGui::SameLine();
    ImGui::Text(": %.3f", static_cast<float>(curr_rt_factor));
    ImGui::SetNextItemWidth(100.0f);
    if (ImGui::InputFloat("##RTFactor", &rt_factor, 0.0f, 0.0f, "%.3f")) {
        set_real_time_factor(static_cast<double>(rt_factor));
    }
    ImGui::Dummy(ImVec2(0, 1));
    ImGui::Separator();

    // Pause / Play toggle button
    const char* label = m_play ? "Pause" : "Play";
    if (ImGui::Button(label, ImVec2(150, 0))) {
        m_play = !m_play;
    }
    if (ImGui::Button("Reset Simulation", ImVec2(150, 0))) {
        m_scene.reset();
        m_scene.step(m_min_time_step);
    }
    ImGui::Dummy(ImVec2(0, 1));
    ImGui::Separator();

    ImGui::Text("Simulation Time: %.2f s", static_cast<float>(m_scene.time_elapsed()));
    ImGui::Dummy(ImVec2(0, 1));
    ImGui::Separator();

    float frame_rate = ImGui::GetIO().Framerate;
    ImGui::Text("Performance:\n %.3f ms/frame\n (%.1f FPS)\n", 1000.0f / frame_rate, frame_rate);
    ImGui::End();
    ImGui::PopStyleVar(2);
}
