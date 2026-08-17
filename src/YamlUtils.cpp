#include "Fx2D/YamlUtils.h"
#include <vector>

namespace FxYAML {

// Remove the indentation (leading spaces/tabs) from each line of a multi-line string.
std::string deIndent(const std::string& raw) {
    // 1) Find the first non-blank line start
    size_t firstLineStart = raw.find_first_not_of("\r\n");
    if (firstLineStart == std::string::npos) return ""; // empty or all-newlines
    // 2) From there, measure its indent
    size_t firstContent = raw.find_first_not_of(" \t", firstLineStart);
    size_t indent = (firstContent == std::string::npos ? raw.size() - firstLineStart :
                                                         firstContent - firstLineStart);
    // 3) Strip that indent from each line
    std::istringstream in(raw);
    std::ostringstream out;
    std::string line;
    while (std::getline(in, line)) {
        if (line.size() > indent) out << line.substr(indent);
        else out << line;
        out << '\n';
    }
    return out.str();
}

YAML::Node LoadSmart(const std::string& textOrPath) {
    try {
        // Inline YAML is never a path, and probing it as one throws ENAMETOOLONG on Linux.
        const bool looks_like_text = textOrPath.find('\n') != std::string::npos;
        // Treat as file if it exists (error_code overloads stay quiet on invalid paths)
        std::error_code ec;
        if (!looks_like_text && fs::exists(textOrPath, ec) && fs::is_regular_file(textOrPath, ec)) {
            return YAML::LoadFile(textOrPath);
        } else {
            auto text_ = deIndent(textOrPath);
            return YAML::Load(text_);
        }
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("YAML parse error: " + std::string(e.what()));
    } catch (const std::exception& e) {
        throw std::runtime_error("YAML load error: " + std::string(e.what()));
    }
}

FxShape buildShape(const YAML::Node& config) {
    if (!config.IsMap()) {
        throw std::runtime_error("Expected a map for shape configuration.");
    }

    // 0) Read optional pose (default {0,0,0})
    std::array<float, 3> pose_offset = {0.f, 0.f, 0.f};
    if (auto p = config["pose"]) {
        if (!p.IsSequence() || p.size() != 3)
            throw std::runtime_error("pose must be a 3-sequence [x,y,theta].");
        pose_offset = {p[0].as<float>(), p[1].as<float>(), p[2].as<float>()};
    }

    auto geom = config["geometry"];
    if (!geom || !geom.IsMap()) {
        throw std::runtime_error("Expected geometry:{…} block.");
    }
    FxShape shape;
    // Optional uniform Minkowski-sum skin radius for rectangles/polygons (rounded corners).
    const float skin_radius = geom["radius"] ? geom["radius"].as<float>() : 0.0f;

    // 1) Circle?
    if (auto circle_node = geom["circle"]) {
        float radius = circle_node.as<float>();
        shape = FxShape(radius);
    }
    // 2) Capsule? Either {capsule: [length, radius]} or {capsule: {length: L, radius: R}}.
    else if (auto cap_node = geom["capsule"]) {
        float length = 0.0f, radius = 0.0f;
        if (cap_node.IsSequence() && cap_node.size() == 2) {
            length = cap_node[0].as<float>();
            radius = cap_node[1].as<float>();
        } else if (cap_node.IsMap()) {
            if (!cap_node["length"] || !cap_node["radius"])
                throw std::runtime_error("capsule map requires 'length' and 'radius'.");
            length = cap_node["length"].as<float>();
            radius = cap_node["radius"].as<float>();
        } else {
            throw std::runtime_error("capsule must be a 2-sequence [length, radius] or a map.");
        }
        shape = FxShape(length, radius);
    }
    // 3) Rectangle? (optionally rounded if `radius:` is supplied)
    else if (auto rect_node = geom["rectangle"]) {
        if (!rect_node.IsSequence() || rect_node.size() != 2)
            throw std::runtime_error("rectangle must be a 2-sequence.");
        auto sz = parseArray<2>(rect_node);
        shape = FxShape(FxVec2f{sz[0], sz[1]}, skin_radius);
    }
    // 4) Polygon? (optionally rounded if `radius:` is supplied)
    else if (auto poly_node = geom["polygon"]) {
        if (!poly_node.IsSequence()) throw std::runtime_error("polygon must be a sequence.");
        std::vector<FxVec2f> verts;
        for (const auto& pt : poly_node) {
            if (!pt.IsSequence() || pt.size() != 2)
                throw std::runtime_error("each polygon vertex must be a 2-sequence.");
            auto pt_ = parseArray<2>(pt);
            verts.push_back(FxVec2f({pt_[0], pt_[1]}));
        }
        shape = FxShape(verts, skin_radius);
    }
    // 5) Chain? Open polyline of zero-thickness segments, for static level geometry.
    else if (auto chain_node = geom["chain"]) {
        if (!chain_node.IsSequence() || chain_node.size() < 3)
            throw std::runtime_error("chain must be a sequence of at least 3 [x,y] points.");
        std::vector<FxVec2f> pts;
        for (const auto& pt : chain_node) {
            if (!pt.IsSequence() || pt.size() != 2)
                throw std::runtime_error("each chain point must be a 2-sequence.");
            auto p_ = parseArray<2>(pt);
            pts.push_back(FxVec2f({p_[0], p_[1]}));
        }
        shape = FxShape::make_chain(pts);
    }
    // 6) Edge? Zero-thickness segment between two local-frame endpoints.
    else if (auto edge_node = geom["edge"]) {
        if (!edge_node.IsSequence() || edge_node.size() != 2)
            throw std::runtime_error("edge must be a 2-sequence of [x,y] endpoints.");
        auto p0 = parseArray<2>(edge_node[0]);
        auto p1 = parseArray<2>(edge_node[1]);
        shape = FxShape(FxVec2f{p0[0], p0[1]}, FxVec2f{p1[0], p1[1]});
    } else {
        throw std::runtime_error("Unknown geometry type in shape config.");
    }
    // 6) Set the offset pose
    shape.set_offset_pose({pose_offset[0], pose_offset[1], pose_offset[2]});
    return shape;
}

FxVisualShape buildVisualShape(const YAML::Node& config) {
    FxVisualShape shape = FxVisualShape(buildShape(config));

    // Read border color
    if (auto border = config["border_color"]) {
        if (!border.IsSequence() || border.size() != 4) {
            throw std::runtime_error("Border color must be a 4-sequence [R,G,B,A].");
        }
        auto c = (parseArray<4>(border)).as<uint8_t>();
        shape.set_outlineColor({c[0], c[1], c[2], c[3]});

        // Also set border thickness if specified
        if (auto thickness = config["border_thickness"]) {
            if (!thickness.IsScalar())
                throw std::runtime_error("Border thickness must be a scalar value.");
            float border_thickness = thickness.as<float>();
            shape.set_outlineThickness(border_thickness);
        }
    }

    if (auto tex = config["texture"]) {
        // 1) sequence of 4 numbers -> RGBA color
        if (tex.IsSequence() && tex.size() == 4 && tex[0].IsScalar() && tex[1].IsScalar() &&
            tex[2].IsScalar() && tex[3].IsScalar()) {
            auto c = (parseArray<4>(tex)).as<uint8_t>();
            shape.set_fillColor({c[0], c[1], c[2], c[3]});
        }
        // 2) single string -> one texture path
        else if (tex.IsScalar()) {
            shape.set_fillTexture(tex.as<std::string>());
        } else {
            throw std::runtime_error(
                "Texture field must be a RGBA (4-integers) or file path (string).");
        }
    }
    return shape;
}

FxCollisionShape buildCollisionShape(const YAML::Node& config) {
    FxCollisionShape shape = FxCollisionShape(buildShape(config));
    return shape;
}

std::shared_ptr<FxEntity> buildEntity(const std::string& entity_name, const YAML::Node& config) {
    // Check if the node is a map
    if (!config.IsMap()) {
        throw std::runtime_error("Expected a map for entity configuration.");
    }

    auto entity = std::make_shared<FxEntity>(entity_name);

    // Read initial pose
    auto init_pose = config["pose"];
    if (init_pose) {
        auto init_pose_array = parseArray<3>(init_pose);
        entity->set_init_pose({init_pose_array[0], init_pose_array[1], init_pose_array[2]});
    }

    // Read initial velocity
    auto init_velocity = config["init_velocity"];
    if (init_velocity) {
        auto init_velocity_array = parseArray<3>(init_velocity);
        entity->set_init_velocity(
            {init_velocity_array[0], init_velocity_array[1], init_velocity_array[2]});
    }

    // Both deferred until the geometry exists: implicit inertia reads the visual shape, and
    // enable_external_forces() zeroes inv_inertia so it must be applied after any inertia.
    bool inertia_from_visual_shape = false;
    bool external_forces_enabled = true;
    bool has_physics_block = false;

    // Read physics properties
    auto physics = config["physics"];
    if (physics) {
        has_physics_block = true;
        entity->set_mass(physics["mass"].as<float>(1.0f)); // default to 1.0 if not specified
        if (physics["inertia"]) {
            entity->set_inertia(physics["inertia"].as<float>());
        } else {
            inertia_from_visual_shape = true; // computed below, once the shape exists
        }
        entity->gravity_scale = physics["gravity_scale"].as<float>(1.0f); // default to 1.0 if not
                                                                          // specified
        entity->vel_damping = physics["vel_damping"].as<float>(0.0f); // default to 0.0 if not
                                                                      // specified
        entity->elasticity = physics["elasticity"].as<float>(0.1f);
        entity->static_friction = physics["static_friction"].as<float>(0.0f); // default to 0.0 if
                                                                              // not specified
        entity->dynamic_friction = physics["dynamic_friction"].as<float>(0.0f); // default to 0.2 if
                                                                                // not specified
        external_forces_enabled = physics["external_forces_enabled"].as<bool>(true); // applied
                                                                                     // below, after
                                                                                     // inertia
        entity->enable_ccd = physics["ccd"].as<bool>(false); // default to false if not specified
        entity->is_sensor = physics["sensor"].as<bool>(false); // default to false if not specified
    }

    // Read visual properties
    auto visual = config["visual"];
    if (visual) {
        FxVisualShape visual_shape = buildVisualShape(visual);
        entity->set_visual_geometry(visual_shape);
    } else {
        entity->del_visual_geometry();
    }
    // Read collision properties
    auto collision = config["collision"];
    if (collision) {
        FxCollisionShape collision_shape = buildCollisionShape(collision);
        entity->set_collision_geometry(collision_shape);
    } else {
        entity->del_collision_geometry();
    }

    // Deferred from the physics block: inertia is derived from the visual shape and mass, so
    // it can only be computed now that the shape is attached. An explicit physics.inertia was
    // already applied above and is left alone.
    if (inertia_from_visual_shape) {
        entity->set_inertia();
    }
    // Must come last: it zeroes inv_mass / inv_inertia, and would be undone by set_inertia().
    if (has_physics_block) {
        entity->enable_external_forces(external_forces_enabled);
    }
    return entity;
}

std::shared_ptr<FxJoint> buildJoint(const std::string& joint_name, const YAML::Node& config,
                                    FxScene& scene) {
    if (!config.IsMap()) throw std::runtime_error("Expected a map for joint '" + joint_name + "'.");

    // type: revolute | prismatic
    auto type_node = config["type"];
    if (!type_node) throw std::runtime_error("Joint '" + joint_name + "' missing 'type' field.");
    std::string type = type_node.as<std::string>();

    // parent and child entity names
    if (!config["parent"] || !config["child"])
        throw std::runtime_error("Joint '" + joint_name + "' requires 'parent' and 'child'.");
    auto e1 = scene.get_entity(config["parent"].as<std::string>());
    auto e2 = scene.get_entity(config["child"].as<std::string>());
    if (!e1)
        throw std::runtime_error("Joint '" + joint_name + "': parent '" +
                                 config["parent"].as<std::string>() + "' not found.");
    if (!e2)
        throw std::runtime_error("Joint '" + joint_name + "': child '" +
                                 config["child"].as<std::string>() + "' not found.");

    // pid: [p, i, d]  (optional, defaults to [1, 0, 0])
    float p = 1.0f, i_gain = 0.0f, d = 0.0f;
    if (auto pid = config["pid"]) {
        if (!pid.IsSequence() || pid.size() != 3)
            throw std::runtime_error("Joint '" + joint_name +
                                     "': pid must be a 3-sequence [p, i, d].");
        p = pid[0].as<float>();
        i_gain = pid[1].as<float>();
        d = pid[2].as<float>();
    }

    // control_mode: "position" (default) | "velocity" | "effort"
    ControlMode ctrl_mode = ControlMode::POSITION;
    if (auto cm = config["control_mode"]) {
        std::string s = cm.as<std::string>();
        if (s == "velocity") ctrl_mode = ControlMode::VELOCITY;
        else if (s == "effort") ctrl_mode = ControlMode::EFFORT;
        else if (s != "position")
            throw std::runtime_error("Joint '" + joint_name + "': unknown control_mode '" + s +
                                     "'.");
    }

    std::shared_ptr<FxJoint> joint;

    if (type == "revolute") {
        // anchor: local point on parent where the pin sits (defaults to parent origin)
        FxVec2f anchor{0.0f, 0.0f};
        if (auto an = config["anchor"]) {
            if (!an.IsSequence() || an.size() != 2)
                throw std::runtime_error("Joint '" + joint_name + "': anchor must be [x, y].");
            anchor = FxVec2f{an[0].as<float>(), an[1].as<float>()};
        }
        float angle_min = config["angle_min"] ? config["angle_min"].as<float>() : -FxPif;
        float angle_max = config["angle_max"] ? config["angle_max"].as<float>() : FxPif;
        auto rj =
            std::make_shared<FxRevoluteJoint>(joint_name, e1, e2, anchor, angle_min, angle_max);
        rj->set_control_mode(ctrl_mode);
        // Prefer unified max_effort, but keep max_torque for older scene files.
        if (config["max_effort"]) rj->set_max_effort(config["max_effort"].as<float>());
        else if (config["max_torque"]) rj->set_max_torque(config["max_torque"].as<float>());
        // target: angle (rad), angular rate (rad/s), or torque effort
        if (auto tgt = config["target"]) {
            if (ctrl_mode == ControlMode::VELOCITY) rj->set_omega(tgt.as<float>(), false);
            else if (ctrl_mode == ControlMode::EFFORT) rj->set_effort(tgt.as<float>());
            else rj->set_theta(tgt.as<float>(), false);
        }
        joint = rj;

    } else if (type == "prismatic") {
        // axis: local axis direction on parent (defaults to [1, 0])
        FxVec2f axis{1.0f, 0.0f};
        if (auto ax = config["axis"]) {
            if (!ax.IsSequence() || ax.size() != 2)
                throw std::runtime_error("Joint '" + joint_name + "': axis must be [x, y].");
            axis = FxVec2f{ax[0].as<float>(), ax[1].as<float>()};
        }
        float pos_min = config["position_min"] ? config["position_min"].as<float>() : -1000.0f;
        float pos_max = config["position_max"] ? config["position_max"].as<float>() : 1000.0f;
        auto pj = std::make_shared<FxPrismaticJoint>(joint_name, e1, e2, axis, pos_min, pos_max);
        pj->set_control_mode(ctrl_mode);
        // Prefer unified max_effort, but keep max_force for older scene files.
        if (config["max_effort"]) pj->set_max_effort(config["max_effort"].as<float>());
        else if (config["max_force"]) pj->set_max_force(config["max_force"].as<float>());
        // target: position (m), velocity (m/s), or force effort
        if (auto tgt = config["target"]) {
            if (ctrl_mode == ControlMode::VELOCITY) pj->set_velocity(tgt.as<float>(), false);
            else if (ctrl_mode == ControlMode::EFFORT) pj->set_effort(tgt.as<float>());
            else pj->set_position(tgt.as<float>(), false);
        }
        joint = pj;

    } else {
        throw std::runtime_error("Joint '" + joint_name + "': unknown type '" + type +
                                 "'. Expected 'revolute' or 'prismatic'.");
    }

    joint->set_pid(FxVec3f{p, i_gain, d});
    if (config["entities_collide"]) joint->entities_collide = config["entities_collide"].as<bool>();
    return joint;
}

// Implementation of buildScene
FxScene buildScene(const YAML::Node& config) {
    // Read scene properties
    auto scene_config = config["scene"];
    if (!scene_config) {
        throw std::runtime_error("Scene configuration not found in YAML file.");
    }

    auto scene_size = scene_config["size"];
    if (!scene_size) {
        throw std::runtime_error("Scene size not defined in YAML file.");
    }
    auto scene_size_array = parseArray<2>(scene_size);
    FxScene scene({scene_size_array[0], scene_size_array[1]});

    // Read gravity from scene config
    auto gravity = scene_config["gravity"];
    if (gravity) {
        auto gravity_array = parseArray<2>(gravity);
        scene.set_gravity({gravity_array[0], gravity_array[1]});
    }

    // Read background from scene config
    if (auto tex = scene_config["background"]) {
        // 1) sequence of 4 numbers -> RGBA color
        if (tex.IsSequence() && tex.size() == 4 && tex[0].IsScalar() && tex[1].IsScalar() &&
            tex[2].IsScalar() && tex[3].IsScalar()) {
            auto c = (parseArray<4>(tex)).as<uint8_t>();
            scene.set_fillColor({c[0], c[1], c[2], c[3]});
        }
        // 2) single string -> one texture path
        else if (tex.IsScalar()) {
            scene.set_fillTexture(tex.as<std::string>());
        } else {
            throw std::runtime_error(
                "Scene background must be a RGBA (4-integers) or file path (string).");
        }
    }

    // Read entities
    auto entities = config["entities"];
    if (!entities) {
        std::cerr << "No entities defined in YAML file." << std::endl;
    } else {
        for (const auto& kv : entities) {
            const std::string entity_name = kv.first.as<std::string>();
            const YAML::Node entity_node = kv.second;
            auto entity = buildEntity(entity_name, entity_node);
            scene.add_entity(entity);
        }
    }
    // Read joints
    auto joints_node = config["joints"];
    if (joints_node) {
        for (const auto& kv : joints_node) {
            const std::string joint_name = kv.first.as<std::string>();
            try {
                auto joint = buildJoint(joint_name, kv.second, scene);
                scene.add_joint(joint);
            } catch (const std::exception& ex) {
                std::cerr << "Warning: skipping joint '" << joint_name << "': " << ex.what()
                          << std::endl;
            }
        }
    }
    // Return the constructed scene
    return scene;
}

} // namespace FxYAML
