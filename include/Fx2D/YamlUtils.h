#pragma once

#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <yaml-cpp/yaml.h>

#include "Fx2D/Entity.h"
#include "Fx2D/Math.h"
#include "Fx2D/Scene.h"

namespace fs = std::filesystem;

namespace FxYAML {

// Remove the indentation (leading spaces/tabs) from each line of a multi-line string.
std::string deIndent(const std::string& raw);

// Parse a YAML sequence of length N into a FxArray<float>.
template<int N>
inline FxArray<float> parseArray(const YAML::Node& node) {
    FxArray<float> arr(N);
    for (std::size_t i = 0; i < static_cast<std::size_t>(N); ++i)
        arr[i] = node[i].as<float>();
    return arr;
}

YAML::Node LoadSmart(const std::string& textOrPath);
// Constructers from a YAML nodes.
FxScene buildScene(const YAML::Node& config);
FxShape buildShape(const YAML::Node& config);
FxVisualShape buildVisualShape(const YAML::Node& config);
FxCollisionShape buildCollisionShape(const YAML::Node& config);
std::shared_ptr<FxEntity> buildEntity(const std::string& entity_name, const YAML::Node& config);
// Build a joint from a YAML node; looks up parent/child entities by name from scene.
std::shared_ptr<FxJoint> buildJoint(const std::string& joint_name, const YAML::Node& config,
                                    FxScene& scene);

inline FxScene buildScene(const std::string& textOrPath) {
    YAML::Node config = LoadSmart(textOrPath);
    return buildScene(config);
}
inline std::shared_ptr<FxEntity> buildEntity(const std::string& entity_name,
                                             const std::string& textOrPath) {
    YAML::Node config = LoadSmart(textOrPath);
    return buildEntity(entity_name, config);
}
inline FxShape buildShape(const std::string& textOrPath) {
    YAML::Node config = LoadSmart(textOrPath);
    return buildShape(config);
}
inline FxVisualShape buildVisualShape(const std::string& textOrPath) {
    YAML::Node config = LoadSmart(textOrPath);
    return buildVisualShape(config);
}

inline FxVisualShape buildCollisionShape(const std::string& textOrPath) {
    YAML::Node config = LoadSmart(textOrPath);
    return buildCollisionShape(config);
}
} // namespace FxYAML
