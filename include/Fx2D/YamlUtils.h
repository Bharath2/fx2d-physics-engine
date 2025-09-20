#pragma once 

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <array>
#include <yaml-cpp/yaml.h>
#include <filesystem>

#include "Fx2D/Math.h"
#include "Fx2D/Core.h"

namespace fs = std::filesystem;

namespace FxYAML {

    // Remove the indentation (leading spaces/tabs) from each line of a multi-line string.
    std::string deIndent(const std::string& raw);

    // Parse a YAML sequence of length N into a FxArray<float>.
    template<int N>
    inline FxArray<float> parseArray(const YAML::Node &node) {
        FxArray<float> arr(N);
        for (int i = 0; i < N; ++i)
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

    inline FxScene buildScene(const std::string& textOrPath){
        YAML::Node config = LoadSmart(textOrPath);
        return buildScene(config);
    }
    inline std::shared_ptr<FxEntity> buildEntity(const std::string& entity_name, const std::string& textOrPath){
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
} 
