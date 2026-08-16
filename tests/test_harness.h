#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

// Shared assertion helpers. Never use <cassert>: Release defines NDEBUG, which expands
// assert() to nothing, so an assert-based test passes without checking anything.

inline void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

inline void require_near(float actual, float expected, float tolerance,
                         const std::string& message) {
    if (!(std::fabs(actual - expected) <= tolerance)) {
        throw std::runtime_error(message + " (expected " + std::to_string(expected) + " +/- " +
                                 std::to_string(tolerance) + ", got " + std::to_string(actual) +
                                 ")");
    }
}
