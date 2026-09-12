#pragma once

// std::execution policies where the standard library provides them, and sequential stand-ins
// where it does not. libc++ ships <execution> without the policies unless its experimental
// PSTL is enabled, which is what Emscripten builds against. The engine only ever asks for seq
// (see the note above FxScene::step), so the stand-in loses nothing.
#include <version>

#if defined(__cpp_lib_execution)
#include <execution>
#define FX2D_HAS_EXECUTION_POLICIES 1
#else
#include <algorithm>
#define FX2D_HAS_EXECUTION_POLICIES 0
namespace std::execution {
struct sequenced_policy {};
inline constexpr sequenced_policy seq{};
} // namespace std::execution
#endif
