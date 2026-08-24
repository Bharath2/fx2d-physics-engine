#include "Fx2D/Profile.h"

namespace FxProfile {
namespace {
double g_slots[SlotCount] = {};
std::size_t g_steps = 0;

const char* const kNames[SlotCount] = {
    "broad phase",     "narrow phase",    "position solve", "constraints", "integration",
    "velocity derive", "velocity passes", "bookkeeping",    "STEP TOTAL",
};
} // namespace

bool enabled() {
#ifdef FX2D_PROFILE
    return true;
#else
    return false;
#endif
}

const char* slot_name(Slot slot) {
    if (slot < 0 || slot >= SlotCount) return "?";
    return kNames[slot];
}

double ms(Slot slot) {
    if (slot < 0 || slot >= SlotCount) return 0.0;
    return g_slots[slot];
}

std::size_t steps() {
    return g_steps;
}

void reset() {
    for (int i = 0; i < SlotCount; ++i)
        g_slots[i] = 0.0;
    g_steps = 0;
}

void add(Slot slot, double elapsed_ms) {
    if (slot < 0 || slot >= SlotCount) return;
    g_slots[slot] += elapsed_ms;
}

void count_step() {
    ++g_steps;
}

} // namespace FxProfile
