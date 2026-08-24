#pragma once

#include <cstddef>

// Per-phase step profiler. The counters always exist so tools can link, but are only written
// when built with -DFX2D_PROFILE=ON; otherwise FX2D_PROF_SCOPE expands to nothing. Phases are
// inclusive and partition one step, summing to just under StepTotal.
namespace FxProfile {

enum Slot : int {
    BroadPhase = 0, // tree sync + pair query
    NarrowPhase, // collision_check / speculative_contact_check, warm-start cache lookup
    PositionSolve, // resolve_penetration
    Constraints, // FxConstraint::resolve over the constraint registry
    Integration, // entity->step(): forces, pose integration, shape world-pose refresh
    VelocityDerive, // (pose - prev_pose) / dt
    VelocityPasses, // init_velocity_pass, warm_start, resolve_velocities sweeps
    Bookkeeping, // impulse cache write-back, cache eviction, sleep timers, events
    StepTotal, // the whole of FxScene::step
    SlotCount
};

// True when the library was compiled with the profiler enabled.
bool enabled();

// Human-readable name for a slot, for report tables.
const char* slot_name(Slot slot);

// Accumulated milliseconds in a slot since the last reset().
double ms(Slot slot);

// Number of completed steps since the last reset(), so callers can report per-step figures.
std::size_t steps();

// Zero every accumulator and the step count. Call between measured runs.
void reset();

// Adds `elapsed_ms` to a slot. Called by the scope timer; public so tools can fold in their own
// measurements if they ever need to.
void add(Slot slot, double elapsed_ms);

// Marks one completed step.
void count_step();

} // namespace FxProfile

#ifdef FX2D_PROFILE
#include <chrono>

namespace FxProfile {
// Accumulates its lifetime into a slot. Scope-based so an early return still records.
class ScopeTimer {
  public:
    explicit ScopeTimer(Slot slot) : m_slot(slot), m_start(std::chrono::steady_clock::now()) {}
    ~ScopeTimer() {
        add(m_slot,
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - m_start)
                .count());
    }
    ScopeTimer(const ScopeTimer&) = delete;
    ScopeTimer& operator=(const ScopeTimer&) = delete;

  private:
    Slot m_slot;
    std::chrono::steady_clock::time_point m_start;
};
} // namespace FxProfile

#define FX2D_PROF_CONCAT_INNER(a, b) a##b
#define FX2D_PROF_CONCAT(a, b) FX2D_PROF_CONCAT_INNER(a, b)
#define FX2D_PROF_SCOPE(slot) \
    ::FxProfile::ScopeTimer FX2D_PROF_CONCAT(fx2d_prof_timer_, __LINE__)(::FxProfile::slot)
#define FX2D_PROF_STEP() ::FxProfile::count_step()
#else
#define FX2D_PROF_SCOPE(slot) ((void)0)
#define FX2D_PROF_STEP() ((void)0)
#endif
