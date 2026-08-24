#include "Fx2D/Entity.h"
#include "Fx2D/Solver.h"

#include <algorithm>

namespace {

constexpr uint32_t kUncolored = 0xffffffffu;

inline bool test_bit(const std::vector<uint64_t>& bits, size_t base, size_t body) {
    return (bits[base + (body >> 6)] & (uint64_t{1} << (body & 63))) != 0;
}

inline void set_bit(std::vector<uint64_t>& bits, size_t base, size_t body) {
    bits[base + (body >> 6)] |= (uint64_t{1} << (body & 63));
}

} // namespace

void FxContactGraph::color_pairs(const std::vector<std::pair<size_t, size_t>>& pairs,
                                 const std::vector<std::shared_ptr<FxEntity>>& entities) {
    const size_t pair_count = pairs.size();
    const size_t body_count = entities.size();

    m_group_starts.clear();
    m_colored_count = 0;
    m_overflow_count = 0;
    m_has_overflow = false;
    if (pair_count == 0 || body_count == 0) return;

    // A body constrains the coloring only if an impulse can move it. Sleep state is ignored on
    // purpose: a sleeper can be woken partway through the step, and treating it as movable is
    // the conservative direction -- it can only split colors that could have merged.
    m_movable.resize(body_count);
    for (size_t i = 0; i < body_count; ++i) {
        const FxEntity& e = *entities[i];
        m_movable[i] = (e.inv_mass() != 0.0f || e.inv_inertia() != 0.0f) ? 1u : 0u;
    }

    const size_t words = (body_count + 63) / 64;
    m_used.assign(kMaxColors * words, 0);
    m_color_of.assign(pair_count, kUncolored);
    m_counts.assign(kMaxColors + 1, 0);

    // Greedy in pair order: first free color wins. Deterministic by construction.
    for (size_t i = 0; i < pair_count; ++i) {
        const size_t a = pairs[i].first;
        const size_t b = pairs[i].second;
        const bool a_movable = m_movable[a] != 0;
        const bool b_movable = m_movable[b] != 0;

        uint32_t color = kMaxColors;
        for (uint32_t candidate = 0; candidate < kMaxColors; ++candidate) {
            const size_t base = static_cast<size_t>(candidate) * words;
            if (a_movable && test_bit(m_used, base, a)) continue;
            if (b_movable && test_bit(m_used, base, b)) continue;
            color = candidate;
            break;
        }

        if (color < kMaxColors) {
            const size_t base = static_cast<size_t>(color) * words;
            if (a_movable) set_bit(m_used, base, a);
            if (b_movable) set_bit(m_used, base, b);
            ++m_colored_count;
        } else {
            // No free color. Solved one at a time in the overflow group, never batched.
            ++m_overflow_count;
            m_has_overflow = true;
        }
        m_color_of[i] = color;
        ++m_counts[color];
    }

    // group_contacts decides the groups, because only it knows which colors actually produced
    // contacts: the pair list is a superset, and a color whose pairs all missed must not leave
    // an empty group behind.
}

void FxContactGraph::group_contacts(const std::vector<uint32_t>& contact_colors,
                                    std::vector<uint32_t>& order) {
    order.clear();
    m_group_starts.clear();
    m_overflow_group = kUncolored;
    if (contact_colors.empty()) return;

    // Counting sort of contact *indices* -- four bytes each, against the whole of an
    // FxContact. Sorting the contacts themselves was measured and cost more than the regrouping
    // saved; so was reordering the pair list, which threw away the narrow phase locality.
    m_group_counts.assign(kMaxColors + 1, 0);
    for (uint32_t color : contact_colors) {
        if (color <= kMaxColors) ++m_group_counts[color];
    }

    // Colors that produced no contact collapse away, so a group is never empty -- a batched
    // solve would otherwise be handed empty work, and the overflow group would be hard to find.
    m_group_of_color.assign(kMaxColors + 1, kUncolored);
    m_group_starts.push_back(0);
    uint32_t running = 0;
    for (uint32_t color = 0; color <= kMaxColors; ++color) {
        if (m_group_counts[color] == 0) continue;
        const uint32_t group = static_cast<uint32_t>(m_group_starts.size() - 1);
        m_group_of_color[color] = group;
        if (color == kMaxColors) m_overflow_group = group;
        running += m_group_counts[color];
        m_group_starts.push_back(running);
    }

    // Reuse the counts as write cursors.
    uint32_t cursor = 0;
    for (uint32_t color = 0; color <= kMaxColors; ++color) {
        const uint32_t count = m_group_counts[color];
        m_group_counts[color] = cursor;
        cursor += count;
    }

    order.resize(running);
    for (uint32_t i = 0; i < contact_colors.size(); ++i) {
        const uint32_t color = contact_colors[i];
        if (color <= kMaxColors) order[m_group_counts[color]++] = i;
    }
}
