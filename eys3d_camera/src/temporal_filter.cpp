#include "temporal_filter.hpp"

#include <cstdlib>

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef __aarch64__
#include <arm_neon.h>
#endif

namespace eys3d_camera {

namespace {

constexpr uint16_t kHole = 0;

inline uint16_t mix_q8(int prev, int cur, int alpha_q8) {
    const int v = (alpha_q8 * cur + (256 - alpha_q8) * prev) >> 8;
    return static_cast<uint16_t>(v);
}

// Returns true when the rolling 8-frame validity history hist satisfies
// the persistence pattern at the given level (1..7). Bit 0 holds the
// current frame's validity (most recent), bit 7 the oldest tracked
// frame. Levels 0 (disabled) and 8 (indefinite) are handled at the
// call site because they do not depend on the bitmask.
inline bool persistence_pattern_matches(uint8_t hist, int level) {
    switch (level) {
        case 1: return __builtin_popcount(hist) == 8;            // 8/8
        case 2: return __builtin_popcount(hist & 0x07) >= 2;     // 2/last 3
        case 3: return __builtin_popcount(hist & 0x0F) >= 2;     // 2/last 4
        case 4: return __builtin_popcount(hist) >= 2;            // 2/8
        case 5: return (hist & 0x03) != 0;                       // 1/last 2
        case 6: return (hist & 0x1F) != 0;                       // 1/last 5
        case 7: return hist != 0;                                // 1/8
        default: return false;
    }
}

// All NEON-eligible persistence levels (1..7) collapse to the same form:
//   popcount(hist & mask) >= threshold
// Pre-resolved from the runtime level so the inner loop carries no switch.
struct PersistencePattern {
    int     mode = 0;        // 0 = disabled, 1 = pattern, 2 = indefinite
    uint8_t mask = 0;
    uint8_t threshold = 0;
};

inline PersistencePattern resolve_persistence(int level) {
    PersistencePattern p;
    switch (level) {
        case 1: p.mode = 1; p.mask = 0xFF; p.threshold = 8; break;
        case 2: p.mode = 1; p.mask = 0x07; p.threshold = 2; break;
        case 3: p.mode = 1; p.mask = 0x0F; p.threshold = 2; break;
        case 4: p.mode = 1; p.mask = 0xFF; p.threshold = 2; break;
        case 5: p.mode = 1; p.mask = 0x03; p.threshold = 1; break;
        case 6: p.mode = 1; p.mask = 0x1F; p.threshold = 1; break;
        case 7: p.mode = 1; p.mask = 0xFF; p.threshold = 1; break;
        case 8: p.mode = 2; break;
        default: p.mode = 0; break;
    }
    return p;
}

#ifdef __aarch64__
inline uint16x8_t mix_q8_neon(uint16x8_t prev, uint16x8_t cur,
                              uint16x8_t alpha_v, uint16x8_t inv_alpha_v) {
    uint32x4_t lo = vmull_u16(vget_low_u16(alpha_v),  vget_low_u16(cur));
    uint32x4_t hi = vmull_u16(vget_high_u16(alpha_v), vget_high_u16(cur));
    lo = vmlal_u16(lo, vget_low_u16(inv_alpha_v),  vget_low_u16(prev));
    hi = vmlal_u16(hi, vget_high_u16(inv_alpha_v), vget_high_u16(prev));
    return vcombine_u16(vshrn_n_u32(lo, 8), vshrn_n_u32(hi, 8));
}
#endif

}  // namespace

void temporal_filter_apply(uint16_t* __restrict__ cur, int w, int h,
                           TemporalState& state,
                           const TemporalFilterParams& params)
{
    const int level = params.persistence;
    const int alpha = params.alpha_q8;
    const int delta = params.delta;

    uint16_t* __restrict__ const prev_buf = state.prev.data();
    uint8_t*  __restrict__ const hist_buf = state.valid_history.data();

#ifdef __aarch64__
    const PersistencePattern pp  = resolve_persistence(level);
    const uint16x8_t alpha_v     = vdupq_n_u16(static_cast<uint16_t>(alpha));
    const uint16x8_t inv_alpha_v = vdupq_n_u16(static_cast<uint16_t>(256 - alpha));
    const uint16x8_t delta_v     = vdupq_n_u16(static_cast<uint16_t>(delta));
    const uint16x8_t zero_v      = vdupq_n_u16(0);
    const uint16x8_t one_v       = vdupq_n_u16(1);
    const uint16x8_t mask_v      = vdupq_n_u16(static_cast<uint16_t>(pp.mask));
    const uint16x8_t threshold_v = vdupq_n_u16(static_cast<uint16_t>(pp.threshold));
    const int w_neon = (w / 8) * 8;
#endif

    #pragma omp parallel for schedule(static)
    for (int v = 0; v < h; ++v) {
        const size_t row = static_cast<size_t>(v) * w;
        uint16_t* __restrict__ cur_row  = cur   + row;
        uint16_t* __restrict__ prev_row = prev_buf + row;
        uint8_t*  __restrict__ hist_row = hist_buf + row;

        int u = 0;
#ifdef __aarch64__
        // 8-lane NEON main loop. Every per-pixel branch from the scalar
        // version (cur valid / cur hole, blend / no blend, persistence
        // mode 0/1/2) maps to a predicated select via vbslq_u16, so the
        // inner body carries no data-dependent branches.
        for (; u < w_neon; u += 8) {
            const uint16x8_t cur  = vld1q_u16(cur_row  + u);
            const uint16x8_t prev = vld1q_u16(prev_row + u);
            const uint16x8_t hist_prev = vmovl_u8(vld1_u8(hist_row + u));

            const uint16x8_t cur_is_hole  = vceqq_u16(cur, zero_v);
            const uint16x8_t cur_is_valid = vmvnq_u16(cur_is_hole);
            const uint16x8_t prev_is_valid = vmvnq_u16(vceqq_u16(prev, zero_v));

            // Shift hist left and inject current frame's validity in bit 0.
            // The low byte (max 0xFE) is what eventually gets stored.
            // hist_prev keeps the pre-shift history; persistence tests
            // run against it so "last N frames" means the N frames
            // before the current hole, never the hole itself (which
            // would make level 1 / 8-of-8 unreachable when the current
            // pixel is a hole).
            const uint16x8_t hist_new = vorrq_u16(
                vshlq_n_u16(hist_prev, 1),
                vandq_u16(cur_is_valid, one_v));
            vst1_u8(hist_row + u, vmovn_u16(hist_new));

            // Valid path: blend if prev valid and gap below delta.
            const uint16x8_t abs_diff     = vabdq_u16(cur, prev);
            const uint16x8_t within_delta = vcltq_u16(abs_diff, delta_v);
            const uint16x8_t do_blend = vandq_u16(
                vandq_u16(cur_is_valid, prev_is_valid), within_delta);
            const uint16x8_t mixed = mix_q8_neon(prev, cur, alpha_v, inv_alpha_v);
            const uint16x8_t valid_out = vbslq_u16(do_blend, mixed, cur);

            // Hole path: persistence mode resolves to one of three
            // pre-computed shapes. Tests against hist_prev so the
            // window covers the N frames prior to this hole.
            uint16x8_t hole_out = cur;       // = 0 for hole, leaves as hole
            if (pp.mode == 1) {
                // popcount(hist_prev & mask) >= threshold
                const uint16x8_t masked = vandq_u16(hist_prev, mask_v);
                const uint8x16_t  bytes = vreinterpretq_u8_u16(masked);
                const uint8x16_t  popcnt8  = vcntq_u8(bytes);
                const uint16x8_t  popcnt16 = vpaddlq_u8(popcnt8);
                const uint16x8_t  fill = vandq_u16(
                    vcgeq_u16(popcnt16, threshold_v), prev_is_valid);
                hole_out = vbslq_u16(fill, prev, cur);
            } else if (pp.mode == 2) {
                // Indefinite: fill as long as prev exists.
                hole_out = vbslq_u16(prev_is_valid, prev, cur);
            }
            // pp.mode == 0: persistence disabled, hole_out stays cur (0).

            const uint16x8_t out = vbslq_u16(cur_is_hole, hole_out, valid_out);
            vst1q_u16(cur_row + u, out);

            // prev: valid lanes adopt the output; hole lanes keep prev.
            const uint16x8_t new_prev = vbslq_u16(cur_is_valid, out, prev);
            vst1q_u16(prev_row + u, new_prev);
        }
#endif
        // Scalar tail (also the entire row on non-aarch64 builds).
        for (; u < w; ++u) {
            const uint16_t cur  = cur_row[u];
            const uint16_t prev = prev_row[u];

            // Preserve the pre-shift history; the persistence test
            // below uses it so the window covers the N frames before
            // this hole. Testing against the post-shift value would
            // make level 1 (8/8) unreachable on a hole because bit 0
            // is forced to 0 when cur is a hole.
            const uint8_t hist_prev = hist_row[u];
            const uint8_t hist = static_cast<uint8_t>(
                (hist_prev << 1) | (cur != kHole ? 1u : 0u));
            hist_row[u] = hist;

            if (cur != kHole) {
                uint16_t out = cur;
                if (prev != kHole
                    && std::abs(static_cast<int>(cur) - static_cast<int>(prev)) < delta) {
                    out = mix_q8(prev, cur, alpha);
                }
                cur_row[u]  = out;
                prev_row[u] = out;
                continue;
            }

            if (level == 0) continue;

            const bool fill = (level == 8)
                ? (prev != kHole)
                : persistence_pattern_matches(hist_prev, level);
            if (fill) {
                // Re-emit the previous output; do not refresh prev_row,
                // because the source pixel for this frame was invalid
                // and the history bit (0) already records that.
                cur_row[u] = prev;
            }
        }
    }
}

}  // namespace eys3d_camera
