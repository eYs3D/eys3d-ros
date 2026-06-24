#include "simd_kernels.hpp"

#include <algorithm>

#ifdef _OPENMP
#include <omp.h>
#endif

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace eys3d_camera::simd {

namespace {

// BT.601 limited-range coefficients, scaled to 8.8 fixed point so the
// inner loop is integer-only.
constexpr int kCoefRV = 359;   //  1.402 * 256
constexpr int kCoefGU = -88;   // -0.344 * 256
constexpr int kCoefGV = -183;  // -0.714 * 256
constexpr int kCoefBU = 454;   //  1.772 * 256

inline uint8_t clamp_u8(int x) {
    return static_cast<uint8_t>(x < 0 ? 0 : (x > 255 ? 255 : x));
}

// Portable scalar row converter. The +128 bias is applied to (Y << 8)
// so the final >> 8 is symmetric round-to-nearest. GCC -O3 will lift
// this to SSE2/AVX2 on x86_64. Serves as both the reference
// implementation and the fallback for non-aarch64 targets.
inline void yuyv_to_rgb8_row_scalar(const uint8_t* __restrict src,
                                    uint8_t* __restrict dst, int pairs) {
    for (int i = 0; i < pairs; ++i) {
        const int y0 = (static_cast<int>(src[0]) << 8) + 128;
        const int u  = static_cast<int>(src[1]) - 128;
        const int y1 = (static_cast<int>(src[2]) << 8) + 128;
        const int v  = static_cast<int>(src[3]) - 128;
        const int r_off =  kCoefRV * v;
        const int g_off =  kCoefGU * u + kCoefGV * v;
        const int b_off =  kCoefBU * u;
        dst[0] = clamp_u8((y0 + r_off) >> 8);
        dst[1] = clamp_u8((y0 + g_off) >> 8);
        dst[2] = clamp_u8((y0 + b_off) >> 8);
        dst[3] = clamp_u8((y1 + r_off) >> 8);
        dst[4] = clamp_u8((y1 + g_off) >> 8);
        dst[5] = clamp_u8((y1 + b_off) >> 8);
        src += 4;
        dst += 6;
    }
}

#if defined(__aarch64__)

// AArch64 NEON row converter. One iteration consumes 64 bytes (16
// YUYV pairs = 32 pixels) and emits 96 bytes (32 rgb8 triplets).
//
// vld4q_u8 de-interleaves YUYV into four uint8x16 lanes:
//   val[0] = Y at even positions, val[1] = U,
//   val[2] = Y at odd positions,  val[3] = V.
//
// The conversion is:
//   result = clamp_u8(Y + round(chroma_offset / 256))
// which is integer-exact equivalent to the textbook
//   result = clamp_u8(round((Y << 8 + chroma_offset) / 256))
// The chroma product runs in int32 and is rounding-narrowed to int16
// before adding to Y (Y itself fits in int16). Every intermediate
// stays within representable range across the full input domain.
//
// Supported video modes use row widths that are multiples of 32
// pixels, so the scalar tail path is not exercised in practice.
inline void yuyv_to_rgb8_row_neon(const uint8_t* __restrict src,
                                  uint8_t* __restrict dst, int pairs) {
    const int32x4_t kRV32 = vdupq_n_s32(kCoefRV);
    const int32x4_t kGU32 = vdupq_n_s32(kCoefGU);
    const int32x4_t kGV32 = vdupq_n_s32(kCoefGV);
    const int32x4_t kBU32 = vdupq_n_s32(kCoefBU);
    const uint8x16_t k128_u8 = vdupq_n_u8(128);

    int i = 0;
    for (; i + 16 <= pairs; i += 16) {
        const uint8x16x4_t yuyv = vld4q_u8(src);

        // Chroma minus 128 as signed 8-bit (wrap through uint8, then
        // reinterpret), widened to int16x8 per half.
        const int8x16_t u_i8 = vreinterpretq_s8_u8(vsubq_u8(yuyv.val[1], k128_u8));
        const int8x16_t v_i8 = vreinterpretq_s8_u8(vsubq_u8(yuyv.val[3], k128_u8));
        const int16x8_t u_lo = vmovl_s8(vget_low_s8(u_i8));
        const int16x8_t u_hi = vmovl_s8(vget_high_s8(u_i8));
        const int16x8_t v_lo = vmovl_s8(vget_low_s8(v_i8));
        const int16x8_t v_hi = vmovl_s8(vget_high_s8(v_i8));

        // Further widen each chroma plane to four int32x4 chunks so the
        // coefficient multiply (|prod| ≤ 128 × 454 = 58112) is exact.
        const int32x4_t u_ll = vmovl_s16(vget_low_s16(u_lo));
        const int32x4_t u_lh = vmovl_s16(vget_high_s16(u_lo));
        const int32x4_t u_hl = vmovl_s16(vget_low_s16(u_hi));
        const int32x4_t u_hh = vmovl_s16(vget_high_s16(u_hi));
        const int32x4_t v_ll = vmovl_s16(vget_low_s16(v_lo));
        const int32x4_t v_lh = vmovl_s16(vget_high_s16(v_lo));
        const int32x4_t v_hl = vmovl_s16(vget_low_s16(v_hi));
        const int32x4_t v_hh = vmovl_s16(vget_high_s16(v_hi));

        // Compute (coef × chroma) in int32, then rounding-narrow back
        // to int16. After >> 8 the magnitude is at most ~230, safely in
        // range for the subsequent int16 add against Y.
        auto pack_offset = [](int32x4_t a, int32x4_t b) -> int16x8_t {
            return vcombine_s16(vrshrn_n_s32(a, 8),
                                vrshrn_n_s32(b, 8));
        };
        const int16x8_t r_off_lo = pack_offset(vmulq_s32(v_ll, kRV32),
                                               vmulq_s32(v_lh, kRV32));
        const int16x8_t r_off_hi = pack_offset(vmulq_s32(v_hl, kRV32),
                                               vmulq_s32(v_hh, kRV32));
        const int16x8_t b_off_lo = pack_offset(vmulq_s32(u_ll, kBU32),
                                               vmulq_s32(u_lh, kBU32));
        const int16x8_t b_off_hi = pack_offset(vmulq_s32(u_hl, kBU32),
                                               vmulq_s32(u_hh, kBU32));
        // G uses both U and V coefficients: (U × kGU) + (V × kGV).
        const int16x8_t g_off_lo = pack_offset(
            vmlaq_s32(vmulq_s32(u_ll, kGU32), v_ll, kGV32),
            vmlaq_s32(vmulq_s32(u_lh, kGU32), v_lh, kGV32));
        const int16x8_t g_off_hi = pack_offset(
            vmlaq_s32(vmulq_s32(u_hl, kGU32), v_hl, kGV32),
            vmlaq_s32(vmulq_s32(u_hh, kGU32), v_hh, kGV32));

        // Widen Y to int16 directly. 0–255 fits without overflow.
        const int16x8_t ye_lo = vreinterpretq_s16_u16(vmovl_u8(vget_low_u8(yuyv.val[0])));
        const int16x8_t ye_hi = vreinterpretq_s16_u16(vmovl_u8(vget_high_u8(yuyv.val[0])));
        const int16x8_t yo_lo = vreinterpretq_s16_u16(vmovl_u8(vget_low_u8(yuyv.val[2])));
        const int16x8_t yo_hi = vreinterpretq_s16_u16(vmovl_u8(vget_high_u8(yuyv.val[2])));

        // Y + chroma offset, saturating-narrow to uint8.
        const uint8x8_t re_lo = vqmovun_s16(vaddq_s16(ye_lo, r_off_lo));
        const uint8x8_t re_hi = vqmovun_s16(vaddq_s16(ye_hi, r_off_hi));
        const uint8x8_t ge_lo = vqmovun_s16(vaddq_s16(ye_lo, g_off_lo));
        const uint8x8_t ge_hi = vqmovun_s16(vaddq_s16(ye_hi, g_off_hi));
        const uint8x8_t be_lo = vqmovun_s16(vaddq_s16(ye_lo, b_off_lo));
        const uint8x8_t be_hi = vqmovun_s16(vaddq_s16(ye_hi, b_off_hi));
        const uint8x8_t ro_lo = vqmovun_s16(vaddq_s16(yo_lo, r_off_lo));
        const uint8x8_t ro_hi = vqmovun_s16(vaddq_s16(yo_hi, r_off_hi));
        const uint8x8_t go_lo = vqmovun_s16(vaddq_s16(yo_lo, g_off_lo));
        const uint8x8_t go_hi = vqmovun_s16(vaddq_s16(yo_hi, g_off_hi));
        const uint8x8_t bo_lo = vqmovun_s16(vaddq_s16(yo_lo, b_off_lo));
        const uint8x8_t bo_hi = vqmovun_s16(vaddq_s16(yo_hi, b_off_hi));

        const uint8x16_t r_even = vcombine_u8(re_lo, re_hi);
        const uint8x16_t g_even = vcombine_u8(ge_lo, ge_hi);
        const uint8x16_t b_even = vcombine_u8(be_lo, be_hi);
        const uint8x16_t r_odd  = vcombine_u8(ro_lo, ro_hi);
        const uint8x16_t g_odd  = vcombine_u8(go_lo, go_hi);
        const uint8x16_t b_odd  = vcombine_u8(bo_lo, bo_hi);

        // Interleave even/odd back to pixel order and scatter-store as
        // R G B R G B ...
        uint8x16x3_t rgb0;
        rgb0.val[0] = vzip1q_u8(r_even, r_odd);
        rgb0.val[1] = vzip1q_u8(g_even, g_odd);
        rgb0.val[2] = vzip1q_u8(b_even, b_odd);
        vst3q_u8(dst, rgb0);

        uint8x16x3_t rgb1;
        rgb1.val[0] = vzip2q_u8(r_even, r_odd);
        rgb1.val[1] = vzip2q_u8(g_even, g_odd);
        rgb1.val[2] = vzip2q_u8(b_even, b_odd);
        vst3q_u8(dst + 48, rgb1);

        src += 64;
        dst += 96;
    }

    if (i < pairs) {
        yuyv_to_rgb8_row_scalar(src, dst, pairs - i);
    }
}

#endif  // __aarch64__

}  // namespace

namespace {
// Single-row dispatcher. Picks NEON on aarch64, scalar elsewhere.
inline void yuyv_to_rgb8_row(const uint8_t* __restrict src,
                             uint8_t* __restrict dst, int pairs) {
#if defined(__aarch64__)
    yuyv_to_rgb8_row_neon(src, dst, pairs);
#else
    yuyv_to_rgb8_row_scalar(src, dst, pairs);
#endif
}
}  // namespace

void yuyv_to_rgb8(const uint8_t* src, uint8_t* dst, int w, int h) {
    const int pairs = w / 2;
    const size_t in_stride  = static_cast<size_t>(w) * 2;
    const size_t out_stride = static_cast<size_t>(w) * 3;

#ifdef _OPENMP
    #pragma omp parallel for schedule(static)
#endif
    for (int v = 0; v < h; ++v) {
        const uint8_t* p = src + static_cast<size_t>(v) * in_stride;
        uint8_t* q       = dst + static_cast<size_t>(v) * out_stride;
        yuyv_to_rgb8_row(p, q, pairs);
    }
}

void yuyv_to_rgb8_split(const uint8_t* src,
                        uint8_t* dst_left, uint8_t* dst_right,
                        int half_w, int h) {
    const int half_pairs    = half_w / 2;
    // Wide source stride: 2 * half_w pixels × 2 bytes.
    const size_t in_stride  = static_cast<size_t>(half_w) * 4;
    // Per-side output stride: half_w pixels × 3 bytes.
    const size_t out_stride = static_cast<size_t>(half_w) * 3;
    // Offset of the right half within the wide YUYV row.
    const size_t right_src_off = static_cast<size_t>(half_w) * 2;

#ifdef _OPENMP
    #pragma omp parallel for schedule(static)
#endif
    for (int v = 0; v < h; ++v) {
        const uint8_t* p_row = src + static_cast<size_t>(v) * in_stride;
        uint8_t* ql = dst_left  + static_cast<size_t>(v) * out_stride;
        uint8_t* qr = dst_right + static_cast<size_t>(v) * out_stride;
        yuyv_to_rgb8_row(p_row,                 ql, half_pairs);
        yuyv_to_rgb8_row(p_row + right_src_off, qr, half_pairs);
    }
}

// ---------------------------------------------------------------------------
// pc_count_nonzero — see simd_kernels.hpp for the contract.
// ---------------------------------------------------------------------------

uint32_t pc_count_nonzero(const uint16_t* row, int w) {
    constexpr uint16_t kDepthMask = 0x3FFF;
    uint32_t count = 0;
    int u = 0;

#if defined(__aarch64__)
    const uint16x8_t mask = vdupq_n_u16(kDepthMask);
    const uint16x8_t zero = vdupq_n_u16(0);

    uint16x8_t acc = vdupq_n_u16(0);
    int batch = 0;
    for (; u + 16 <= w; u += 16) {
        const uint16x8_t v0 = vandq_u16(vld1q_u16(row + u),     mask);
        const uint16x8_t v1 = vandq_u16(vld1q_u16(row + u + 8), mask);

        // 0xFFFF on (v != 0), 0 on hole.
        const uint16x8_t m0 = vmvnq_u16(vceqq_u16(v0, zero));
        const uint16x8_t m1 = vmvnq_u16(vceqq_u16(v1, zero));
        acc = vaddq_u16(acc, vshrq_n_u16(m0, 15));
        acc = vaddq_u16(acc, vshrq_n_u16(m1, 15));

        if (++batch == 2048) {
            count += vaddvq_u16(acc);
            acc = vdupq_n_u16(0);
            batch = 0;
        }
    }
    count += vaddvq_u16(acc);
#endif

    for (; u < w; ++u) {
        const uint16_t z = row[u] & kDepthMask;
        count += (z != 0) ? 1u : 0u;
    }
    return count;
}

}  // namespace eys3d_camera::simd
