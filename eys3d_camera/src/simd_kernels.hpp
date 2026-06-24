// SIMD kernels with compile-time arch dispatch.
//
// aarch64 builds select the NEON intrinsic path; every other target uses
// the portable scalar implementation. Both ABIs mandate their respective
// 128-bit SIMD extensions (NEON on ARMv8, SSE2 on x86_64), so no runtime
// feature detection is needed. A dedicated SSE2 path is not provided
// because the scalar loops auto-vectorise to SSE2/AVX2 under GCC -O3.

#pragma once

#include <cstddef>
#include <cstdint>

namespace eys3d_camera::simd {

// YUYV (4:2:2 packed: Y0 U Y1 V) → rgb8 (R G B R G B ...).
//
// BT.601 limited-range coefficients in 8.8 fixed point. The aarch64 NEON
// kernel processes 32 pixels per iteration (vld4q_u8 + vst3q_u8); the
// scalar kernel auto-vectorises to SSE2/AVX2 under GCC -O3 on x86_64.
// Both kernels produce identical output bytes.
//
// `src` is `w * h * 2` bytes (YUYV); `dst` is `w * h * 3` bytes (rgb8).
// `w` must be even (YUYV invariant); `h` ≥ 1. OpenMP parallelises across
// rows when available.
void yuyv_to_rgb8(const uint8_t* src, uint8_t* dst, int w, int h);

// Split-aware variant for wide-color L|R modes (G100+ 2560x720 wide
// YUYV, modes 22 / 25 / 26). The source raster is `(2 * half_w) * h * 2`
// bytes; each row's first `half_w` YUYV pairs are converted to
// `dst_left`, the next `half_w` pairs to `dst_right`. Both output
// buffers are `half_w * h * 3` bytes.
//
// Compared with "convert wide → memcpy split", this avoids the wide
// rgb8 intermediate (~5.5 MB at 2560x720) and the two row-by-row memcpy
// passes (~10.8 MB DRAM I/O per frame). Output bytes are identical to
// `yuyv_to_rgb8` followed by an in-order row slice.
void yuyv_to_rgb8_split(const uint8_t* src,
                        uint8_t* dst_left, uint8_t* dst_right,
                        int half_w, int h);

// Count non-zero Z14 depth samples in a row. Forms the first pass of
// the point-cloud reprojector, providing each row's contiguous output
// slot size. The high 2 bits of every Z14 sample carry status flags
// and are masked off before the test against zero. Out-of-range
// pixels arrive as 0; this kernel does not enforce the depth range
// itself.
//
// `row` points to `w` consecutive uint16 depth samples; the function
// returns the count of surviving samples.
uint32_t pc_count_nonzero(const uint16_t* row, int w);

}  // namespace eys3d_camera::simd
