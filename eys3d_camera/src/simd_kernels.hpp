// SIMD kernels with compile-time arch dispatch.
//
// aarch64 builds select the NEON intrinsic path; every other target uses
// the portable scalar implementation, which auto-vectorises under -O3. Both
// ABIs mandate their 128-bit SIMD extension (NEON on ARMv8, SSE2 on x86_64),
// so there is no runtime feature detection.

#pragma once

#include <cstddef>
#include <cstdint>

namespace eys3d_camera::simd {

// YUYV (4:2:2 packed: Y0 U Y1 V) → rgb8 (R G B R G B ...).
//
// BT.601 limited-range coefficients in 8.8 fixed point; both kernels emit
// identical bytes. `src` is `w * h * 2` bytes, `dst` is `w * h * 3`. `w` must
// be even (YUYV invariant); `h` >= 1. OpenMP parallelises across rows.
void yuyv_to_rgb8(const uint8_t* src, uint8_t* dst, int w, int h);

// Split-aware variant for wide-color L|R modes (G100+ 2560x720 wide YUYV,
// modes 22 / 25 / 26). The source raster is `(2 * half_w) * h * 2` bytes;
// each row's first `half_w` YUYV pairs go to `dst_left`, the next `half_w`
// to `dst_right`. Both outputs are `half_w * h * 3` bytes, identical to
// `yuyv_to_rgb8` followed by an in-order row slice.
void yuyv_to_rgb8_split(const uint8_t* src,
                        uint8_t* dst_left, uint8_t* dst_right,
                        int half_w, int h);

// ---------------------------------------------------------------------------
//   Monochrome fast path
// ---------------------------------------------------------------------------
// The G62 / R77 sensors are monochrome; their color stream carries luma
// only. Decoding straight to a single gray plane (MJPEG via TJPF_GRAY, or
// the Y bytes of YUYV) skips chroma upsampling and the YCbCr->RGB matrix,
// then a cheap replicate yields an exact-gray rgb8 frame (R == G == B).

// Extract the Y plane from a YUYV raster: gray[i] = src[2*i]. `src` is
// `w * h * 2` bytes, `gray` is `w * h` bytes.
void yuyv_extract_y(const uint8_t* src, uint8_t* gray, int w, int h);

// Replicate a `w * h` gray plane to rgb8 (`w * h * 3`), R = G = B = gray.
void gray_to_rgb8(const uint8_t* gray, uint8_t* dst, int w, int h);

// Split-aware replicate for wide L|R mono modes. `gray` is the wide plane
// (`(2 * half_w) * h` bytes); each row's first `half_w` grays go to
// `dst_left`, the next `half_w` to `dst_right` (each `half_w * h * 3`).
void gray_to_rgb8_split(const uint8_t* gray,
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
