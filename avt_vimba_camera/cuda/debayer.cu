/**
 * Malvar-He-Cutler high-quality demosaicing CUDA kernel.
 *
 * Input:  uint8 BayerRG8 image in pinned (page-locked) memory.
 * Output: uint8 RGB HWC -- for ROS sensor_msgs/Image publication
 *
 * Uses shared memory tiling with a 5x5 kernel halo for the Malvar-He-Cutler
 * interpolation filters. Image borders are handled with clamping.
 *
 * Reference: H. S. Malvar, L. He, R. Cutler, "High-quality linear
 * interpolation for demosaicing of Bayer-patterned color images," ICASSP 2004.
 */

#include <cstdint>
#include <cuda_runtime.h>

namespace avt_vimba_camera {

// ---------------------------------------------------------------------------
// Malvar-He-Cutler filter coefficients (scaled by 8 for integer arithmetic)
// These are the 5x5 filter kernels from the paper, multiplied by 8.
// ---------------------------------------------------------------------------

// Filter to get Green at Red/Blue locations
// [0  0 -1  0  0]
// [0  0  2  0  0]
// [-1 2  4  2 -1]
// [0  0  2  0  0]
// [0  0 -1  0  0]
// Divided by 8

// Filter to get Red at Green in Red row / Blue at Green in Blue row
// [0  0  0.5 0  0]
// [0 -1  0  -1  0]
// [-1  4  5   4 -1]
// [0 -1  0  -1  0]
// [0  0  0.5 0  0]
// Divided by 8

// Filter to get Red at Green in Blue row / Blue at Green in Red row
// (transpose of above)

// Filter to get Red at Blue / Blue at Red
// [0  0 -1.5 0  0]
// [0  2  0   2  0]
// [-1.5 0 6  0 -1.5]
// [0  2  0   2  0]
// [0  0 -1.5 0  0]
// Divided by 8

// Tile dimensions (excluding halo)
constexpr int TILE_W = 16;
constexpr int TILE_H = 16;
constexpr int HALO   = 2;  // 5x5 kernel needs 2-pixel halo
constexpr int SMEM_W = TILE_W + 2 * HALO;
constexpr int SMEM_H = TILE_H + 2 * HALO;

/**
 * Clamp coordinate to valid image range.
 */
__device__ __forceinline__ int clamp_coord(int v, int max_val) {
    return min(max(v, 0), max_val - 1);
}

/**
 * Load a tile of BayerRG8 data into shared memory with halo, using clamping.
 */
__device__ void load_tile_to_smem(
    const uint8_t* __restrict__ bayer,
    uint8_t smem[SMEM_H][SMEM_W],
    int tile_x, int tile_y,
    int width, int height,
    int tx, int ty)
{
    // Each thread loads multiple pixels to fill the shared memory tile
    for (int dy = ty; dy < SMEM_H; dy += TILE_H) {
        for (int dx = tx; dx < SMEM_W; dx += TILE_W) {
            int gx = clamp_coord(tile_x - HALO + dx, width);
            int gy = clamp_coord(tile_y - HALO + dy, height);
            smem[dy][dx] = bayer[gy * width + gx];
        }
    }
}

/**
 * Main demosaicing kernel for BayerRG8 pattern.
 *
 * BayerRG layout (starting at top-left):
 *   R G R G ...
 *   G B G B ...
 *   R G R G ...
 *   ...
 *
 * Each thread computes one output pixel's RGB values using the
 * Malvar-He-Cutler interpolation filters.
 *
 * Only produces uint8 RGB HWC output. The CHW float32 path has been removed
 * to avoid dead computation (can be re-added when the GPU-direct perception
 * pipeline is wired up).
 */
__global__ void debayer_malvar_kernel(
    const uint8_t* __restrict__ bayer_in,
    uint8_t* __restrict__ rgb_out,        // HWC uint8 [H, W, 3]
    int width,
    int height)
{
    __shared__ uint8_t smem[SMEM_H][SMEM_W];

    const int tx = threadIdx.x;
    const int ty = threadIdx.y;
    const int tile_x = blockIdx.x * TILE_W;
    const int tile_y = blockIdx.y * TILE_H;

    // Load tile with halo into shared memory
    load_tile_to_smem(bayer_in, smem, tile_x, tile_y, width, height, tx, ty);
    __syncthreads();

    // Global pixel coordinates
    const int gx = tile_x + tx;
    const int gy = tile_y + ty;

    if (gx >= width || gy >= height) return;

    // Shared memory coordinates (with halo offset)
    const int sx = tx + HALO;
    const int sy = ty + HALO;

    // Determine Bayer pattern position
    // BayerRG: (0,0)=R, (1,0)=G_R, (0,1)=G_B, (1,1)=B
    const int is_red_col  = (gx & 1) == 0;
    const int is_red_row  = (gy & 1) == 0;

    float r, g, b;

    // Helper: load from shared memory
    #define S(dx, dy) ((float)smem[sy + (dy)][sx + (dx)])

    if (is_red_row && is_red_col) {
        // Red pixel location
        r = S(0, 0);

        // Green at Red: Malvar filter
        g = ( 4.0f * S(0,0)
            + 2.0f * (S(-1,0) + S(1,0) + S(0,-1) + S(0,1))
            - 1.0f * (S(-2,0) + S(2,0) + S(0,-2) + S(0,2))
            ) / 8.0f;

        // Blue at Red: Malvar filter
        b = ( 6.0f * S(0,0)
            + 2.0f * (S(-1,-1) + S(1,-1) + S(-1,1) + S(1,1))
            - 1.5f * (S(-2,0) + S(2,0) + S(0,-2) + S(0,2))
            ) / 8.0f;

    } else if (is_red_row && !is_red_col) {
        // Green pixel in Red row
        g = S(0, 0);

        // Red at Green in Red row
        r = ( 5.0f * S(0,0)
            + 4.0f * (S(-1,0) + S(1,0))
            + 0.5f * (S(0,-2) + S(0,2))
            - 1.0f * (S(-2,0) + S(2,0) + S(-1,-1) + S(1,-1) + S(-1,1) + S(1,1))
            ) / 8.0f;

        // Blue at Green in Red row
        b = ( 5.0f * S(0,0)
            + 4.0f * (S(0,-1) + S(0,1))
            + 0.5f * (S(-2,0) + S(2,0))
            - 1.0f * (S(0,-2) + S(0,2) + S(-1,-1) + S(1,-1) + S(-1,1) + S(1,1))
            ) / 8.0f;

    } else if (!is_red_row && is_red_col) {
        // Green pixel in Blue row
        g = S(0, 0);

        // Red at Green in Blue row (transposed filter)
        r = ( 5.0f * S(0,0)
            + 4.0f * (S(0,-1) + S(0,1))
            + 0.5f * (S(-2,0) + S(2,0))
            - 1.0f * (S(0,-2) + S(0,2) + S(-1,-1) + S(1,-1) + S(-1,1) + S(1,1))
            ) / 8.0f;

        // Blue at Green in Blue row
        b = ( 5.0f * S(0,0)
            + 4.0f * (S(-1,0) + S(1,0))
            + 0.5f * (S(0,-2) + S(0,2))
            - 1.0f * (S(-2,0) + S(2,0) + S(-1,-1) + S(1,-1) + S(-1,1) + S(1,1))
            ) / 8.0f;

    } else {
        // Blue pixel location
        b = S(0, 0);

        // Green at Blue: same filter as Green at Red
        g = ( 4.0f * S(0,0)
            + 2.0f * (S(-1,0) + S(1,0) + S(0,-1) + S(0,1))
            - 1.0f * (S(-2,0) + S(2,0) + S(0,-2) + S(0,2))
            ) / 8.0f;

        // Red at Blue: Malvar filter
        r = ( 6.0f * S(0,0)
            + 2.0f * (S(-1,-1) + S(1,-1) + S(-1,1) + S(1,1))
            - 1.5f * (S(-2,0) + S(2,0) + S(0,-2) + S(0,2))
            ) / 8.0f;
    }

    #undef S

    // Clamp to [0, 255]
    r = fminf(fmaxf(r, 0.0f), 255.0f);
    g = fminf(fmaxf(g, 0.0f), 255.0f);
    b = fminf(fmaxf(b, 0.0f), 255.0f);

    // Write RGB uint8 output (HWC layout)
    const int rgb_idx = (gy * width + gx) * 3;
    rgb_out[rgb_idx + 0] = static_cast<uint8_t>(r);
    rgb_out[rgb_idx + 1] = static_cast<uint8_t>(g);
    rgb_out[rgb_idx + 2] = static_cast<uint8_t>(b);
}


// ---------------------------------------------------------------------------
// Host wrapper
// ---------------------------------------------------------------------------

/**
 * Launch the Malvar-He-Cutler debayer kernel.
 *
 * @param bayer_in    Pointer to BayerRG8 input (pinned host or device memory).
 * @param rgb_out     Pointer to uint8 RGB HWC output (device memory), size: W*H*3.
 * @param width       Image width in pixels.
 * @param height      Image height in pixels.
 * @param stream      CUDA stream for async execution (0 for default).
 */
void cuda_debayer(
    const uint8_t* bayer_in,
    uint8_t* rgb_out,
    int width,
    int height,
    cudaStream_t stream)
{
    dim3 block(TILE_W, TILE_H);
    dim3 grid(
        (width  + TILE_W - 1) / TILE_W,
        (height + TILE_H - 1) / TILE_H
    );

    debayer_malvar_kernel<<<grid, block, 0, stream>>>(
        bayer_in, rgb_out, width, height);
}

}  // namespace avt_vimba_camera
