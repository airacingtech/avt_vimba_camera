#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include <stdexcept>

#include <cuda_runtime.h>

namespace avt_vimba_camera {

/**
 * Pool of CUDA pinned (page-locked) memory buffers.
 *
 * Pre-allocates a fixed number of pinned buffers at construction for
 * zero-copy DMA from the Vimba SDK frame callback into GPU-accessible memory.
 * Each buffer can be passed directly to VmbFrameAnnounce() and is also
 * accessible by CUDA kernels without an explicit memcpy.
 */
class PinnedBufferPool {
public:
    /**
     * Allocate `count` pinned buffers of `buffer_size` bytes each.
     *
     * @param count       Number of buffers to allocate.
     * @param buffer_size Size of each buffer in bytes.
     * @throws std::runtime_error if cudaMallocHost fails.
     */
    PinnedBufferPool(size_t count, size_t buffer_size)
        : buffer_size_(buffer_size)
    {
        buffers_.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            void* ptr = nullptr;
            cudaError_t err = cudaMallocHost(&ptr, buffer_size);
            if (err != cudaSuccess) {
                // Free already-allocated buffers before throwing
                for (auto* p : buffers_) {
                    cudaFreeHost(p);
                }
                buffers_.clear();
                throw std::runtime_error(
                    std::string("cudaMallocHost failed: ") +
                    cudaGetErrorString(err));
            }
            buffers_.push_back(static_cast<uint8_t*>(ptr));
        }
    }

    ~PinnedBufferPool() {
        for (auto* ptr : buffers_) {
            cudaFreeHost(ptr);
        }
        buffers_.clear();
    }

    // Non-copyable, non-movable (owns raw CUDA allocations)
    PinnedBufferPool(const PinnedBufferPool&) = delete;
    PinnedBufferPool& operator=(const PinnedBufferPool&) = delete;
    PinnedBufferPool(PinnedBufferPool&&) = delete;
    PinnedBufferPool& operator=(PinnedBufferPool&&) = delete;

    /** Get pointer to buffer at index `idx`. */
    uint8_t* get_buffer(size_t idx) {
        if (idx >= buffers_.size()) {
            throw std::out_of_range("PinnedBufferPool: index out of range");
        }
        return buffers_[idx];
    }

    const uint8_t* get_buffer(size_t idx) const {
        if (idx >= buffers_.size()) {
            throw std::out_of_range("PinnedBufferPool: index out of range");
        }
        return buffers_[idx];
    }

    /** Number of buffers in the pool. */
    size_t size() const { return buffers_.size(); }

    /** Size of each buffer in bytes. */
    size_t buffer_size() const { return buffer_size_; }

private:
    std::vector<uint8_t*> buffers_;
    size_t buffer_size_;
};

}  // namespace avt_vimba_camera
