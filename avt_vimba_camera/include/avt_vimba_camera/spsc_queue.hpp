#pragma once

#include <atomic>
#include <cstddef>
#include <optional>
#include <vector>

namespace avt_vimba_camera {

/**
 * Lock-free single-producer single-consumer (SPSC) ring buffer.
 *
 * Used to decouple the Vimba frame-received callback (producer) from the
 * ROS publisher thread (consumer) without mutexes. The capacity must be a
 * power of two for efficient modular indexing.
 *
 * Template parameter T should be cheaply movable (e.g., a struct of
 * pointer + metadata, not a large buffer itself).
 */
template <typename T>
class SPSCQueue {
public:
    /**
     * Construct a queue with the given capacity.
     * Capacity is rounded up to the next power of two.
     */
    explicit SPSCQueue(size_t capacity)
        : mask_(next_pow2(capacity) - 1)
        , buffer_(next_pow2(capacity))
        , read_idx_(0)
        , write_idx_(0)
    {}

    /**
     * Try to enqueue an element (producer side).
     *
     * @param item Element to enqueue (moved in on success).
     * @return true if enqueued, false if queue is full.
     */
    bool try_push(T&& item) {
        const size_t w = write_idx_.load(std::memory_order_relaxed);
        const size_t next_w = (w + 1) & mask_;
        // Full when next write position equals read position
        if (next_w == read_idx_.load(std::memory_order_acquire)) {
            return false;
        }
        buffer_[w & mask_] = std::move(item);
        write_idx_.store(next_w, std::memory_order_release);
        return true;
    }

    /**
     * Try to dequeue an element (consumer side).
     *
     * @param item Output: the dequeued element (moved out on success).
     * @return true if dequeued, false if queue is empty.
     */
    bool try_pop(T& item) {
        const size_t r = read_idx_.load(std::memory_order_relaxed);
        if (r == write_idx_.load(std::memory_order_acquire)) {
            return false;  // empty
        }
        item = std::move(buffer_[r & mask_]);
        read_idx_.store((r + 1) & mask_, std::memory_order_release);
        return true;
    }

    /** Check if the queue is empty (approximate, for monitoring). */
    bool empty() const {
        return read_idx_.load(std::memory_order_acquire) ==
               write_idx_.load(std::memory_order_acquire);
    }

private:
    static size_t next_pow2(size_t v) {
        v--;
        v |= v >> 1;
        v |= v >> 2;
        v |= v >> 4;
        v |= v >> 8;
        v |= v >> 16;
        v |= v >> 32;
        return v + 1;
    }

    const size_t mask_;
    std::vector<T> buffer_;

    // Separate cache lines to avoid false sharing
    alignas(64) std::atomic<size_t> read_idx_;
    alignas(64) std::atomic<size_t> write_idx_;
};

}  // namespace avt_vimba_camera
