#pragma once

#include <atomic>
#include <cstddef>
#include <vector>
#include <new>

namespace audio_core {

// SPSC (Single-Producer Single-Consumer) Lock-Free Ring Buffer
// Safe for Real-Time audio threads without locks or allocations.
template <typename T>
class RingBuffer {
public:
    explicit RingBuffer(size_t capacity) {
        // Enforce next power of two
        size_t cap = 2;
        while (cap < capacity) {
            cap <<= 1;
        }
        m_capacity = cap;
        m_mask = cap - 1;
        m_buffer = std::vector<T>(m_capacity);
    }

    ~RingBuffer() = default;

    RingBuffer(const RingBuffer&) = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;
    RingBuffer(RingBuffer&&) = delete;
    RingBuffer& operator=(RingBuffer&&) = delete;

    [[nodiscard]] size_t capacity() const noexcept {
        return m_capacity;
    }

    // Called by Producer thread only
    bool try_push(const T& item) noexcept {
        const size_t current_tail = m_tail.load(std::memory_order_relaxed);
        const size_t current_head = m_head.load(std::memory_order_acquire);

        if ((current_tail - current_head) >= m_capacity) {
            return false; // Buffer full
        }

        m_buffer[current_tail & m_mask] = item;
        m_tail.store(current_tail + 1, std::memory_order_release);
        return true;
    }

    // Called by Consumer thread only
    bool try_pop(T& item) noexcept {
        const size_t current_head = m_head.load(std::memory_order_relaxed);
        const size_t current_tail = m_tail.load(std::memory_order_acquire);

        if (current_head == current_tail) {
            return false; // Buffer empty
        }

        item = m_buffer[current_head & m_mask];
        m_head.store(current_head + 1, std::memory_order_release);
        return true;
    }

    [[nodiscard]] size_t size() const noexcept {
        const size_t head = m_head.load(std::memory_order_relaxed);
        const size_t tail = m_tail.load(std::memory_order_relaxed);
        return (tail >= head) ? (tail - head) : 0;
    }

    [[nodiscard]] size_t available_read() const noexcept {
        return size();
    }

    [[nodiscard]] size_t available_write() const noexcept {
        const size_t s = size();
        return (s < m_capacity) ? (m_capacity - s) : 0;
    }

    bool push(const T& item) noexcept {
        return try_push(item);
    }

    bool pop(T& item) noexcept {
        return try_pop(item);
    }

    void clear() noexcept {
        const size_t tail = m_tail.load(std::memory_order_relaxed);
        m_head.store(tail, std::memory_order_release);
    }

    [[nodiscard]] bool empty() const noexcept {
        return m_head.load(std::memory_order_relaxed) == m_tail.load(std::memory_order_relaxed);
    }

private:
    static constexpr size_t kCacheLineSize = 64;

    size_t m_capacity;
    size_t m_mask;
    std::vector<T> m_buffer;

    // Separate head and tail onto different cache lines to eliminate false sharing
    alignas(kCacheLineSize) std::atomic<size_t> m_tail{0}; // Written by Producer
    alignas(kCacheLineSize) std::atomic<size_t> m_head{0}; // Written by Consumer
};

} // namespace audio_core
