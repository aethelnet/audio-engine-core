#pragma once

#include <vector>
#include <thread>
#include <atomic>
#include <cstdint>
#include <algorithm>

#if defined(__x86_64__) || defined(_M_X64)
#include <xmmintrin.h>
#include <pmmintrin.h>
#include <immintrin.h>
#endif

namespace audio_core::threading {

// ============================================================================
// AudioWorkerPool: Lock-Free, Zero-Allocation Multi-Core Real-Time Thread Pool
// - Pre-spawned worker threads with hardware FTZ/DAZ denormal protection
// - Dynamic atomic work-stealing counter
// - Sub-microsecond cache-aligned synchronization without mutexes
// ============================================================================
class AudioWorkerPool {
public:
    using JobFn = void(*)(void* context, uint32_t job_idx) noexcept;

    static constexpr uint32_t kAutoDetect = 0xFFFFFFFF;

    explicit AudioWorkerPool(uint32_t num_threads = kAutoDetect) {
        init(num_threads);
    }

    ~AudioWorkerPool() {
        shutdown();
    }

    AudioWorkerPool(const AudioWorkerPool&) = delete;
    AudioWorkerPool& operator=(const AudioWorkerPool&) = delete;
    AudioWorkerPool(AudioWorkerPool&&) = delete;
    AudioWorkerPool& operator=(AudioWorkerPool&&) = delete;

    void init(uint32_t num_threads) {
        shutdown();

        if (num_threads == kAutoDetect) {
            uint32_t hw = std::thread::hardware_concurrency();
            num_threads = (hw > 1) ? (hw - 1) : 0;
        }
        num_threads = std::min(num_threads, 32u);
        m_num_workers = num_threads;

        if (m_num_workers == 0) {
            return; // Single-threaded fallback mode
        }

        m_running.store(true, std::memory_order_release);
        m_cycle_id.store(0, std::memory_order_relaxed);
        m_workers_done.store(0, std::memory_order_relaxed);

        m_workers.reserve(m_num_workers);
        for (uint32_t i = 0; i < m_num_workers; ++i) {
            m_workers.emplace_back([this, i]() {
                worker_loop(i);
            });
        }
    }

    void shutdown() {
        if (!m_running.load(std::memory_order_acquire)) return;
        m_running.store(false, std::memory_order_release);
        m_cycle_id.fetch_add(1, std::memory_order_release);
        m_cycle_id.notify_all();

        for (auto& w : m_workers) {
            if (w.joinable()) {
                w.join();
            }
        }
        m_workers.clear();
        m_num_workers = 0;
    }

    [[nodiscard]] uint32_t num_workers() const noexcept {
        return m_num_workers;
    }

    // Parallel-for loop over jobs [0 .. total_jobs - 1]
    // Lock-free, zero-allocation dispatch across all workers and calling thread.
    void parallel_for(uint32_t total_jobs, void* context, JobFn job_fn) noexcept {
        if (total_jobs == 0 || !job_fn) return;

        // If no worker threads configured, execute directly on calling thread
        if (m_num_workers == 0) {
            for (uint32_t j = 0; j < total_jobs; ++j) {
                job_fn(context, j);
            }
            return;
        }

        m_current_context = context;
        m_current_job_fn = job_fn;
        m_total_jobs.store(total_jobs, std::memory_order_relaxed);
        m_job_counter.store(0, std::memory_order_relaxed);
        m_workers_done.store(0, std::memory_order_relaxed);

        // Wake all worker threads for new cycle
        const uint32_t next_cycle = m_cycle_id.load(std::memory_order_relaxed) + 1;
        m_cycle_id.store(next_cycle, std::memory_order_release);
        m_cycle_id.notify_all();

        // Calling thread participates in work processing
        process_jobs();

        // Wait for all worker threads to report completion
        while (m_workers_done.load(std::memory_order_acquire) < m_num_workers) {
            #if defined(__x86_64__) || defined(_M_X64)
            _mm_pause();
            #else
            std::this_thread::yield();
            #endif
        }
    }

private:
    void worker_loop(uint32_t /*worker_idx*/) noexcept {
        // Enforce Flush-To-Zero and Denormals-Are-Zero on this worker core
        #if defined(__x86_64__) || defined(_M_X64)
        _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
        _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
        #endif

        uint32_t last_cycle = 0;

        while (m_running.load(std::memory_order_relaxed)) {
            // Wait for next cycle notification
            uint32_t curr_cycle = m_cycle_id.load(std::memory_order_acquire);
            while (curr_cycle == last_cycle && m_running.load(std::memory_order_relaxed)) {
                m_cycle_id.wait(curr_cycle);
                curr_cycle = m_cycle_id.load(std::memory_order_acquire);
            }

            if (!m_running.load(std::memory_order_relaxed)) break;
            last_cycle = curr_cycle;

            // Dynamically steal and process jobs
            process_jobs();

            // Mark completion of this worker
            m_workers_done.fetch_add(1, std::memory_order_release);
        }
    }

    void process_jobs() noexcept {
        const uint32_t total = m_total_jobs.load(std::memory_order_relaxed);
        void* ctx = m_current_context;
        JobFn fn = m_current_job_fn;
        if (!fn) return;

        while (true) {
            uint32_t job = m_job_counter.fetch_add(1, std::memory_order_relaxed);
            if (job >= total) {
                break;
            }
            fn(ctx, job);
        }
    }

    std::vector<std::thread> m_workers;
    uint32_t m_num_workers{0};
    std::atomic<bool> m_running{false};

    alignas(64) std::atomic<uint32_t> m_cycle_id{0};
    alignas(64) std::atomic<uint32_t> m_workers_done{0};
    alignas(64) std::atomic<uint32_t> m_job_counter{0};
    alignas(64) std::atomic<uint32_t> m_total_jobs{0};

    void* m_current_context{nullptr};
    JobFn m_current_job_fn{nullptr};
};

} // namespace audio_core::threading
