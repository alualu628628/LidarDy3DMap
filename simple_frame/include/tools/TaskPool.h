#ifndef SIMPLE_FRAME_TASK_POOL_H
#define SIMPLE_FRAME_TASK_POOL_H

#include <algorithm>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <vector>

namespace simple_frame {

// A small fixed-size pool for independent sector reconstruction tasks.  It is
// deliberately local to simple_frame so the package does not inherit the task
// dropping semantics of hash_fusion's streaming pool.
class TaskPool {
public:
    explicit TaskPool(std::size_t worker_count) {
        worker_count = std::max<std::size_t>(1, worker_count);
        workers_.reserve(worker_count);
        for (std::size_t i = 0; i < worker_count; ++i) {
            workers_.emplace_back([this] { WorkerLoop(); });
        }
    }

    ~TaskPool() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        ready_.notify_all();
        for (std::thread& worker : workers_) {
            if (worker.joinable()) worker.join();
        }
    }

    TaskPool(const TaskPool&) = delete;
    TaskPool& operator=(const TaskPool&) = delete;

    template <typename Function>
    std::future<void> Submit(Function&& function) {
        std::packaged_task<void()> task(std::forward<Function>(function));
        std::future<void> result = task.get_future();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tasks_.emplace(std::move(task));
        }
        ready_.notify_one();
        return result;
    }

private:
    void WorkerLoop() {
        while (true) {
            std::packaged_task<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                ready_.wait(lock, [this] { return stopping_ || !tasks_.empty(); });
                if (stopping_ && tasks_.empty()) return;
                task = std::move(tasks_.front());
                tasks_.pop();
            }
            task();
        }
    }

    std::mutex mutex_;
    std::condition_variable ready_;
    std::queue<std::packaged_task<void()>> tasks_;
    std::vector<std::thread> workers_;
    bool stopping_ = false;
};

}  // namespace simple_frame

#endif
