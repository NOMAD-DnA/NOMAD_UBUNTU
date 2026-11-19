#pragma once
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace nomad {
// One caller owns a batch. The caller participates; threads includes that caller.
// Jobs may only read shared inputs and write their own preallocated result slot.
class CellExecutor {
  std::mutex mutex_, run_mutex_;
  std::condition_variable ready_, done_;
  std::vector<std::thread> workers_;
  std::function<void(size_t)> job_;
  std::atomic<size_t> next_{0};
  size_t count_ = 0, pending_ = 0, generation_ = 0;
  bool stop_ = false;
  std::exception_ptr error_;
  void work();
public:
  explicit CellExecutor(size_t threads);
  ~CellExecutor();
  CellExecutor(const CellExecutor&) = delete;
  CellExecutor& operator=(const CellExecutor&) = delete;
  size_t threads() const { return workers_.size()+1; }
  void run(size_t count, std::function<void(size_t)> job);
};
}
