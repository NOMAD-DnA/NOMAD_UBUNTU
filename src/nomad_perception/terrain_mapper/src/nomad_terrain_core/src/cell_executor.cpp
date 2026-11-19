#include "nomad_costmap/cell_executor.hpp"
#include <algorithm>
#include <stdexcept>

namespace nomad {
CellExecutor::CellExecutor(size_t threads)
{
  if (threads < 1 || threads > 32) throw std::invalid_argument("Slope threads must be 1..32");
  try {
    for (size_t i=1;i<threads;++i) workers_.emplace_back([this] {
      size_t seen=0;
      std::unique_lock<std::mutex> lock(mutex_);
      for (;;) {
        ready_.wait(lock,[&] { return stop_ || seen != generation_; });
        if (stop_) return;
        seen=generation_;
        lock.unlock();
        work();
        lock.lock();
        if (--pending_ == 0) done_.notify_one();
      }
    });
  } catch (...) {
    { std::lock_guard<std::mutex> lock(mutex_); stop_=true; }
    ready_.notify_all();
    for (auto& worker:workers_) worker.join();
    throw;
  }
}
CellExecutor::~CellExecutor()
{
  { std::lock_guard<std::mutex> lock(mutex_); stop_=true; }
  ready_.notify_all();
  for (auto& worker:workers_) worker.join();
}
void CellExecutor::work()
{
  try {
    // Small chunks balance sparse and dense cells without an atomic per point.
    for (;;) {
      const size_t begin=next_.fetch_add(4,std::memory_order_relaxed);
      if (begin >= count_) break;
      for (size_t i=begin;i<std::min(begin+4,count_);++i) job_(i);
    }
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!error_) error_=std::current_exception();
    next_.store(count_,std::memory_order_relaxed);
  }
}
void CellExecutor::run(size_t count, std::function<void(size_t)> job)
{
  std::lock_guard<std::mutex> batch_lock(run_mutex_);
  if (!count) return;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    job_=std::move(job); count_=count; next_=0; error_=nullptr;
    pending_=workers_.size(); ++generation_;
  }
  ready_.notify_all();
  work();
  std::unique_lock<std::mutex> lock(mutex_);
  done_.wait(lock,[&] { return pending_ == 0; });
  job_={};
  if (error_) std::rethrow_exception(error_);
}
}
