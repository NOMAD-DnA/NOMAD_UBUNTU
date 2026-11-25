#pragma once
#include <chrono>
#include <condition_variable>
#include <fcntl.h>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <cerrno>

namespace nomad_planning_cpp {
// All sampling/serialization/disk work stays off the ROS command executor.
// No history queue: slow storage lowers recording frequency, never grows RAM.
class TrajectoryWriter {
 public:
  using Snapshot = std::function<std::string()>;
  using Failure = std::function<void(const std::string&)>;
  TrajectoryWriter(const std::string& file, Snapshot snapshot, Failure failure,
                   std::size_t max_bytes = 64u * 1024u * 1024u)
      : snapshot_(std::move(snapshot)), failure_(std::move(failure)), max_bytes_(max_bytes) {
    // Never truncate an existing run or follow a symlink/device/FIFO.
    fd_ = ::open(file.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0644);
    if (fd_ < 0) throw std::runtime_error("cannot create trajectory file: " + file);
    try { thread_ = std::thread(&TrajectoryWriter::run, this); }
    catch (...) { ::close(fd_); throw; }
  }
  TrajectoryWriter(const TrajectoryWriter&) = delete;
  TrajectoryWriter& operator=(const TrajectoryWriter&) = delete;
  ~TrajectoryWriter() { stop(); }
  void stop() {
    { std::lock_guard<std::mutex> lock(mutex_); stopping_ = true; }
    wake_.notify_all();
    if (thread_.joinable()) thread_.join();
  }
 private:
  void write_line(const std::string& line) {
    std::string data = line + '\n';
    if (bytes_ + data.size() > max_bytes_) throw std::runtime_error("trajectory size limit (64 MiB) reached");
    std::size_t offset = 0;
    while (offset < data.size()) {
      const auto n = ::write(fd_, data.data() + offset, data.size() - offset);
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0) throw std::runtime_error("trajectory write failed");
      offset += static_cast<std::size_t>(n);
    }
    bytes_ += data.size();
  }
  void run() noexcept {
    try {
      write_line("{\"kind\":\"recorder\",\"event\":\"started\",\"schema\":\"nomad_trajectory_v1\",\"sample_hz\":10}");
      auto sync_at = std::chrono::steady_clock::now();
      for (;;) {
        { std::lock_guard<std::mutex> lock(mutex_); if (stopping_) break; }
        auto line = snapshot_();
        if (!line.empty()) { write_line(line); ++samples_; }
        if (std::chrono::steady_clock::now() >= sync_at) {
          if (::fdatasync(fd_) != 0) throw std::runtime_error("trajectory sync failed");
          sync_at = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        }
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait_for(lock, std::chrono::milliseconds(100), [this] { return stopping_; });
      }
      write_line("{\"kind\":\"recorder\",\"event\":\"closed\",\"samples\":" + std::to_string(samples_) + "}");
      if (::fdatasync(fd_) != 0) throw std::runtime_error("trajectory final sync failed");
    } catch (const std::exception& e) { try { failure_(e.what()); } catch (...) {} }
      catch (...) { try { failure_("trajectory sampling failed"); } catch (...) {} }
    ::close(fd_);
  }
  Snapshot snapshot_; Failure failure_;
  std::size_t max_bytes_, bytes_ = 0, samples_ = 0;
  int fd_ = -1; bool stopping_ = false;
  std::mutex mutex_; std::condition_variable wake_; std::thread thread_;
};
} // namespace nomad_planning_cpp
