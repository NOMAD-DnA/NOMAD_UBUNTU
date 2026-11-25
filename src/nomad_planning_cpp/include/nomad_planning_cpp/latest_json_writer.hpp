#pragma once

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <fcntl.h>
#include <functional>
#include <mutex>
#include <poll.h>
#include <pthread.h>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <utility>

namespace nomad_planning_cpp {

// GUI output only. Snapshot generation and all fd I/O run on this worker,
// never on the ROS command timer. There is no telemetry history queue: after
// each completed line the next snapshot is taken from the current state.
// A partially written JSON line must finish before another line can start.
class LatestJsonWriter {
 public:
  using Clock = std::chrono::steady_clock;
  using Snapshot = std::function<std::string()>;
  using Failure = std::function<void()>;
  struct Options {
    std::chrono::milliseconds period{500};
    std::chrono::milliseconds write_timeout{3000};
    std::size_t max_line_bytes = 512 * 1024;
    std::size_t max_ack_lines = 32;
    std::size_t max_ack_bytes = 16 * 1024;
  };

  LatestJsonWriter(int fd, Snapshot snapshot, Failure failure)
      : LatestJsonWriter(fd, std::move(snapshot), std::move(failure), Options{}) {}

  LatestJsonWriter(int fd, Snapshot snapshot, Failure failure, Options options)
      : fd_(fd), snapshot_(std::move(snapshot)), failure_(std::move(failure)), options_(options) {
    if (options_.period.count() <= 0 || options_.write_timeout.count() <= 0 ||
        options_.max_line_bytes < 2 || !options_.max_ack_lines || !options_.max_ack_bytes)
      throw std::invalid_argument("invalid GUI writer bounds");
    struct stat output{};
    if (::fstat(fd_, &output) != 0 ||
        !(S_ISFIFO(output.st_mode) || S_ISSOCK(output.st_mode) || ::isatty(fd_)))
      throw std::invalid_argument("GUI output must be a pipe, socket or terminal");
    original_flags_ = ::fcntl(fd_, F_GETFL, 0);
    if (original_flags_ < 0 || ::fcntl(fd_, F_SETFL, original_flags_ | O_NONBLOCK) < 0)
      throw std::runtime_error("cannot make GUI output nonblocking");
    try { thread_ = std::thread(&LatestJsonWriter::run, this); }
    catch (...) { ::fcntl(fd_, F_SETFL, original_flags_); throw; }
  }

  LatestJsonWriter(const LatestJsonWriter&) = delete;
  LatestJsonWriter& operator=(const LatestJsonWriter&) = delete;
  ~LatestJsonWriter() { stop(); }

  // Only tiny command acknowledgements use a queue. Never discard an ACK
  // silently: overflow makes the transport fail closed through failure_().
  bool acknowledge(std::string line) {
    if (stopping_.load() || failed_.load()) return false;
    bool overflow = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_.load() || failed_.load()) return false;
      if (line.size() + 1 > options_.max_ack_bytes ||
          acks_.size() >= options_.max_ack_lines ||
          ack_bytes_ + line.size() + 1 > options_.max_ack_bytes) overflow = true;
      else { line.push_back('\n'); ack_bytes_ += line.size(); acks_.push_back(std::move(line)); }
    }
    if (overflow) { fail(); return false; }
    wake_.notify_one();
    return true;
  }

  // Coalescing notification only. Repeated status requests never build a queue.
  void request_snapshot() noexcept { requested_.store(true); wake_.notify_one(); }
  bool failed() const noexcept { return failed_.load(); }

  // A small optional grace is useful for a final ACK. A nonreading peer cannot
  // extend shutdown: every write is nonblocking and poll wakes within 50 ms.
  void stop(std::chrono::milliseconds grace = std::chrono::milliseconds(0)) {
    if (!thread_.joinable()) return;
    stop_at_.store((Clock::now() + grace).time_since_epoch().count());
    stopping_.store(true);
    wake_.notify_all();
    thread_.join();
    ::fcntl(fd_, F_SETFL, original_flags_);
  }

 private:
  void fail() noexcept {
    if (stopping_.load()) return; // A clean shutdown need not deliver a final display line.
    if (failed_.exchange(true)) return;
    try { if (failure_) failure_(); } catch (...) {}
    wake_.notify_all();
  }

  bool stop_due() const noexcept {
    return stopping_.load() && Clock::now().time_since_epoch().count() >= stop_at_.load();
  }

  void run() noexcept {
    // An EPIPE must disarm the session, not terminate the process via SIGPIPE.
    // The mask is thread local; no process-wide signal handler is changed.
    sigset_t blocked; ::sigemptyset(&blocked); ::sigaddset(&blocked, SIGPIPE);
    if (::pthread_sigmask(SIG_BLOCK, &blocked, nullptr) != 0) { fail(); return; }
    auto next_snapshot = Clock::now();
    auto progressed = Clock::now();
    std::string current;
    std::size_t offset = 0;
    try {
      while (!failed_.load() && !stop_due()) {
        if (current.empty()) {
          {
            std::unique_lock<std::mutex> lock(mutex_);
            if (!acks_.empty()) {
              current = std::move(acks_.front()); acks_.pop_front(); ack_bytes_ -= current.size();
            } else if (stopping_.load()) return;
            else if (!requested_.load() && Clock::now() < next_snapshot) {
              wake_.wait_until(lock, next_snapshot, [this] {
                return stopping_.load() || failed_.load() || requested_.load() || !acks_.empty();
              });
              continue;
            }
          }
          if (current.empty()) {
            requested_.store(false);
            current = snapshot_(); // State lock/copy inside callback; serialize outside it.
            next_snapshot = Clock::now() + options_.period;
            if (current.empty()) continue;
            if (current.size() + 1 > options_.max_line_bytes) { fail(); return; }
            current.push_back('\n');
          }
          offset = 0;
          progressed = Clock::now();
        }
        const auto written = ::write(fd_, current.data() + offset, current.size() - offset);
        if (written > 0) {
          offset += static_cast<std::size_t>(written);
          progressed = Clock::now();
          if (offset == current.size()) current.clear();
          continue;
        }
        if (written < 0 && errno == EINTR) continue;
        if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
          if (Clock::now() - progressed >= options_.write_timeout) { fail(); return; }
          pollfd output{fd_, POLLOUT, 0};
          const int ready = ::poll(&output, 1, 50);
          if ((ready < 0 && errno != EINTR) ||
              (ready > 0 && (output.revents & (POLLERR | POLLHUP | POLLNVAL)))) { fail(); return; }
          continue;
        }
        fail();
        return;
      }
    } catch (...) { fail(); }
  }

  int fd_, original_flags_ = 0;
  Snapshot snapshot_;
  Failure failure_;
  Options options_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<std::string> acks_;
  std::size_t ack_bytes_ = 0;
  std::atomic<bool> requested_{false}, stopping_{false}, failed_{false};
  std::atomic<Clock::duration::rep> stop_at_{0};
  std::thread thread_;
};

} // namespace nomad_planning_cpp
