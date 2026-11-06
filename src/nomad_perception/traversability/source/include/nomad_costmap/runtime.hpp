#pragma once
#include "core.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <functional>
#include <mutex>
#include <thread>
#include <sys/types.h>

namespace nomad {
  int64_t steady_ns();
  Json read_json(const std::filesystem::path&);
  void atomic_json(const std::filesystem::path&, const Json&);
  std::string sha256_file(const std::filesystem::path&);
  std::string sha256_text(const std::string&);
  std::string executable_path();
  class Arguments {
    std::map<std::string, std::string> values;
public:
    Arguments(int argc, char** argv);
    std::string get(const std::string& key, const std::string& fallback = "") const;
    bool has(const std::string& key) const;
  };

// One ordered, framed CBOR stream per child. The reader is independent of ROS
// callbacks and map computation. A full queue is a visible fault, never overwrite.
  class Channel {
    int fd_;
    std::atomic<bool> stopping_{false};
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<Json> outgoing_;
    std::thread reader_, writer_;
    std::string fault_;
    void fail(const std::string&);
public:
    explicit Channel(int fd) : fd_(fd)
    {
    }
    ~Channel();
    Channel(const Channel&) = delete;
    Channel& operator=(const Channel&) = delete;
    void start(std::function<void(Json)> callback);
    void send(Json message);
    void stop();
    std::string fault() const;
  };
  struct Child {
    pid_t pid = -1;
    std::unique_ptr<Channel> channel;
    void launch(const std::vector<std::string>&, std::function<void(Json)>);
    bool alive();
    void stop();
    ~Child();
  };
  class PublisherLock {
    int fd_ = -1;
public:
    explicit PublisherLock(const std::string& topic);
    ~PublisherLock();
  };
  Json frame_json(const Frame&);
  Frame frame_from_json(const Json&);
}
