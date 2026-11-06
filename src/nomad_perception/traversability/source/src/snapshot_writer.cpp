#include "nomad_costmap/snapshot_writer.hpp"

namespace nomad {
SnapshotWriter::SnapshotWriter(std::function<void(const Json&)> sink) : sink_(std::move(sink)) {
  worker_ = std::thread([this] {
    while (true) {
      std::shared_ptr<const Json> next;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait(lock, [this] { return stopping_ || latest_; });
        if (!latest_) return;
        next = std::move(latest_);
      }
      try { sink_(*next); }
      catch (const std::exception& error) {
        std::lock_guard<std::mutex> lock(mutex_);
        fault_ = error.what();
      }
    }
  });
}
SnapshotWriter::SnapshotWriter(const std::filesystem::path& directory)
  : SnapshotWriter([directory](const Json& result) {
      atomic_json(directory / "terrain/live.json", result.at("live"));
      atomic_json(directory / "terrain/timing.json", result.at("timing"));
    }) {}
SnapshotWriter::~SnapshotWriter() { stop(); }
void SnapshotWriter::submit(std::shared_ptr<const Json> snapshot) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopping_ || !fault_.empty()) return;
  latest_ = std::move(snapshot);
  changed_.notify_one();
}
void SnapshotWriter::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
    changed_.notify_one();
  }
  if (worker_.joinable()) worker_.join();
}
std::string SnapshotWriter::fault() {
  std::lock_guard<std::mutex> lock(mutex_);
  return fault_;
}
}
