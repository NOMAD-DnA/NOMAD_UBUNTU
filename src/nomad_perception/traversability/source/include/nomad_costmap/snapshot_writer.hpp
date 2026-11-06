#pragma once
#include "runtime.hpp"

namespace nomad {
// Best-effort latest diagnostic files, independent of observation delivery.
// The immutable snapshot is shared with the mapper, avoiding a full JSON copy.
class SnapshotWriter {
  std::function<void(const Json&)> sink_;
  std::mutex mutex_;
  std::condition_variable changed_;
  std::shared_ptr<const Json> latest_;
  bool stopping_ = false;
  std::string fault_;
  std::thread worker_;
public:
  explicit SnapshotWriter(std::function<void(const Json&)> sink);
  explicit SnapshotWriter(const std::filesystem::path& directory);
  ~SnapshotWriter();
  void submit(std::shared_ptr<const Json> snapshot);
  void stop();
  std::string fault();
};
}
