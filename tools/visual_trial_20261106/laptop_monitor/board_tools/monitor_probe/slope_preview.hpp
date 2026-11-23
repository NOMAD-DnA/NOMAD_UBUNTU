#pragma once
// GUI preview only. Read the running mapper's startup report, never saved next-run settings.
#include <cmath>
#include <filesystem>
#include <fstream>
#include <regex>
#include <string>

namespace slope_preview {
struct Scale {
  double max_deg = 30.;
  double k = 0.;
  bool applied = false;
};

inline Scale report_scale(const std::string& line) {
  static const std::regex fields(
    R"(slope_score_per_deg=([0-9.]+) slope_block_deg=([0-9.]+))");
  std::smatch match;
  if (line.find("terrain_mapper_started ") == std::string::npos ||
      !std::regex_search(line, match, fields)) return {};
  try {
    const double k = std::stod(match[1]), block = std::stod(match[2]);
    if (std::isfinite(k) && std::isfinite(block) && k > 0 && k <= 100 && block > 0 && block <= 90)
      return {block, k, true};
  } catch (...) {}
  return {};
}

inline Scale running_scale(const std::filesystem::path& proc = "/proc") {
  namespace fs = std::filesystem;
  std::error_code error;
  Scale found;
  for (fs::directory_iterator it(proc, error), end; !error && it != end; it.increment(error)) {
    const auto name = it->path().filename().string();
    if (name.empty() || name.find_first_not_of("0123456789") != std::string::npos) continue;
    std::error_code ec;
    if (fs::read_symlink(it->path()/"exe", ec).filename() != "nomad_terrain_mapper" || ec) continue;
    // The GUI starts the mapper with stdout redirected to this session's terrain.log.
    // Resolve the link, rather than reading a process pipe or terminal.
    const auto log = fs::read_symlink(it->path()/"fd/1", ec);
    if (ec || !fs::is_regular_file(log, ec) || ec) return {};
    std::ifstream input(log);
    std::string line;
    Scale candidate;
    for (int count = 0; count < 512 && std::getline(input, line); ++count) {
      candidate = report_scale(line);
      if (candidate.applied) break;
    }
    if (!candidate.applied || found.applied) return {}; // no ambiguous multiple publishers
    found = candidate;
  }
  return found;
}
} // namespace slope_preview
