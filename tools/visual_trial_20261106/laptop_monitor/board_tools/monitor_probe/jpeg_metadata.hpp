#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

namespace nomad_monitor {
// Read dimensions without decoding/re-encoding an already compressed preview.
inline std::pair<int, int> jpeg_dimensions(const std::vector<uint8_t>& data) {
  if (data.size() < 4 || data[0] != 0xff || data[1] != 0xd8 ||
      data[data.size() - 2] != 0xff || data.back() != 0xd9)
    throw std::runtime_error("Incomplete tracking JPEG");
  std::size_t pos = 2;
  while (pos < data.size()) {
    if (data[pos++] != 0xff) break;
    while (pos < data.size() && data[pos] == 0xff) ++pos;
    if (pos >= data.size()) break;
    const uint8_t marker = data[pos++];
    if (marker == 0xda || marker == 0xd9 || marker == 0x00) break;
    if (marker == 0x01 || marker == 0xd8 || (marker >= 0xd0 && marker <= 0xd7)) continue;
    if (data.size() - pos < 2) break;
    const std::size_t len = (std::size_t(data[pos]) << 8) | data[pos + 1];
    if (len < 2 || len > data.size() - pos) break;
    const bool sof = marker >= 0xc0 && marker <= 0xcf && marker != 0xc4 &&
                     marker != 0xc8 && marker != 0xcc;
    if (sof) {
      if (len < 8) break;
      const int height = (int(data[pos + 3]) << 8) | data[pos + 4];
      const int width = (int(data[pos + 5]) << 8) | data[pos + 6];
      if (width <= 0 || height <= 0 || width > 4096 || height > 4096) break;
      return {width, height};
    }
    pos += len;
  }
  throw std::runtime_error("Tracking JPEG has no valid frame dimensions");
}
}  // namespace nomad_monitor
