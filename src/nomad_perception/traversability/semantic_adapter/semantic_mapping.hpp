#pragma once
#include <array>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace nomad_semantic {
using Json = nlohmann::json;
inline Json read_json(const std::string &path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("Cannot open JSON: " + path);
  Json value; file >> value; return value;
}
struct Mapping {
  std::array<uint8_t, 256> lookup{};
  std::string input_topic, output_topic, frame_id;
  uint32_t width = 0, height = 0;
  static Mapping load(const Json &config, const Json &model) {
    Mapping result;
    auto require = [](bool ok, const char *why) {
      if (!ok) throw std::runtime_error(why);
    };
    require(config.at("schema_version") == 1, "Unsupported mapping schema");
    require(config.at("source_class_count") == 150, "Expected ADE150 schema");
    require(config.at("unknown_target_id") == 0, "Unknown IDs must stay ignored");
    const std::array<std::string, 12> expected{{"ignore", "person", "vehicle", "wood",
      "soil", "gravel", "paved", "rock", "leaves", "vegetation", "artificial", "sky"}};
    require(config.at("project_classes") == Json(expected), "Project class order differs");
    require(config.at("unavailable_target_ids") == Json::array({3,5,8}),
      "Unsupported ADE target classes must be explicit");
    const auto &contract = config.at("image_contract");
    require(contract.at("width") == 640 && contract.at("height") == 360 &&
      contract.at("encoding") == "mono8", "Expected 640x360 mono8 contract");
    result.width = 640; result.height = 360;
    result.frame_id = contract.at("frame_id").get<std::string>();
    require(result.frame_id == "zed2_23472639_left_rectified_optical_frame",
      "Unexpected semantic optical frame");
    result.input_topic = config.at("input_topic").get<std::string>();
    result.output_topic = config.at("output_topic").get<std::string>();
    require(!result.input_topic.empty() && !result.output_topic.empty() &&
      result.input_topic != result.output_topic, "Input/output topics must differ");
    require(config.at("classes").is_array() && config.at("classes").size() == 150,
      "Mapping must contain every source ID exactly once");
    const auto &labels = model.at("id2label");
    require(labels.is_object() && labels.size() == 150, "Source model is not ADE150");
    std::array<bool,150> seen{};
    for (const auto &item : config.at("classes")) {
      require(item.at("source_id").is_number_integer() &&
        item.at("target_id").is_number_integer(), "Mapping IDs must be integers");
      const int source = item.at("source_id").get<int>();
      const int target = item.at("target_id").get<int>();
      require(source >= 0 && source < 150 && target >= 0 && target < 12,
        "Class ID outside the allowed range");
      require(!seen[source], "Duplicate source ID");
      require(target != 3 && target != 5 && target != 8,
        "This ADE model cannot identify wood, gravel or fallen leaves");
      require(labels.at(std::to_string(source)) == item.at("source_name"),
        "Source model class order differs from mapping");
      seen[source] = true;
      result.lookup[source] = static_cast<uint8_t>(target);
    }
    return result;
  }
  // Padding is not a pixel. Do not retimestamp or change optical coordinates.
  bool convert(const sensor_msgs::msg::Image &input, sensor_msgs::msg::Image &output,
               std::string &error) const {
    if (input.encoding != "mono8") error = "Expected mono8 class IDs";
    else if (input.width != width || input.height != height) error = "Unexpected image dimensions";
    else if (input.header.frame_id != frame_id) error = "Unexpected optical frame";
    else if (input.header.stamp.sec < 0 || input.header.stamp.nanosec >= 1000000000u ||
             (input.header.stamp.sec == 0 && input.header.stamp.nanosec == 0))
      error = "Invalid acquisition timestamp";
    else if (input.is_bigendian > 1 || input.step < width ||
      uint64_t(input.step) * input.height != input.data.size()) error = "Invalid row stride or data size";
    else {
      output.header = input.header;
      output.width = width; output.height = height; output.encoding = "mono8";
      output.is_bigendian = input.is_bigendian; output.step = width;
      output.data.resize(size_t(width) * height);
      for (size_t row = 0; row < height; ++row)
        for (size_t col = 0; col < width; ++col)
          output.data[row * width + col] = lookup[input.data[row * input.step + col]];
      error.clear(); return true;
    }
    return false;
  }
};
}  // namespace nomad_semantic
