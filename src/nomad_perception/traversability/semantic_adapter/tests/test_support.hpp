#pragma once
#include "semantic_mapping.hpp"
#include <iostream>

inline void check(bool ok, const std::string &why) {
  if (!ok) throw std::runtime_error(why);
}
inline nomad_semantic::Json model_fixture(const nomad_semantic::Json &config) {
  nomad_semantic::Json model;
  for (const auto &item : config.at("classes"))
    model["id2label"][std::to_string(item.at("source_id").get<int>())] = item.at("source_name");
  return model;
}
inline sensor_msgs::msg::Image image_fixture() {
  sensor_msgs::msg::Image image;
  // Deliberately old acquisition stamp: conversion must never make it fresh.
  image.header.stamp.sec = 123; image.header.stamp.nanosec = 456789;
  image.header.frame_id = "zed2_23472639_left_rectified_optical_frame";
  image.width = 640; image.height = 360; image.encoding = "mono8";
  image.step = 644; image.data.assign(size_t(image.step) * image.height, 255);
  const std::array<uint8_t,17> samples{{0,12,20,13,34,2,4,6,11,91,52,46,126,150,254,255,3}};
  for (size_t row=0; row<image.height; ++row)
    for (size_t col=0; col<image.width; ++col)
      image.data[row*image.step+col] = samples[(row+col)%samples.size()];
  return image;
}
inline void check_output(const sensor_msgs::msg::Image &input, const sensor_msgs::msg::Image &output) {
  const std::array<uint8_t,17> expected{{10,1,2,4,7,11,9,6,6,4,0,0,0,0,0,0,6}};
  check(output.header == input.header, "Acquisition header changed");
  check(output.width==640 && output.height==360 && output.encoding=="mono8" &&
    output.step==640 && output.is_bigendian==input.is_bigendian && output.data.size()==640*360,
    "Output Image contract differs");
  for (size_t row=0; row<360; ++row)
    for (size_t col=0; col<640; ++col)
      check(output.data[row*640+col]==expected[(row+col)%expected.size()], "Incorrect pixel ID / row padding handling");
}
