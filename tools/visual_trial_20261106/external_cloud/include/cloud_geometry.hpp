#pragma once

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <opencv2/core.hpp>

#include <array>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace external_cloud {
using Tree = boost::property_tree::ptree;

inline void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

inline std::string read_text(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot open " + path);
  std::ostringstream text;
  text << input.rdbuf();
  return text.str();
}

inline Tree parse_json(const std::string& text) {
  std::istringstream input(text);
  Tree tree;
  boost::property_tree::read_json(input, tree);
  return tree;
}

inline cv::Matx33d matrix3(const Tree& rows) {
  cv::Matx33d value;
  require(rows.size() == 3, "Expected three matrix rows");
  int r = 0;
  for (const auto& row : rows) {
    require(row.second.size() == 3, "Expected three matrix columns");
    int c = 0;
    for (const auto& x : row.second) {
      const double v = x.second.get_value<double>();
      require(std::isfinite(v), "Nonfinite matrix value");
      value(r, c++) = v;
    }
    ++r;
  }
  return value;
}

inline void check_rotation(const cv::Matx33d& r) {
  require(cv::norm(cv::Mat(r.t() * r - cv::Matx33d::eye())) < 0.001 &&
              std::abs(cv::determinant(cv::Mat(r)) - 1.0) < 0.001,
          "Invalid calibration rotation");
}

struct Intrinsics {
  double fx{}, fy{}, cx{}, cy{};
};

inline std::array<Intrinsics, 2> read_vio_intrinsics(const Tree& root) {
  const auto& value = root.get_child("value0");
  const auto& resolutions = value.get_child("resolution");
  require(resolutions.size() == 2, "Expected two calibrated camera resolutions");
  for (const auto& resolution : resolutions) {
    require(resolution.second.size() == 2, "Invalid calibrated resolution");
    auto it = resolution.second.begin();
    const int width = (it++)->second.get_value<int>();
    const int height = it->second.get_value<int>();
    require(width == 640 && height == 480, "Bridge requires calibrated VGA images");
  }
  const auto& cameras = value.get_child("intrinsics");
  require(cameras.size() == 2, "Expected two calibrated camera intrinsics");
  std::array<Intrinsics, 2> result;
  int i = 0;
  for (const auto& camera : cameras) {
    require(camera.second.get<std::string>("camera_type") == "pinhole",
            "Bridge requires rectified pinhole images");
    const auto& k = camera.second.get_child("intrinsics");
    result[i++] = {k.get<double>("fx"), k.get<double>("fy"),
                   k.get<double>("cx"), k.get<double>("cy")};
  }
  return result;
}

class Geometry {
 public:
  Geometry(const Tree& factory, const std::array<Intrinsics, 2>& intrinsics)
      : k(intrinsics) {
    require(factory.get<std::string>("productName").find("OAK-D-LITE") == 0,
            "Factory calibration is not OAK-D-LITE");
    std::map<int, Tree> cameras;
    for (const auto& item : factory.get_child("cameraData")) {
      require(item.second.size() == 2, "Invalid factory cameraData pair");
      auto it = item.second.begin();
      const int socket = (it++)->second.get_value<int>();
      cameras.emplace(socket, it->second);
    }
    for (int socket : {1, 2}) {
      require(cameras.at(socket).get<int>("width") == 640 &&
                  cameras.at(socket).get<int>("height") == 480,
              "Factory mono calibration must be VGA");
    }
    const auto& bc = cameras.at(1).get_child("extrinsics");
    const auto& ca = cameras.at(2).get_child("extrinsics");
    require(bc.get<int>("toCameraSocket") == 2 &&
                ca.get<int>("toCameraSocket") == 0 &&
                bc.get<int>("lengthUnit") == 1 && ca.get<int>("lengthUnit") == 1,
            "Expected factory B-to-C-to-A chain in centimeters");
    r_cb = matrix3(bc.get_child("rotationMatrix"));
    r_ac = matrix3(ca.get_child("rotationMatrix"));
    t_cb = translation_m(bc);
    t_ac = translation_m(ca);
    const auto& rect = factory.get_child("stereoRectificationData");
    require(rect.get<int>("leftCameraSocket") == 1 &&
                rect.get<int>("rightCameraSocket") == 2,
            "Expected rectified CAM_B/C stereo pair");
    r_left = matrix3(rect.get_child("rectifiedRotationLeft"));
    r_right = matrix3(rect.get_child("rectifiedRotationRight"));
    for (const auto& rotation : {r_cb, r_ac, r_left, r_right}) check_rotation(rotation);
    const auto rectified_rotation = r_right * r_cb * r_left.t();
    const auto rectified_translation = r_right * t_cb;
    require(cv::norm(cv::Mat(rectified_rotation - cv::Matx33d::eye())) < 0.001 &&
                rectified_translation[0] < 0.0 &&
                std::hypot(rectified_translation[1], rectified_translation[2]) < 0.0001,
            "Factory rectification does not form a horizontal stereo pair");
    baseline_m = -rectified_translation[0];
    require(baseline_m >= 0.05 && baseline_m <= 0.1, "Invalid stereo baseline");
    for (const auto& camera : k) {
      require(std::isfinite(camera.fx) && std::isfinite(camera.fy) &&
                  std::isfinite(camera.cx) && std::isfinite(camera.cy) &&
                  camera.fx > 0 && camera.fy > 0 && camera.cx >= 0 &&
                  camera.cx < 640 && camera.cy >= 0 && camera.cy < 480,
              "Invalid rectified intrinsics");
    }
    require(std::abs(k[0].fx - k[1].fx) < 0.01 &&
                std::abs(k[0].fy - k[1].fy) < 0.01 &&
                std::abs(k[0].cy - k[1].cy) < 0.01,
            "StereoSGBM requires matching rectified focal lengths and epipolar rows");
    // EEPROM rotations map physical camera coordinates into rectified coordinates.
    // Undo left rectification before applying B->C->A; naming a cloud CAM_A is
    // insufficient without this rotation and both physical translations.
    r_a_rectified_left = r_ac * r_cb * r_left.t();
    t_a_rectified_left = r_ac * t_cb + t_ac;
  }

  bool point_a(int u, int v, double raw_disparity, double min_z,
               double max_z, cv::Vec3f& output) const {
    const double d = raw_disparity - k[0].cx + k[1].cx;
    if (!std::isfinite(d) || d <= 0.0) return false;
    const double z = k[0].fx * baseline_m / d;
    if (z <= min_z || z >= max_z) return false;
    const cv::Vec3d rectified((u - k[0].cx) * z / k[0].fx,
                              (v - k[0].cy) * z / k[0].fy, z);
    const auto a = r_a_rectified_left * rectified + t_a_rectified_left;
    if (!std::isfinite(a[0]) || !std::isfinite(a[1]) ||
        !std::isfinite(a[2]) || a[2] <= 0.0) return false;
    output = cv::Vec3f(a);
    return true;
  }

  static cv::Vec3d translation_m(const Tree& extrinsics) {
    const auto& t = extrinsics.get_child("translation");
    cv::Vec3d value(t.get<double>("x") / 100.0, t.get<double>("y") / 100.0,
                    t.get<double>("z") / 100.0);
    require(std::isfinite(value[0]) && std::isfinite(value[1]) &&
                std::isfinite(value[2]), "Nonfinite factory translation");
    return value;
  }

  std::array<Intrinsics, 2> k;
  double baseline_m{};
  cv::Matx33d r_cb, r_ac, r_left, r_right, r_a_rectified_left;
  cv::Vec3d t_cb, t_ac, t_a_rectified_left;
};
}  // namespace external_cloud
