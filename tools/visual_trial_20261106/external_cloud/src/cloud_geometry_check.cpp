#include "cloud_geometry.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <vector>

int main(int argc, char** argv) {
  try {
    if (argc != 3) throw std::runtime_error("usage: cloud_geometry_check factory.json vio_calib.json");
    const auto factory = external_cloud::parse_json(external_cloud::read_text(argv[1]));
    const auto vio = external_cloud::parse_json(external_cloud::read_text(argv[2]));
    external_cloud::Geometry geometry(factory, external_cloud::read_vio_intrinsics(vio));
    auto check = [](bool ok, const char* reason) { external_cloud::require(ok, reason); };
    check(std::abs(geometry.baseline_m - 0.07495242881048438) < 1e-6,
          "Saved measured baseline differs");
    cv::Vec3f point;
    check(geometry.point_a(320, 320, 24.0, 0.2, 10.0, point), "Valid disparity rejected");
    const double z = geometry.k[0].fx * geometry.baseline_m / 24.0;
    const cv::Vec3d rectified((320 - geometry.k[0].cx) * z / geometry.k[0].fx,
                              (320 - geometry.k[0].cy) * z / geometry.k[0].fy, z);
    const auto physical_b = geometry.r_left.t() * rectified;
    const auto physical_c = geometry.r_cb * physical_b + geometry.t_cb;
    const auto physical_a = geometry.r_ac * physical_c + geometry.t_ac;
    check(cv::norm(cv::Vec3d(point) - physical_a) < 1e-6, "CAM_A transform chain mismatch");
    const auto back_to_c = geometry.r_ac.t() * (cv::Vec3d(point) - geometry.t_ac);
    const auto back_to_b = geometry.r_cb.t() * (back_to_c - geometry.t_cb);
    check(cv::norm(geometry.r_left * back_to_b - rectified) < 1e-6,
          "CAM_A reconstruction does not round trip");
    check(cv::norm(cv::Vec3d(point) - rectified) > 0.02, "CAM_A transform unexpectedly omitted");
    check(!geometry.point_a(320, 320, 0, 0.2, 10, point), "Zero disparity accepted");
    check(!geometry.point_a(320, 320, -1, 0.2, 10, point), "Negative disparity accepted");
    check(!geometry.point_a(320, 320, 1000, 0.2, 10, point), "Too-close point accepted");
    check(!geometry.point_a(320, 320, 0.01, 0.2, 10, point), "Too-far point accepted");

    // Synthetic pair: a known 24-pixel horizontal shift, no sensor or ROS node.
    cv::Mat left(480, 640, CV_8UC1), right(480, 640, CV_8UC1);
    cv::RNG random(20261106);
    random.fill(left, cv::RNG::UNIFORM, 0, 256);
    cv::GaussianBlur(left, left, cv::Size(3, 3), 0.5);
    random.fill(right, cv::RNG::UNIFORM, 0, 256);
    left(cv::Rect(24, 0, 616, 480)).copyTo(right(cv::Rect(0, 0, 616, 480)));
    auto matcher = cv::StereoSGBM::create(0, 192, 7, 8 * 49, 32 * 49,
                                        1, 31, 15, 50, 2, cv::StereoSGBM::MODE_SGBM);
    cv::setNumThreads(1);
    cv::Mat disparity;
    matcher->compute(left(cv::Rect(0, 199, 640, 281)),
                     right(cv::Rect(0, 199, 640, 281)), disparity);
    std::size_t good = 0, total = 0;
    for (int v = 206; v < 472; v += 8)
      for (int u = 240; u < 600; u += 8) {
        ++total;
        const double d = disparity.at<std::int16_t>(v - 199, u) / 16.0;
        if (std::abs(d - 24.0) <= 0.25 && geometry.point_a(u, v, d, 0.2, 10, point)) ++good;
      }
    check(good > total * 0.95, "SGBM known-shift metric reconstruction failed");
    std::cout << "PASS geometry/units/CAM_A chain/roundtrip/range rejection/SGBM shift; "
              << good << "/" << total << " known-shift points; baseline_m="
              << geometry.baseline_m << "\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << "\n";
    return 1;
  }
}
