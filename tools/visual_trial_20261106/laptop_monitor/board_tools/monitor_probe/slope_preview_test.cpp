// No ROS nodes, publishers, sensors or vehicle commands are created by this test.
#define main monitor_program_main
#include "monitor_probe.cpp"
#undef main
#include <cassert>
#include <fstream>

int main() {
  using slope_preview::report_scale;
  assert(!report_scale("unrelated slope_score_per_deg=8 slope_block_deg=10").applied);
  assert(!report_scale("terrain_mapper_started slope_score_per_deg=0 slope_block_deg=10").applied);
  assert(!report_scale("terrain_mapper_started slope_score_per_deg=8 slope_block_deg=0").applied);
  const auto four = report_scale("terrain_mapper_started slope_score_per_deg=4.000 slope_block_deg=20.000");
  const auto eight = report_scale("terrain_mapper_started slope_score_per_deg=8.000 slope_block_deg=10.000");
  assert(four.applied && four.max_deg == 20 && eight.applied && eight.max_deg == 10);
  Image image;
  image.width = 4; image.height = 2; image.step = 16; image.encoding = "32FC1";
  const uint16_t endian = 1;
  image.is_bigendian = *reinterpret_cast<const uint8_t*>(&endian) == 0;
  float angles[] = {0, 5, 10, 20, 30, 45, NAN, -5};
  image.data.resize(sizeof angles);
  std::memcpy(image.data.data(), angles, sizeof angles);
  for (const auto& scale : {four, eight, slope_preview::Scale{40, 2, true}}) {
    auto rgb = slope_image(image, scale);
    for (int y=0; y<2; ++y) for (int x=0; x<4; ++x) {
      const float a = angles[(1-y)*4+x];
      const auto expected = std::isfinite(a) ? heat_color(std::abs(a)/scale.max_deg) : cv::Vec3b(100,100,100);
      assert(rgb.at<cv::Vec3b>(y,x) == expected);
    }
  }
  assert(slope_image(image, four).at<cv::Vec3b>(1,2) == cv::Vec3b(0,255,255)); // 10 yellow
  assert(slope_image(image, four).at<cv::Vec3b>(1,3) == cv::Vec3b(0,0,255));   // 20 red
  assert(slope_image(image, eight).at<cv::Vec3b>(1,1) == cv::Vec3b(0,255,255)); // 5 yellow
  assert(slope_image(image, eight).at<cv::Vec3b>(1,2) == cv::Vec3b(0,0,255));   // 10 red
  Grid grid; grid.info.width=4; grid.info.height=1; grid.data={-1,0,50,100};
  auto costs = cost_image(grid);
  assert(costs.at<cv::Vec3b>(0,0) == cv::Vec3b(100,100,100));
  assert(costs.at<cv::Vec3b>(0,1) == cv::Vec3b(0,255,0));
  assert(costs.at<cv::Vec3b>(0,2) == cv::Vec3b(0,255,255));
  assert(costs.at<cv::Vec3b>(0,3) == cv::Vec3b(0,0,255));
  namespace fs=std::filesystem;
  auto root=fs::temp_directory_path()/("nomad_slope_preview_test_"+std::to_string(getpid()));
  fs::create_directories(root/"123/fd");
  fs::create_symlink("/example/nomad_terrain_mapper",root/"123/exe");
  std::ofstream(root/"terrain.log") << "terrain_mapper_started slope_score_per_deg=8.000 slope_block_deg=10.000\n";
  fs::create_symlink(root/"terrain.log",root/"123/fd/1");
  assert(slope_preview::running_scale(root).max_deg == 10);
  fs::remove(root/"123/exe");
  assert(!slope_preview::running_scale(root).applied);
  fs::remove_all(root);
  std::cout << "slope preview: scale, colours, unknown, signed angles, XY orientation, cost PNG and actual-process report passed\n";
}
