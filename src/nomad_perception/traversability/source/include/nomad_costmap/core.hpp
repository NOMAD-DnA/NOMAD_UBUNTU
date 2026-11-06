#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <limits>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace nomad {
  using Json = nlohmann::json;
  using V3 = Eigen::Vector3d;
  using M3 = Eigen::Matrix3d;
  using M4 = Eigen::Matrix4d;
  using Points = std::vector<V3>;
  using Key = std::pair<int64_t, int64_t>;
  constexpr double nan = std::numeric_limits<double>::quiet_NaN();
  struct Pose {
    int64_t stamp = 0;
    V3 p = V3::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  };
  struct Frame {
    int64_t stamp = 0;
    int source = 0;
    Points points;
    std::vector<int64_t> stamps;
    Pose pose;
  };
  struct Mounts {
    std::array<M4, 2> sensor{M4::Identity(), M4::Identity()};
  };
  struct Config {
    Json semantic;
    double resolution = .12, radius = .18, window = 3., forward_radius = 3., max_size = 160.;
    double bin = .03, minor = .01, variance_ratio = .02, jump = .12, rms = .01, target_error = .02, tls_gap = 1e-8, xy_rank = 1e-6;
    int min_raw = 12, min_bins = 6, min_heights = 6, raw_reference = 30, per_cell = 512, global_points = 1000000;
    double space_fraction = .1;
    static Config load(const Json&, const Json&);
  };
  M4 rigid(const Json&);
  Json matrix_json(const M4&);
  Json points_json(const Points&);
  Points points_from_json(const Json&);
  Pose pose_from_json(const Json&);
  Json pose_json(const Pose&);
  Pose interpolate(const std::vector<Pose>&, int64_t, int64_t max_gap = 200000000);
  Points transform(const Points&, const M4&);
  double median(std::vector<double>);
  int64_t nearest_even(double);
  struct ROI {
    Mounts mounts;
    Pose pose;
    bool valid = false;
    std::array<Eigen::Vector2d, 2> origins, forward;
    double radius = 3.;
    void set_pose(const Pose&);
    bool contains(const Eigen::Vector2d&, double margin = 0.)const;
  };
  struct Fit {
    V3 normal = V3::Constant(nan);
    double offset = 0., tilt = 0., residual = 0., qspace = 0.;
  };
  std::optional<Fit> fit_tls(const Points&, const std::vector<double>&, const Config&, bool weighted = true);
  Points frame_voxels(const Points& camera, const Points& world);
  struct Estimate {
    bool valid = false, reused = false;
    double height = nan, confidence = 0.;
    Fit fit;
    int raw_count = 0;
    int64_t last_observed = 0;
  };
  struct Capture {
    int64_t stamp;
    Points points;
  };
  struct SampleCell {
    std::array<std::deque<Capture>, 2> sources;
  };
  class Ground {
public:
    Config cfg;
    ROI roi;
    std::map<Key, SampleCell> samples;
    std::vector<Key> insertion;
    std::set<Key> observed, dirty, previous_active;
    std::map<Key, Estimate> estimates;
    std::map<Key, std::pair<double, int64_t> > heights;
    std::map<Key, int64_t> last_observed, calculated;
    std::array<int64_t, 2> last_stamp{-1, -1}, last_bucket{-1, -1}, source_capture{0, 0};
    int64_t latest = -1;
    uint64_t revisions = 0;
    Json counters = Json::object();
    V3 forward_axis = V3::UnitX();
    Ground(const Config&, const Mounts&);
    void ingest(const Frame&, bool revision = false);
    Json snapshot();
private:
    struct Rep {
      V3 mean;
      Key bin;
      int count;
      double span;
    };
    std::vector<Rep> prepare()const;
    std::optional<Fit> refit(Key)const;
  };
  struct Evidence {
    int64_t last = 0;
    std::array<uint64_t, 2> votes{0, 0};
  };
  struct SemanticCell {
    int64_t geometry = 0, camera = 0;
    std::vector<int64_t> captures;
    std::map<int, Evidence> classes;
    std::vector<int> class_order;
  };
  class SemanticGrid {
public:
    Config cfg;
    std::map<Key, SemanticCell> cells;
    uint64_t revision = 0;
    using Expiry = std::tuple<int64_t, Key, int, int64_t>;
    std::priority_queue<Expiry, std::vector<Expiry>, std::greater<Expiry> > expirations;
    explicit SemanticGrid(const Config& c) : cfg(c)
    {
    }
    void add(const Frame&, const std::vector<int>&, const std::vector<int64_t>&, int64_t camera);
    uint64_t version(int64_t now);
    Json snapshot(int64_t now)const;
  };
  struct Camera {
    M4 world = M4::Identity();
    M3 k = M3::Identity();
    int width = 1280, height = 720;
  };
  struct Label {
    int64_t stamp = 0;
    int width = 640, height = 360;
    std::vector<uint8_t> ids, safe;
  };
  void label_safe(Label&, int margin);
  std::vector<int> project(const Points&, const Label&, const Camera&, const Config&);
  struct Geometry {
    Frame frame;
    std::vector<int> ids;
    std::vector<int64_t> label_stamps;
    int64_t attempt = -1;
    std::optional<std::vector<uint8_t> > emitted;
  };
  class LocalGround {
public:
    Config cfg;
    Mounts mounts;
    SemanticGrid grid;
    std::array<std::deque<Geometry>, 2> frames;
    std::array<int64_t, 2> last{-1, -1}, bucket{-1, -1};
    uint64_t sequence = 0;
    int64_t last_stamp = 0;
    LocalGround(const Config& c, const Mounts& m) : cfg(c), mounts(m), grid(c)
    {
    }
    void submit(Frame, const Points* camera = nullptr);
    std::vector<Frame> process(const std::vector<Pose>&, const std::deque<Label>&, const M4& mount, const M3& k, int width, int height);
  };
  Json build_cost(const Json& slope, const Json& semantic, const Config&);
  std::array<M4, 2> factory_mounts(const Json& factory, const Json& config);
  Mounts sensor_mounts(const M4& imu_b, const M4& b_lidar, const M4& imu_a);
  Json scan_pair(const Json& scan, const Json& cloud, const M4& camera_lidar);
  Frame deskew(const Json& pair, const std::vector<Pose>&, const M4& imu_lidar, double offset_s = 0.);
}
