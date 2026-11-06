#include "nomad_costmap/core.hpp"
#include "nomad_costmap/camera_model.hpp"
#include <fstream>
#include <iostream>
#include <cfenv>
using namespace nomad;
int main(int argc, char** argv)
{
  std::fesetround(FE_TONEAREST);
  try{
    Json request;
    std::cin >> request;
    Config cfg = Config::load(request.at("config"), request.at("slope_config"));
    std::string op = request.at("op");
    Json out;
    if (op == "tls")
    {
      auto fit = fit_tls(points_from_json(request.at("points")), request.value("weights", std::vector<double>{}), cfg, request.value("weighted", true));
      out = fit?Json{{"normal", {fit->normal.x(), fit->normal.y(), fit->normal.z()}}, {"slope_deg", fit->tilt}, {"offset_m", fit->offset}, {"residual_m", fit->residual}, {"q_space", fit->qspace}}:Json(nullptr);
    }
    else if(op=="camera_info") {
      RectifiedCameraModel model;out=Json::array();
      for(auto& info:request.at("frames")) {
        try {auto k=model.accept(info,cfg.semantic);Json values=Json::array();for(int r=0;r<3;++r)for(int c=0;c<3;++c)values.push_back(k(r,c));out.push_back({{"valid",true},{"K",values}});}
        catch(const std::exception& e){out.push_back({{"valid",false},{"error",e.what()}});}
      }
    }
    else if (op == "filter")
      out = points_json(frame_voxels(points_from_json(request.at("camera")), points_from_json(request.at("points"))));
    else if (op == "factory")
    {
      auto m = factory_mounts(request.at("factory"), cfg.semantic);
      out = {{"imu_b", matrix_json(m[0])}, {"imu_a", matrix_json(m[1])}};
    }
    else if (op == "scan")
      out = scan_pair(request.at("scan"), request.at("cloud"), rigid(request.at("calibration")));
    else if (op == "cost")
      out = build_cost(request.at("live"), request.at("semantics"), cfg);
    else if (op == "project")
    {
      Label l;
      l.width = request.at("label_width");
      l.height = request.at("label_height");
      l.ids = request.at("labels").get<std::vector<uint8_t> >();
      label_safe(l, cfg.semantic.at("boundary_margin_px"));
      Camera c;
      c.world = rigid(request.at("camera_pose"));
      c.width = request.at("image_width");
      c.height = request.at("image_height");
      for (int r = 0; r < 3; ++r)
        for (int k = 0; k < 3; ++k)
          c.k(r, k) = request.at("K")[r][k];
      out = project(points_from_json(request.at("points")), l, c, cfg);
    }
    else if (op == "ground")
    {
      Mounts m;
      for (int i = 0; i < 2; ++i)
        m.sensor[i] = rigid(request.at("mounts")[i]);
      Ground ground(cfg, m);
      if (request.contains("forward_axis"))
        ground.forward_axis = V3(request["forward_axis"][0], request["forward_axis"][1], request["forward_axis"][2]);
      out = Json::array();
      for (auto& input:request.at("frames"))
      {
        Frame f;
        f.stamp = input.at("stamp_ns");
        f.source = input.at("source");
        f.pose = pose_from_json(input.at("pose"));
        f.points = points_from_json(input.at("points"));
        f.stamps = input.value("point_stamps", std::vector<int64_t>(f.points.size(), f.stamp));
        ground.ingest(f, input.value("revision", false));
        out.push_back(ground.snapshot());
      }
    }
    else if (op == "pose" || op == "deskew")
    {
      std::vector<Pose> poses;
      for (auto& row:request.at("poses"))
        poses.push_back(pose_from_json(row));
      if (op == "pose")
        out = pose_json(interpolate(poses, request.at("stamp_ns")));
      else {
        auto frame = deskew(request.at("pair"), poses, rigid(request.at("mount")));
        out = {{"points", points_json(frame.points)}, {"point_stamps", frame.stamps}};
      }
    }
    else if (op == "local")
    {
      Mounts mounts;
      for (int i = 0; i < 2; ++i)
        mounts.sensor[i] = rigid(request.at("mounts")[i]);
      LocalGround local(cfg, mounts);
      M3 k;
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          k(r, c) = request.at("K")[r][c];
      std::deque<Label> labels;
      out = Json::array();
      for (auto& event:request.at("events"))
      {
        std::vector<Pose> poses;
        for (auto& row:event.at("poses"))
          poses.push_back(pose_from_json(row));
        if (event.contains("label"))
        {
          auto input = event.at("label");
          Label label;
          label.stamp = input.at("stamp_ns");
          label.width = input.at("width");
          label.height = input.at("height");
          label.ids = input.at("ids").get<std::vector<uint8_t> >();
          label_safe(label, cfg.semantic.at("boundary_margin_px"));
          labels.push_back(std::move(label));
        }
        for (auto& input:event.at("frames"))
        {
          Frame f;
          f.stamp = input.at("stamp_ns");
          f.source = input.at("source");
          f.points = points_from_json(input.at("points"));
          f.stamps = input.value("point_stamps", std::vector<int64_t>(f.points.size(), f.stamp));
          f.pose = poses.back();
          Points optical = points_from_json(input.value("optical", Json::array()));
          local.submit(std::move(f), &optical);
        }
        Json frames = Json::array();
        for (auto& f:local.process(poses, labels, rigid(request.at("mount")), k, request.at("width"), request.at("height")))
          frames.push_back({{"stamp_ns", f.stamp}, {"source", f.source}, {"points", points_json(f.points)}, {"point_stamps", f.stamps}});
        out.push_back({{"ground", frames}, {"semantic", local.grid.snapshot(poses.back().stamp)}});
      }
    }
    else if (op == "semantic")
    {
      SemanticGrid grid(cfg);
      out = Json::array();
      for (auto& input:request.at("frames"))
      {
        Frame f;
        f.stamp = input.at("stamp_ns");
        f.source = input.at("source");
        f.points = points_from_json(input.at("points"));
        f.stamps = input.value("point_stamps", std::vector<int64_t>(f.points.size(), f.stamp));
        auto ids = input.at("labels").get<std::vector<int> >();
        auto stamps = input.value("label_stamps", f.stamps);
        grid.add(f, ids, stamps, input.value("camera_stamp", f.stamp));
        auto snapshot = grid.snapshot(input.value("now_ns", f.stamp));
        snapshot["version"] = grid.version(input.value("now_ns", f.stamp));
        out.push_back(snapshot);
      }
    }
    else
      throw std::runtime_error("Unknown probe operation");
    std::cout << out.dump() << '\n';
    return 0;
  }
  catch (const std::exception& e){
    std::cerr << e.what() << '\n';
    return 2;
  }
}
