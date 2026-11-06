#pragma once
#include "core.hpp"
namespace nomad {
// The selected ZED input is the LEFT RECTIFIED image. Its projection model
// comes from P, not the raw-camera K field. The first accepted model is frozen
// across invalid messages, matching the preserved Python bridge.
class RectifiedCameraModel {
  Json signature_;
 public:
  M3 accept(const Json& info,const Json& semantic);
};
}
