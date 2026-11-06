#include "nomad_costmap/camera_model.hpp"
#include <cmath>
#include <stdexcept>
namespace nomad {
M3 RectifiedCameraModel::accept(const Json& info,const Json& semantic){
  auto require=[](bool condition,const char* reason){if(!condition)throw std::runtime_error(reason);};
  const auto& expected=semantic.at("expected_zed_camera_info");
  require(info.at("frame")==semantic.at("zed_optical_frame"),"Unexpected ZED optical frame");
  require(info.at("width")==expected.at("width")&&info.at("height")==expected.at("height"),"CameraInfo dimensions differ from calibration");
  require(info.at("p").size()==12&&info.at("r").size()==9,"Invalid rectified CameraInfo arrays");
  M3 k;Json values=Json::array();
  for(int row=0;row<3;++row){
    double translation=info.at("p")[row*4+3].get<double>();
    require(std::isfinite(translation)&&std::abs(translation)<=1e-9,"Expected left rectified projection translation zero");
    for(int col=0;col<3;++col){
      double value=info.at("p")[row*4+col].get<double>();
      require(std::isfinite(value)&&std::abs(value-expected.at("K")[row*3+col].get<double>())<=expected.at("pixel_tolerance").get<double>(),"Rectified P intrinsics differ from calibration");
      double rectification=info.at("r")[row*3+col].get<double>();
      require(std::isfinite(rectification)&&std::abs(rectification-(row==col?1.:0.))<=1e-5+(row==col?1e-5:0.),"Expected identity rectification rotation");
      k(row,col)=value;values.push_back(value);
    }
  }
  require(k(0,0)>0&&k(1,1)>0,"Invalid rectified camera focal length");
  require(info.at("distortion_model")==expected.at("distortion_model")&&info.at("d").size()==expected.at("D").size(),"Rectified distortion model differs");
  for(size_t i=0;i<info.at("d").size();++i){double d=info.at("d")[i].get<double>();require(std::isfinite(d)&&std::abs(d-expected.at("D")[i].get<double>())<=1e-8,"Rectified distortion differs from calibration");}
  Json signature={info.at("width"),info.at("height"),values};
  require(signature_.is_null()||signature_==signature,"CameraInfo changed during mapping");
  signature_=std::move(signature);return k;
}
}
