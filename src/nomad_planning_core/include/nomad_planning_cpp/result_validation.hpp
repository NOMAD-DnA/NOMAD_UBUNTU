#pragma once

#include "nomad_planning_cpp/core.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <string>

namespace nomad_planning_cpp {

// Immutable view of a current ROS grid. Validation walks only the remaining
// route/trajectory; it never copies a full map on the 20 Hz command timer.
struct ResultGridView {
  double resolution;
  int width, height;
  double origin_x, origin_y, origin_yaw;
  const std::vector<std::int8_t>& data;

  Cell cell(double x, double y) const {
    const double dx=x-origin_x,dy=y-origin_y,c=std::cos(origin_yaw),s=std::sin(origin_yaw);
    const double gx=(c*dx+s*dy)/resolution,gy=(-s*dx+c*dy)/resolution;
    if(!std::isfinite(gx)||!std::isfinite(gy)||gx<std::numeric_limits<int>::min()+1.||gy<std::numeric_limits<int>::min()+1.||gx>std::numeric_limits<int>::max()-1.||gy>std::numeric_limits<int>::max()-1.)return {std::numeric_limits<int>::min(),std::numeric_limits<int>::min()};
    return {static_cast<int>(std::floor(gx)),static_cast<int>(std::floor(gy))};
  }
  bool contains(Cell p) const { return p.x>=0&&p.y>=0&&p.x<width&&p.y<height; }
  int value(Cell p) const { return data[static_cast<std::size_t>(p.y)*width+p.x]; }
  Pose centre(Cell p) const {
    const double x=(p.x+.5)*resolution,y=(p.y+.5)*resolution,c=std::cos(origin_yaw),s=std::sin(origin_yaw);
    return {origin_x+c*x-s*y,origin_y+s*x+c*y,0};
  }
};

inline std::string result_grid_contract(const ResultGridView& g) {
  if(!std::isfinite(g.resolution)||g.resolution<=0||g.width<=0||g.height<=0||
      static_cast<std::uint64_t>(g.width)*static_cast<std::uint64_t>(g.height)!=g.data.size()||
      !std::isfinite(g.origin_x)||!std::isfinite(g.origin_y)||!std::isfinite(g.origin_yaw))
    return "MAP_GEOMETRY_INVALID";
  return {};
}

inline Path remaining_result_path(const Path& path,const Pose& current) {
  if(path.empty()||!std::isfinite(current.x)||!std::isfinite(current.y)||!std::isfinite(current.yaw))return {};
  if(path.size()==1)return {current,path.front()};
  double best=kInf;std::size_t next=1;Pose projected=path.front();
  for(std::size_t j=1;j<path.size();++j){
    const auto& a=path[j-1];const auto& b=path[j];
    const double dx=b.x-a.x,dy=b.y-a.y,den=dx*dx+dy*dy;
    const double t=den>0?std::clamp(((current.x-a.x)*dx+(current.y-a.y)*dy)/den,0.,1.):0.;
    Pose p{a.x+t*dx,a.y+t*dy,a.yaw+t*wrap(b.yaw-a.yaw)};
    const double d=distance(current,p);
    if(d<best){best=d;next=j;projected=p;}
  }
  Path out{current,projected};out.insert(out.end(),path.begin()+next,path.end());return out;
}

// Global endpoint must still be observed free. Local trajectory endpoints need
// not be observed twice: unknown transit is controlled by allow_unknown only.
inline std::string validate_remaining_result_path(const ResultGridView& g,const Path& path,
    const Pose& current,bool allow_unknown,bool require_known_endpoint=false,
    double half_length=0,double half_width=0,const Pose* initial_footprint=nullptr,double center_offset=0) {
  auto contract=result_grid_contract(g);if(!contract.empty())return contract;
  if(path.empty())return "NO_PATH";
  for(const auto& p:path)if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.yaw))return "PATH_NONFINITE";
  if(require_known_endpoint){
    const auto end=g.cell(path.back().x,path.back().y);
    if(!g.contains(end))return "PATH_ENDPOINT_OUTSIDE_MAP";
    if(g.value(end)<0)return "PATH_ENDPOINT_UNOBSERVED";
    if(g.value(end)>=80)return "PATH_KNOWN_OBSTACLE";
  }
  const bool footprint=half_length>0&&half_width>0;
  if(footprint&&(!std::isfinite(center_offset)||std::abs(center_offset)>=half_length))return "FOOTPRINT_GEOMETRY_INVALID";
  bool footprint_valid=true;
  auto footprint_cells=[&](const Pose& p){
    std::set<std::pair<int,int>> cells;
    if(!footprint)return cells;
    const double c=std::cos(p.yaw),s=std::sin(p.yaw);
    const double cx=p.x+center_offset*c,cy=p.y+center_offset*s;
    const double map_c=std::cos(g.origin_yaw),map_s=std::sin(g.origin_yaw),dx=cx-g.origin_x,dy=cy-g.origin_y;
    const double px=map_c*dx+map_s*dy,py=-map_s*dx+map_c*dy;
    const double relative=p.yaw-g.origin_yaw,rc=std::cos(relative),rs=std::sin(relative);
    const double ex=std::abs(rc)*half_length+std::abs(rs)*half_width,ey=std::abs(rs)*half_length+std::abs(rc)*half_width;
    const double extent=.5*g.resolution*(std::abs(rc)+std::abs(rs));
    const double x0=std::floor((px-ex)/g.resolution),x1=std::floor((px+ex)/g.resolution),y0=std::floor((py-ey)/g.resolution),y1=std::floor((py+ey)/g.resolution);
    if(!std::isfinite(x0)||!std::isfinite(x1)||!std::isfinite(y0)||!std::isfinite(y1)||x0<std::numeric_limits<int>::min()+1.||y0<std::numeric_limits<int>::min()+1.||x1>std::numeric_limits<int>::max()-1.||y1>std::numeric_limits<int>::max()-1.||(x1-x0+1)*(y1-y0+1)>1000000){footprint_valid=false;return cells;}
    for(int y=static_cast<int>(y0);y<=static_cast<int>(y1);++y)for(int x=static_cast<int>(x0);x<=static_cast<int>(x1);++x){
      const auto centre=g.centre({x,y});const double dx=centre.x-cx,dy=centre.y-cy;
      if(std::abs(c*dx+s*dy)<=half_length+extent&&std::abs(-s*dx+c*dy)<=half_width+extent)cells.emplace(x,y);
    }
    return cells;
  };
  // Match the existing strict observed-guard exception for unknown cells under
  // the starting footprint. Outside cells use only the explicit fixed anchor;
  // known obstacles are never exempt.
  std::set<std::pair<int,int>> initial_unknown;
  if(footprint&&!allow_unknown)for(auto p:footprint_cells(current))if(g.contains({p.first,p.second})&&g.value({p.first,p.second})<0)initial_unknown.insert(p);
  std::set<std::pair<int,int>> initial_outside;
  if(footprint&&initial_footprint)for(auto p:footprint_cells(*initial_footprint))if(!g.contains({p.first,p.second}))initial_outside.insert(p);
  if(!footprint_valid)return "FOOTPRINT_GEOMETRY_INVALID";
  auto permitted=[&](const Pose& p)->std::string{
    const auto cell=g.cell(p.x,p.y);
    if(!g.contains(cell))return "PATH_OUTSIDE_MAP";
    const int value=g.value(cell);
    if(value< -1||value>100)return "MAP_VALUE_INVALID";
    if(value>=80)return "PATH_KNOWN_OBSTACLE";
    if(value<0&&!allow_unknown)return "PATH_UNOBSERVED";
    if(footprint){
      for(auto q:footprint_cells(p)){
        if(!g.contains({q.first,q.second})){if(initial_outside.count(q))continue;return "FOOTPRINT_OUTSIDE_MAP";}
        const int v=g.value({q.first,q.second});
        if(v< -1||v>100)return "MAP_VALUE_INVALID";
        if(v>=80)return "FOOTPRINT_KNOWN_OBSTACLE";
        if(v<0&&!allow_unknown&&!initial_unknown.count(q))return "FOOTPRINT_UNOBSERVED";
      }
      if(!footprint_valid)return "FOOTPRINT_GEOMETRY_INVALID";
    }
    return {};
  };
  const auto remaining=remaining_result_path(path,current);if(remaining.empty())return "PATH_NONFINITE";
  for(const auto& p:remaining)if(!g.contains(g.cell(p.x,p.y)))return "PATH_OUTSIDE_MAP";
  for(std::size_t j=1;j<remaining.size();++j){
    const auto& a=remaining[j-1];const auto& b=remaining[j];
    const double angular=std::abs(wrap(b.yaw-a.yaw))*std::hypot(half_length+std::abs(center_offset),half_width);
    const int steps=std::max(1,static_cast<int>(std::ceil((distance(a,b)+angular)/(g.resolution*.25))));
    for(int n=0;n<=steps;++n){
      const double t=static_cast<double>(n)/steps;
      Pose p{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.yaw+t*wrap(b.yaw-a.yaw)};
      auto why=permitted(p);if(!why.empty())return why;
    }
  }
  return {};
}

} // namespace nomad_planning_cpp
