#include "nomad_costmap/indoor.hpp"
#include <cmath>

namespace nomad {
// Amanatides/Woo grid traversal, clipped analytically to the vehicle XY disk.
// Only positive-length intersections count; touching a corner alone is not traversal.
std::vector<Key> trace_ray_cells(const V3& origin,const V3& hit,const V3& vehicle,double resolution,double radius) {
  require(origin.allFinite() && hit.allFinite() && vehicle.allFinite() &&
    std::isfinite(resolution) && resolution>0 && std::isfinite(radius) && radius>0,"Invalid ray traversal inputs");
  const Eigen::Vector2d a=origin.head<2>(),d=hit.head<2>()-a,offset=a-vehicle.head<2>();
  const double length2=d.squaredNorm();
  if(length2<=1e-20) return {};
  const double b=offset.dot(d),c=offset.squaredNorm()-radius*radius;
  const double discriminant=b*b-length2*c;
  if(discriminant<=0.) return {};
  const double root=std::sqrt(discriminant);
  const double begin=std::max(0.,(-b-root)/length2),end=std::min(1.,(-b+root)/length2);
  if(end<=begin+1e-14) return {};
  V3 start=origin+begin*(hit-origin);
  for(int axis=0;axis<2;++axis) if(d[axis]!=0.)
    start[axis]=std::nextafter(start[axis],d[axis]>0?INFINITY:-INFINITY);
  Key key=key_of(start,resolution);
  const int sx=(d.x()>0)-(d.x()<0),sy=(d.y()>0)-(d.y()<0);
  const double dx=sx?resolution/std::abs(d.x()):INFINITY;
  const double dy=sy?resolution/std::abs(d.y()):INFINITY;
  double tx=sx?((double(key.first)+(sx>0))*resolution-a.x())/d.x():INFINITY;
  double ty=sy?((double(key.second)+(sy>0))*resolution-a.y())/d.y():INFINITY;
  double t=begin;
  const size_t limit=size_t(std::ceil(4.*radius/resolution))+16;
  std::vector<Key> keys;
  for(size_t steps=0;steps<limit;++steps) {
    const double next=std::min({tx,ty,end});
    if(next>t+1e-14) keys.push_back(key);
    if(next>=end-1e-14) return keys;
    const bool step_x=tx<=ty+1e-14,step_y=ty<=tx+1e-14;
    if(step_x) {key.first+=sx;tx+=dx;}
    if(step_y) {key.second+=sy;ty+=dy;}
    t=std::max(t,next);
  }
  throw std::runtime_error("Ray traversal exceeded its disk-bounded step limit");
}
}
