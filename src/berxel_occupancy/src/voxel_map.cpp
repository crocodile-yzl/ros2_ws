#include "berxel_occupancy/voxel_map.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <unordered_set>
namespace berxel_occupancy {
namespace {
bool finite(const Point&p){return std::all_of(p.begin(),p.end(),[](double v){return std::isfinite(v);});}
}
Point optical_to_map(Point p,double height,double pitch_degrees){
 const double angle=pitch_degrees*std::acos(-1.)/180., c=std::cos(angle),s=std::sin(angle);
 return {c*p[2]-s*p[1],-p[0],height-s*p[2]-c*p[1]};
}
double decode_depth(const uint8_t*p,const std::string& encoding,bool big_endian){
 double depth=std::numeric_limits<double>::quiet_NaN();
 if(encoding=="mono16"||encoding=="16UC1"){
  const unsigned value=big_endian?(unsigned(p[0])<<8|p[1]):(unsigned(p[1])<<8|p[0]);depth=value*.001;
 }else if(encoding=="32FC1"){
  uint32_t bits=0;for(int i=0;i<4;++i)bits|=uint32_t(p[i])<<(8*(big_endian?3-i:i));
  float value;std::memcpy(&value,&bits,4);depth=value;
 }
 return std::isfinite(depth)&&depth>0?depth:std::numeric_limits<double>::quiet_NaN();
}
VoxelMap::VoxelMap(Options options):options_(options){
 if(!std::isfinite(options.resolution)||options.resolution<=0||!finite(options.minimum)||!finite(options.maximum))throw std::invalid_argument("invalid map bounds/resolution");
 std::size_t total=1;
 for(int a=0;a<3;++a){
  const double count=std::ceil((options.maximum[a]-options.minimum[a])/options.resolution);
  if(count<1||count>1000000)throw std::invalid_argument("invalid map dimension");
  dimensions_[a]=static_cast<int>(count);total*=dimensions_[a];
  if(total>10000000)throw std::invalid_argument("map exceeds 10 million voxel limit");
 }
 if(!(std::isfinite(options.hit)&&std::isfinite(options.miss)&&options.hit>0&&options.miss<0&&options.clamp_min<options.free&&options.free<0&&options.occupied>0&&options.occupied<options.clamp_max&&std::isfinite(options.clamp_min)&&std::isfinite(options.clamp_max)))throw std::invalid_argument("invalid evidence thresholds");
 evidence_.assign(total,0);observed_.assign(total,false);
}
int VoxelMap::index(Point p)const{
 if(!finite(p))return -1;
 std::array<int,3> cell{};
 for(int a=0;a<3;++a){if(p[a]<options_.minimum[a]||p[a]>=options_.maximum[a])return -1;cell[a]=int(std::floor((p[a]-options_.minimum[a])/options_.resolution));}
 return (cell[2]*dimensions_[1]+cell[1])*dimensions_[0]+cell[0];
}
Point VoxelMap::center(int id)const{
 std::array<int,3> cell{id%dimensions_[0],(id/dimensions_[0])%dimensions_[1],id/(dimensions_[0]*dimensions_[1])};
 Point p{};for(int a=0;a<3;++a){const double lower=options_.minimum[a]+cell[a]*options_.resolution;p[a]=.5*(lower+std::min(lower+options_.resolution,options_.maximum[a]));}return p;
}
std::vector<int> VoxelMap::ray(Point origin,Point endpoint,bool& hit)const{
 hit=index(endpoint)>=0;
 if(!finite(origin)||!finite(endpoint))return {};
 Point delta{};double enter=0,leave=1;
 for(int a=0;a<3;++a){
  delta[a]=endpoint[a]-origin[a];
  if(delta[a]==0){if(origin[a]<options_.minimum[a]||origin[a]>=options_.maximum[a])return {};continue;}
  double lo=(options_.minimum[a]-origin[a])/delta[a],hi=(options_.maximum[a]-origin[a])/delta[a];
  if(lo>hi)std::swap(lo,hi);
  enter=std::max(enter,lo);leave=std::min(leave,hi);
 }
 if(enter>leave||leave<0||enter>1)return {};
 Point start{},finish{};
 for(int a=0;a<3;++a){
  start[a]=std::clamp(origin[a]+delta[a]*enter,options_.minimum[a],std::nextafter(options_.maximum[a],options_.minimum[a]));
  finish[a]=std::clamp(origin[a]+delta[a]*leave,options_.minimum[a],std::nextafter(options_.maximum[a],options_.minimum[a]));
 }
 int id=index(start),last=index(finish);if(id<0||last<0)return {};
 std::array<int,3> cell{},step{};Point crossing{},increment{};
 for(int a=0;a<3;++a){
  cell[a]=int(std::floor((start[a]-options_.minimum[a])/options_.resolution));
  step[a]=(delta[a]>0)-(delta[a]<0);
  if(step[a]==0){crossing[a]=increment[a]=std::numeric_limits<double>::infinity();continue;}
  const double boundary=options_.minimum[a]+(cell[a]+(step[a]>0?1:0))*options_.resolution;
  crossing[a]=(boundary-origin[a])/delta[a];increment[a]=options_.resolution/std::abs(delta[a]);
 }
 std::vector<int> cells;cells.reserve(dimensions_[0]+dimensions_[1]+dimensions_[2]);
 const int limit=dimensions_[0]+dimensions_[1]+dimensions_[2]+3;
 for(int count=0;count<limit;++count){
  cells.push_back(id);if(id==last)break;
  const double next=std::min({crossing[0],crossing[1],crossing[2]});
  for(int a=0;a<3;++a)if(crossing[a]<=next+1e-12){cell[a]+=step[a];crossing[a]+=increment[a];}
  if(cell[0]<0||cell[0]>=dimensions_[0]||cell[1]<0||cell[1]>=dimensions_[1]||cell[2]<0||cell[2]>=dimensions_[2])break;
  id=(cell[2]*dimensions_[1]+cell[1])*dimensions_[0]+cell[0];
 }
 return cells;
}
void VoxelMap::integrate(const Point&origin,const std::vector<Point>&endpoints){
 std::unordered_set<int> free_cells,hit_cells;
 for(const auto&endpoint:endpoints){bool hit=false;auto cells=ray(origin,endpoint,hit);if(cells.empty())continue;
  if(hit){hit_cells.insert(cells.back());cells.pop_back();}for(int id:cells)free_cells.insert(id);
 }
 auto update=[&](int id,double value){observed_[id]=true;evidence_[id]=std::clamp(double(evidence_[id])+value,options_.clamp_min,options_.clamp_max);};
 for(int id:free_cells)if(hit_cells.count(id)==0)update(id,options_.miss);
 for(int id:hit_cells)update(id,options_.hit);
}
State VoxelMap::state(Point p)const{int id=index(p);if(id<0||!observed_[id])return State::Unknown;if(evidence_[id]>=options_.occupied)return State::Occupied;if(evidence_[id]<=options_.free)return State::Free;return State::Uncertain;}
double VoxelMap::evidence(Point p)const{int id=index(p);return id<0?0:evidence_[id];}
std::vector<Point> VoxelMap::centers(State wanted)const{std::vector<Point> result;for(std::size_t id=0;id<observed_.size();++id)if(observed_[id]&&state(center(id))==wanted)result.push_back(center(id));return result;}
std::vector<int8_t> VoxelMap::projection(double minimum_height,double maximum_height)const{
 std::vector<int8_t> result(dimensions_[0]*dimensions_[1],-1);
 for(int y=0;y<dimensions_[1];++y)for(int x=0;x<dimensions_[0];++x){
  bool all_free=true,any_height=false,occupied=false;
  for(int z=0;z<dimensions_[2];++z){
   int id=(z*dimensions_[1]+y)*dimensions_[0]+x;auto p=center(id);
   if(p[2]<minimum_height||p[2]>maximum_height)continue;
   any_height=true;auto value=state(p);
   if(value==State::Occupied){occupied=true;break;}
   if(value!=State::Free)all_free=false;
  }
  result[y*dimensions_[0]+x]=occupied?100:(any_height&&all_free?0:-1);
 }
 return result;
}
std::size_t VoxelMap::observed_count()const{return std::count(observed_.begin(),observed_.end(),true);}
}
