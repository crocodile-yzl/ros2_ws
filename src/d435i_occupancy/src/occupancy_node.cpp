#include "d435i_occupancy/voxel_map.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
namespace d435i_occupancy {
class OccupancyNode:public rclcpp::Node {
public:
 OccupancyNode():Node("d435i_occupancy"){
  options_.resolution=declare_parameter("resolution",.1);
  options_.minimum={declare_parameter("minimum_x",-.5),declare_parameter("minimum_y",-1.5),declare_parameter("minimum_z",-.2)};
  options_.maximum={declare_parameter("maximum_x",4.),declare_parameter("maximum_y",1.5),declare_parameter("maximum_z",2.)};
  height_=declare_parameter("camera_height",.9);pitch_=declare_parameter("camera_pitch_deg",34.);
  stride_=declare_parameter("pixel_stride",4);double hz=declare_parameter("update_rate",20.);
  minimum_depth_=declare_parameter("minimum_depth",.2);maximum_depth_=declare_parameter("maximum_depth",3.);
  obstacle_minimum_=declare_parameter("obstacle_minimum_height",.15);obstacle_maximum_=declare_parameter("obstacle_maximum_height",1.8);
  frame_=declare_parameter("map_frame",std::string("occupancy_map"));
  const auto depth_topic=declare_parameter("depth_topic",std::string("/camera/depth/image_rect_raw"));
  const auto info_topic=declare_parameter("camera_info_topic",std::string("/camera/depth/camera_info"));
  if(stride_<1||!std::isfinite(hz)||hz<=0||hz>60||!std::isfinite(height_)||height_<0||!std::isfinite(pitch_)||std::abs(pitch_)>90||!(minimum_depth_>0&&minimum_depth_<maximum_depth_&&std::isfinite(maximum_depth_))||!(std::isfinite(obstacle_minimum_)&&std::isfinite(obstacle_maximum_)&&obstacle_minimum_<obstacle_maximum_)||frame_.empty())throw std::invalid_argument("invalid occupancy parameters");
  map_=std::make_unique<VoxelMap>(options_);
  reference_=create_publisher<visualization_msgs::msg::MarkerArray>("/occupancy/camera_reference",rclcpp::QoS(1).transient_local());
  publish_reference();
  occupied_=create_publisher<sensor_msgs::msg::PointCloud2>("/occupancy/occupied",rclcpp::QoS(1).transient_local());
  free_=create_publisher<sensor_msgs::msg::PointCloud2>("/occupancy/free",rclcpp::QoS(1).transient_local());
  obstacles_=create_publisher<sensor_msgs::msg::PointCloud2>("/occupancy/obstacles",rclcpp::QoS(1).transient_local());
  projection_=create_publisher<nav_msgs::msg::OccupancyGrid>("/occupancy/projection",rclcpp::QoS(1).transient_local());
  frames_=create_publisher<std_msgs::msg::UInt64>("/occupancy/processed_frames",10);
  info_sub_=create_subscription<sensor_msgs::msg::CameraInfo>(info_topic,rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::CameraInfo::ConstSharedPtr p){info_=p;});
  image_sub_=create_subscription<sensor_msgs::msg::Image>(depth_topic,rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::Image::ConstSharedPtr p){
   if(last_received_&&last_received_->header.frame_id==p->header.frame_id&&last_received_->header.stamp==p->header.stamp)return;
   latest_=p;last_received_=p;
  });
  timer_=create_wall_timer(std::chrono::duration<double>(1./hz),[this]{process();});
  RCLCPP_INFO(get_logger(),"Fixed camera occupancy: height %.2f m, pitch %.1f deg, voxel %.2f m, %.1f Hz, stride %d",height_,pitch_,options_.resolution,hz,stride_);
 }
private:
 void publish_reference(){
  using Marker=visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray markers;
  auto make=[&](int id,int type,float r,float g,float b){
   Marker m;m.header.frame_id=frame_;m.ns="camera_reference";m.id=id;m.type=type;
   m.action=Marker::ADD;m.pose.orientation.w=1.;m.color.r=r;m.color.g=g;m.color.b=b;m.color.a=1.;
   return m;
  };
  auto point=[](double x,double y,double z){geometry_msgs::msg::Point p;p.x=x;p.y=y;p.z=z;return p;};
  auto camera=make(0,Marker::SPHERE,0.,.9,1.);camera.pose.position=point(0,0,height_);
  camera.scale.x=camera.scale.y=camera.scale.z=.14;markers.markers.push_back(camera);
  const double angle=pitch_*std::acos(-1.)/180.;
  auto direction=make(1,Marker::ARROW,.2,1.,.3);
  direction.points={point(0,0,height_),point(std::cos(angle),0,height_-std::sin(angle))};
  direction.scale.x=.025;direction.scale.y=.07;direction.scale.z=.12;markers.markers.push_back(direction);
  auto height=make(2,Marker::LINE_LIST,0.,.9,1.);height.scale.x=.015;
  height.points={point(0,0,0),point(0,0,height_)};markers.markers.push_back(height);
  auto ground=make(3,Marker::SPHERE,0.,.9,1.);ground.pose.position=point(0,0,.01);
  ground.scale.x=ground.scale.y=.12;ground.scale.z=.02;markers.markers.push_back(ground);
  auto label=[&](int id,const std::string&text,geometry_msgs::msg::Point position,float r,float g,float b){
   auto m=make(id,Marker::TEXT_VIEW_FACING,r,g,b);m.text=text;m.pose.position=position;m.scale.z=.12;
   markers.markers.push_back(m);
  };
  label(4,"D435i camera",point(0,0,height_+.20),0.,.9,1.);
  label(5,"X forward",point(.8,0,.12),1.,.3,.3);
  label(6,"Y left",point(0,.8,.12),.3,1.,.3);
  label(7,"Z up",point(0,0,1.5),.4,.6,1.);
  label(8,"Ground z=0",point(-.3,-.5,.06),.8,.8,.8);
  reference_->publish(markers);
 }
 bool valid(const sensor_msgs::msg::Image&i,const sensor_msgs::msg::CameraInfo&c){
  const int bytes=(i.encoding=="32FC1")?4:((i.encoding=="16UC1"||i.encoding=="mono16")?2:0);
  if(!bytes||!i.width||!i.height||i.width>16384||i.height>16384||i.step<uint64_t(i.width)*bytes||i.data.size()<uint64_t(i.step)*i.height)return false;
  if(c.width!=i.width||i.header.frame_id.empty()||c.header.frame_id!=i.header.frame_id)return false;
  // Berxel sets CameraInfo.height=0 and binning_x=height. Use Image dimensions;
  // its K is already calibrated for the delivered depth image, without binning.
  for(double v:c.k)if(!std::isfinite(v))return false;
  return c.k[0]>0&&c.k[4]>0&&c.k[2]>=0&&c.k[2]<i.width&&c.k[5]>=0&&c.k[5]<i.height&&std::abs(c.k[8]-1.)<1e-6;
 }
 sensor_msgs::msg::PointCloud2 cloud(const std::vector<Point>&points,const std_msgs::msg::Header&header){
  sensor_msgs::msg::PointCloud2 result;result.header=header;
  sensor_msgs::PointCloud2Modifier modifier(result);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(result,"x"),y(result,"y"),z(result,"z");
  for(const auto&p:points){*x=p[0];*y=p[1];*z=p[2];++x;++y;++z;}
  return result;
 }
 void process(){
  if(!latest_||!info_)return;
  auto image=std::move(latest_);latest_.reset();
  if(!valid(*image,*info_)){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Rejected depth: check encoding, buffer, finite K, width and matching optical frame");return;}
  std::vector<Point> endpoints;endpoints.reserve((image->width/stride_+1)*(image->height/stride_+1));
  const int bytes=image->encoding=="32FC1"?4:2;
  for(unsigned v=0;v<image->height;v+=stride_)for(unsigned u=0;u<image->width;u+=stride_){
   const auto*p=image->data.data()+std::size_t(v)*image->step+u*bytes;
   double depth=decode_depth(p,image->encoding,image->is_bigendian);
   // Invalid, too-near and beyond-range returns provide no clearing evidence.
   if(!std::isfinite(depth)||depth<minimum_depth_||depth>maximum_depth_)continue;
   endpoints.push_back(optical_to_map({(u-info_->k[2])*depth/info_->k[0],(v-info_->k[5])*depth/info_->k[4],depth},height_,pitch_));
  }
  const auto begin=std::chrono::steady_clock::now();
  map_->integrate({0,0,height_},endpoints);
  auto occupied=map_->centers(State::Occupied),free=map_->centers(State::Free);
  std::vector<Point> obstacles;for(const auto&p:occupied)if(p[2]>=obstacle_minimum_&&p[2]<=obstacle_maximum_)obstacles.push_back(p);
  std_msgs::msg::Header header;header.frame_id=frame_;header.stamp=image->header.stamp;
  occupied_->publish(cloud(occupied,header));free_->publish(cloud(free,header));obstacles_->publish(cloud(obstacles,header));
  nav_msgs::msg::OccupancyGrid grid;grid.header=header;grid.info.resolution=options_.resolution;
  grid.info.width=std::ceil((options_.maximum[0]-options_.minimum[0])/options_.resolution);
  grid.info.height=std::ceil((options_.maximum[1]-options_.minimum[1])/options_.resolution);
  grid.info.origin.position.x=options_.minimum[0];grid.info.origin.position.y=options_.minimum[1];grid.info.origin.orientation.w=1.;grid.data.assign(grid.info.width*grid.info.height,-1);
  grid.data=map_->projection(obstacle_minimum_,obstacle_maximum_);
  projection_->publish(grid);
  std_msgs::msg::UInt64 count;count.data=++processed_;frames_->publish(count);
  RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"frames=%lu rays=%zu occupied=%zu obstacle=%zu observed=%zu map+publish=%.1f ms",static_cast<unsigned long>(processed_),endpoints.size(),occupied.size(),obstacles.size(),map_->observed_count(),std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count());
 }
 Options options_;std::unique_ptr<VoxelMap> map_;
 double height_,pitch_,minimum_depth_,maximum_depth_,obstacle_minimum_,obstacle_maximum_;int stride_;std::string frame_;uint64_t processed_=0;
 sensor_msgs::msg::Image::ConstSharedPtr latest_,last_received_;
 sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
 rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
 rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
 rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_,free_,obstacles_;
 rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr projection_;
 rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr frames_;
 rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr reference_;
 rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<d435i_occupancy::OccupancyNode>());}catch(const std::exception&e){RCLCPP_FATAL(rclcpp::get_logger("d435i_occupancy"),"%s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
