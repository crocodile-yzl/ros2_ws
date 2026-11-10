#pragma once
#include <array>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string>
namespace d435i_occupancy {
using Point = std::array<double,3>;
enum class State { Unknown, Free, Uncertain, Occupied };
struct Options {
 double resolution=.1;
 Point minimum{-.5,-2.,-.2}, maximum{4.,2.,2.};
 double hit=.85, miss=-.4, clamp_min=-2., clamp_max=3.5, occupied=.7, free=-.2;
};
Point optical_to_map(Point p,double height,double pitch_degrees);
double decode_depth(const uint8_t *p,const std::string &encoding,bool big_endian);
class VoxelMap {
public:
 explicit VoxelMap(Options options={});
 void integrate(const Point &origin,const std::vector<Point>& endpoints);
 State state(Point p) const;
 double evidence(Point p) const;
 std::vector<Point> centers(State state) const;
 std::size_t observed_count() const;
 std::vector<int8_t> projection(double minimum_height,double maximum_height) const;
private:
 Options options_;
 std::array<int,3> dimensions_{};
 std::vector<float> evidence_;
 std::vector<bool> observed_;
 int index(Point p) const;
 Point center(int index) const;
 std::vector<int> ray(Point origin,Point endpoint,bool &hit) const;
};
}
