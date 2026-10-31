#include "berxel_occupancy/voxel_map.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <cstring>
using namespace berxel_occupancy;
void require(bool b,const char *m) {if(!b)throw std::runtime_error(m);}
int main(){
 int failures=0;
 auto test=[&](const char *name,auto body){try {body();std::cout<<"PASS "<<name<<'\n';}catch(const std::exception&e){++failures;std::cerr<<"FAIL "<<name<<": "<<e.what()<<'\n';}};
 Options o; o.resolution=1.;o.minimum={0,0,0};o.maximum={5,5,5};
 test("unknown distinct",[&]{VoxelMap m(o);require(m.state({1,1,1})==State::Unknown,"unobserved must be unknown");});
 test("ray frees interiors and hits endpoint",[&]{VoxelMap m(o);m.integrate({.5,.5,.5},{{3.5,.5,.5}});require(m.state({2.5,.5,.5})==State::Free,"interior free");require(m.state({3.5,.5,.5})==State::Occupied,"endpoint hit");require(m.state({4.5,.5,.5})==State::Unknown,"behind remains unknown");});
 test("diagonal DDA",[&]{VoxelMap m(o);m.integrate({.5,.5,.5},{{3.5,3.5,3.5}});require(m.state({1.5,1.5,1.5})==State::Free,"diagonal interior free");require(m.state({1.5,.5,.5})==State::Unknown,"corner-only voxel unknown");});
 test("clipped ray is free without border hit",[&]{VoxelMap m(o);m.integrate({.5,.5,.5},{{8.5,.5,.5}});require(m.state({4.5,.5,.5})==State::Free,"clipped border free");require(m.observed_count()==5,"all traversed voxels observed");});
 test("outside origin clips",[&]{VoxelMap m(o);m.integrate({-2.,.5,.5},{{2.5,.5,.5}});require(m.state({.5,.5,.5})==State::Free,"entry free");require(m.state({2.5,.5,.5})==State::Occupied,"endpoint hit");});
 test("per-frame hit precedence and deduplication",[&]{VoxelMap m(o);m.integrate({.5,.5,.5},{{1.5,.5,.5},{1.5,.5,.5},{3.5,.5,.5}});require(std::abs(m.evidence({1.5,.5,.5})-.85)<1e-6,"one hit dominates all misses");});
 test("clearing and clamp",[&]{VoxelMap m(o);for(int n=0;n<100;++n)m.integrate({.5,.5,.5},{{1.5,.5,.5}});require(std::abs(m.evidence({1.5,.5,.5})-3.5)<1e-6,"upper clamp");for(int n=0;n<100;++n)m.integrate({.5,.5,.5},{{3.5,.5,.5}});require(m.state({1.5,.5,.5})==State::Free,"clear previous hit");require(std::abs(m.evidence({1.5,.5,.5})+2.)<1e-6,"lower clamp");});
 test("nonfinite endpoints never clear",[&]{VoxelMap m(o);m.integrate({.5,.5,.5},{{1.5,.5,.5}});double before=m.evidence({1.5,.5,.5});m.integrate({.5,.5,.5},{{NAN,.5,.5},{INFINITY,.5,.5}});require(before>0 && m.evidence({1.5,.5,.5})==before,"no invalid rays");});
 test("optical transform height pitch",[&]{auto p=optical_to_map({0,0,1},.9,34.);require(std::abs(p[0]-.82903757)<1e-6,"forward rotation");require(std::abs(p[2]-.34080710)<1e-6,"down pitch");auto q=optical_to_map({1,0,0},.9,34.);require(q[1]==-1,"optical right becomes map right");});
 test("depth encoding and invalid values",[&]{uint8_t mm[]={0xe8,3};require(std::abs(decode_depth(mm,"mono16",false)-1.)<1e-8,"mm to meters");uint8_t big[]={3,0xe8};require(decode_depth(big,"16UC1",true)==1.,"big endian");uint8_t zero[]={0,0};require(std::isnan(decode_depth(zero,"mono16",false)),"zero invalid");float value=NAN;uint8_t bytes[4];std::memcpy(bytes,&value,4);require(std::isnan(decode_depth(bytes,"32FC1",false)),"nan invalid");value=1.25;std::memcpy(bytes,&value,4);require(decode_depth(bytes,"32FC1",false)==1.25,"float meters");});
 test("partial boundary voxel remains published",[&]{Options partial=o;partial.maximum[0]=4.1;VoxelMap m(partial);m.integrate({.5,.5,.5},{{4.05,.5,.5}});require(m.centers(State::Occupied).size()==1,"partial final cell must appear");require(m.centers(State::Occupied)[0][0]<4.1,"published center inside bounds");});
 test("conservative projection requires full height observation",[&]{VoxelMap m(o);m.integrate({.5,.5,.5},{{3.5,.5,.5}});auto p=m.projection(0,2);require(p.size()==25,"projection dimensions");require(p[1]==-1,"one free voxel does not free whole column");require(p[3]==100,"occupied dominates unknown");m.integrate({.5,.5,1.5},{{3.5,.5,1.5}});require(m.projection(0,2)[1]==0,"all included heights free");});
 return failures?1:0;
}
