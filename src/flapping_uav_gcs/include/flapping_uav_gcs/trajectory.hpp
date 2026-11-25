#ifndef FLAPPING_UAV_GCS__TRAJECTORY_HPP_
#define FLAPPING_UAV_GCS__TRAJECTORY_HPP_
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
namespace flapping_uav_gcs
{
struct TrajectoryConfig
{
  std::string mode{"line"};std::array<double,2> center{{0,0}},half_extents{{1.5,0}};
  std::vector<std::array<double,2>> boundary;
  double margin{.8},period_s{40},ramp_s{3},max_speed{.5},max_acceleration{.4},min_approach_s{3};
  int cycles{1};
  std::string figure8_axis{"x"}; // Axis along which the two lobes are arranged.
};
struct TrajectorySample
{
  std::array<double,2> position{{0,0}},velocity{{0,0}},acceleration{{0,0}};
  std::string phase;bool finished{false};
};
class HorizontalTrajectory
{
public:
  explicit HorizontalTrajectory(TrajectoryConfig c):c_(c)
  {
    if((c.mode!="line"&&c.mode!="figure8") || !finite(c.center) || !finite(c.half_extents) ||
      c.half_extents[0]<=0 || (c.mode=="figure8"&&c.half_extents[1]<=0) || c.half_extents[1]<0 ||
      !positive(c.margin)||!positive(c.period_s)||!positive(c.ramp_s)||!positive(c.max_speed)||
      !positive(c.max_acceleration)||!positive(c.min_approach_s)||c.cycles<1||c.boundary.size()<3||
      (c.figure8_axis!="x"&&c.figure8_axis!="y")){
      throw std::invalid_argument("Invalid horizontal trajectory configuration");}
    for(size_t i=0;i<c.boundary.size();++i){
      const auto & a=c.boundary[i];const auto & b=c.boundary[(i+1)%c.boundary.size()];
      const auto & d=c.boundary[(i+2)%c.boundary.size()];
      if(!finite(a)||std::hypot(b[0]-a[0],b[1]-a[1])<1e-6 ||
        (b[0]-a[0])*(d[1]-b[1])-(b[1]-a[1])*(d[0]-b[0])<=0){
        throw std::invalid_argument("Trajectory boundary must be strictly convex and counterclockwise");}
    }
    const double hy=c.mode=="line"?0:c.half_extents[1];
    for(double sx:{-1.,1.}){for(double sy:{-1.,1.}){
      if(!inside({{c.center[0]+sx*c.half_extents[0],c.center[1]+sy*hy}})){
        throw std::invalid_argument("Trajectory envelope exceeds anchor boundary margin");}}}
    const bool along_y=c.figure8_axis=="y";
    const double d=c.mode=="line"?c.half_extents[0]:
      std::hypot((along_y?2:1)*c.half_extents[0],(along_y?1:2)*c.half_extents[1]);
    const double curvature=c.mode=="line"?c.half_extents[0]:
      std::hypot((along_y?4:1)*c.half_extents[0],(along_y?1:4)*c.half_extents[1]);
    const double b=1.5*d/c.ramp_s;
    const double accel_limit=2*c.max_acceleration/(b+std::sqrt(b*b+4*curvature*c.max_acceleration));
    omega_=std::min({2*pi()/c.period_s,c.max_speed/d,accel_limit,2*pi()*c.cycles/c.ramp_s});
    duration_=2*pi()*c.cycles/omega_+c.ramp_s;
  }
  bool inside(const std::array<double,2> & p) const
  {
    if(!finite(p)){return false;}
    for(size_t i=0;i<c_.boundary.size();++i){const auto & a=c_.boundary[i];const auto & b=c_.boundary[(i+1)%c_.boundary.size()];
      const double dx=b[0]-a[0],dy=b[1]-a[1];
      if((dx*(p[1]-a[1])-dy*(p[0]-a[0]))/std::hypot(dx,dy)<c_.margin-1e-9){return false;}}
    return true;
  }
  std::array<double,2> start_point() const{return point(0);}
  std::array<double,2> point(double phase) const
  {
    if(c_.mode=="line"){return {{c_.center[0]-c_.half_extents[0]*std::cos(phase),c_.center[1]}};}
    if(c_.figure8_axis=="y"){return {{c_.center[0]+c_.half_extents[0]*std::sin(2*phase),c_.center[1]+c_.half_extents[1]*std::sin(phase)}};}
    return {{c_.center[0]+c_.half_extents[0]*std::sin(phase),c_.center[1]+c_.half_extents[1]*std::sin(2*phase)}};
  }
  void begin(const std::array<double,2> & position)
  {
    if(!inside(position)){throw std::invalid_argument("Current position is outside the configured trajectory boundary margin");}
    initial_=position;const auto end=start_point();const double distance=std::hypot(end[0]-position[0],end[1]-position[1]);
    approach_=distance<1e-8?0:std::max({c_.min_approach_s,1.875*distance/c_.max_speed,std::sqrt(5.773502692*distance/c_.max_acceleration)});
    started_=true;
  }
  double period()const{return 2*pi()/omega_;}
  double approach_duration()const{return approach_;}
  double total_duration()const{return approach_+duration_;}
  TrajectorySample sample(double elapsed) const
  {
    if(!started_||!std::isfinite(elapsed)){throw std::logic_error("Trajectory must be started with finite time");}
    elapsed=std::max(0.,elapsed);TrajectorySample out;
    if(elapsed<approach_){
      out.phase="APPROACH";const auto end=start_point();const double u=elapsed/approach_;
      const double s=u*u*u*(10+u*(-15+6*u));const double ds=30*u*u*(1-u)*(1-u)/approach_;
      const double dds=60*u*(1-u)*(1-2*u)/(approach_*approach_);
      for(size_t i=0;i<2;++i){const double delta=end[i]-initial_[i];out.position[i]=initial_[i]+delta*s;
        out.velocity[i]=delta*ds;out.acceleration[i]=delta*dds;}
      return out;
    }
    const double t=elapsed-approach_;
    if(t>=duration_){out.phase="FINISHED";out.finished=true;out.position=start_point();return out;}
    out.phase="TRACKING";double phase,speed,acceleration;
    if(t<c_.ramp_s){const double u=t/c_.ramp_s;
      phase=omega_*c_.ramp_s*(u*u*u-.5*u*u*u*u);speed=omega_*u*u*(3-2*u);acceleration=omega_*6*u*(1-u)/c_.ramp_s;
    }else if(t<=duration_-c_.ramp_s){phase=omega_*(t-c_.ramp_s/2);speed=omega_;acceleration=0;
    }else{const double u=(duration_-t)/c_.ramp_s;
      phase=2*pi()*c_.cycles-omega_*c_.ramp_s*(u*u*u-.5*u*u*u*u);speed=omega_*u*u*(3-2*u);acceleration=-omega_*6*u*(1-u)/c_.ramp_s;}
    out.position=point(phase);std::array<double,2> derivative,second;
    if(c_.mode=="line"){derivative={{c_.half_extents[0]*std::sin(phase),0}};second={{c_.half_extents[0]*std::cos(phase),0}};
    }else if(c_.figure8_axis=="y"){
      derivative={{2*c_.half_extents[0]*std::cos(2*phase),c_.half_extents[1]*std::cos(phase)}};
      second={{-4*c_.half_extents[0]*std::sin(2*phase),-c_.half_extents[1]*std::sin(phase)}};
    }else{derivative={{c_.half_extents[0]*std::cos(phase),2*c_.half_extents[1]*std::cos(2*phase)}};
      second={{-c_.half_extents[0]*std::sin(phase),-4*c_.half_extents[1]*std::sin(2*phase)}};}
    for(size_t i=0;i<2;++i){out.velocity[i]=derivative[i]*speed;out.acceleration[i]=second[i]*speed*speed+derivative[i]*acceleration;}
    return out;
  }
private:
  static bool finite(const std::array<double,2> & p){return std::isfinite(p[0])&&std::isfinite(p[1]);}
  static bool positive(double x){return std::isfinite(x)&&x>0;}
  static double pi(){return std::acos(-1.);}
  TrajectoryConfig c_;double omega_{0},duration_{0},approach_{0};bool started_{false};std::array<double,2> initial_{{0,0}};
};
}
#endif
