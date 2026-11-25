#ifndef FLAPPING_UAV_GCS__POSITION_SOURCE_HPP_
#define FLAPPING_UAV_GCS__POSITION_SOURCE_HPP_
#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <stdexcept>
#include <string>
namespace flapping_uav_gcs
{
struct SelectedPosition
{
  bool valid{false};std::string source{"INVALID"};
  std::array<double,2> position{{NAN,NAN}};
  uint64_t switches{0};
};
// The UWB health gate and DR horizon are supplied by LooseOdometry. Offsets
// preserve continuity at both handoffs; the recovery offset decays to zero.
class PositionSource
{
public:
  explicit PositionSource(double blend_s):blend_s_(blend_s)
  {if(!std::isfinite(blend_s)||blend_s<0){throw std::invalid_argument("Invalid recovery blend duration");}}
  void reset(){last_=SelectedPosition{};blending_=false;}
  SelectedPosition update(double now,bool uwb_good,const std::array<double,2> & uwb,
    bool flow_good,const std::array<double,2> & odometry)
  {
    auto out=last_;out.valid=false;out.source="INVALID";
    if(uwb_good && finite(uwb)){
      if(last_.valid && last_.source=="FLOW_ODOMETRY" && blend_s_>0){
        blending_=true;blend_started_=now;
        for(size_t i=0;i<2;++i){offset_[i]=last_.position[i]-uwb[i];}
      }
      double u=blend_s_>0?(now-blend_started_)/blend_s_:1.;
      if(blending_ && u>=1){blending_=false;}
      out.valid=true;out.source=blending_?"UWB_BLEND":"UWB";out.position=uwb;
      if(blending_){u=std::max(0.,u);const double factor=1-u*u*(3-2*u);
        for(size_t i=0;i<2;++i){out.position[i]+=offset_[i]*factor;}}
    }else if(flow_good && finite(odometry)){
      if(last_.source!="FLOW_ODOMETRY"){
        for(size_t i=0;i<2;++i){offset_[i]=last_.valid?last_.position[i]-odometry[i]:0.;}
      }
      blending_=false;out.valid=true;out.source="FLOW_ODOMETRY";
      for(size_t i=0;i<2;++i){out.position[i]=odometry[i]+offset_[i];}
    }else{blending_=false;}
    if(out.source!=last_.source){++out.switches;}last_=out;return out;
  }
private:
  static bool finite(const std::array<double,2> & p){return std::isfinite(p[0])&&std::isfinite(p[1]);}
  double blend_s_,blend_started_{0};bool blending_{false};
  std::array<double,2> offset_{{0,0}};SelectedPosition last_;
};
}
#endif
