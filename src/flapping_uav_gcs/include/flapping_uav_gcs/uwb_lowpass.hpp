#ifndef FLAPPING_UAV_GCS__UWB_LOWPASS_HPP_
#define FLAPPING_UAV_GCS__UWB_LOWPASS_HPP_
#include <array>
#include <cmath>
#include <stdexcept>

namespace flapping_uav_gcs
{
// One update per accepted observation. No timer prediction or artificial fixes.
class UwbLowPass
{
public:
  UwbLowPass(bool enabled,double tau_s,double reset_gap_s)
  :enabled_(enabled),tau_(tau_s),gap_(reset_gap_s)
  {
    if(!std::isfinite(tau_)||tau_<=0||!std::isfinite(gap_)||gap_<=0){
      throw std::invalid_argument("Invalid UWB low-pass time constant or reset gap");}
  }
  void reset(){initialized_=false;time_=NAN;value_={{NAN,NAN}};}
  bool update(double time,const std::array<double,2> & point,bool restart=false)
  {
    if(!std::isfinite(time)||!std::isfinite(point[0])||!std::isfinite(point[1])){return false;}
    if(initialized_&&!restart&&time<=time_){return false;}
    if(!enabled_||!initialized_||restart||time-time_>gap_){value_=point;}
    else{
      const double alpha=-std::expm1(-(time-time_)/tau_);
      for(size_t i=0;i<2;++i){value_[i]+=alpha*(point[i]-value_[i]);}
    }
    initialized_=true;time_=time;return true;
  }
  const std::array<double,2> & value()const{return value_;}
private:
  bool enabled_,initialized_{false};double tau_,gap_,time_{NAN};
  std::array<double,2> value_{{NAN,NAN}};
};
}
#endif
