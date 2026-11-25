#include <gtest/gtest.h>
#include "flapping_uav_gcs/trajectory.hpp"
using flapping_uav_gcs::TrajectoryConfig;
using flapping_uav_gcs::HorizontalTrajectory;
namespace {
TrajectoryConfig config(const std::string & mode)
{
  TrajectoryConfig c;c.mode=mode;c.center={{3.2715,-2.8415}};
  c.half_extents=mode=="line"?std::array<double,2>{{1.5,0}}:std::array<double,2>{{2.22435,1.83735}};
  c.boundary={{{0,0}},{{.146,-5.659}},{{6.543,-5.683}},{{6.294,0}}};
  c.period_s=mode=="line"?40:60;return c;
}
}
TEST(Trajectory, ThreeMetreLineAndHorizontalFigureEightInsideMeasuredAnchorArea)
{
  for(const std::string mode:{"line","figure8"}){
    HorizontalTrajectory p(config(mode));double min_x=1e9,max_x=-1e9,min_y=1e9,max_y=-1e9;
    for(int i=0;i<=10000;++i){const auto point=p.point(2*std::acos(-1.)*i/10000);
      EXPECT_TRUE(p.inside(point));min_x=std::min(min_x,point[0]);max_x=std::max(max_x,point[0]);
      min_y=std::min(min_y,point[1]);max_y=std::max(max_y,point[1]);}
    if(mode=="line"){EXPECT_NEAR(max_x-min_x,3,1e-10);EXPECT_NEAR(max_y-min_y,0,1e-10);}
    else{EXPECT_NEAR(max_x-min_x,4.4487,1e-10);EXPECT_NEAR(max_y-min_y,3.6747,1e-10);}
  }
}
TEST(Trajectory, AnalyticVelocitiesAccelerationsAndSmoothJoins)
{
  for(const std::string mode:{"line","figure8"}){for(const std::string axis:{"x","y"}){
    auto c=config(mode);c.figure8_axis=axis;
    HorizontalTrajectory p(c);p.begin({{2,-2}});
    const auto begin=p.sample(0);EXPECT_NEAR(begin.velocity[0],0,1e-12);EXPECT_NEAR(begin.acceleration[0],0,1e-12);
    const double h=1e-4;
    for(int i=1;i<2000;++i){const double t=p.total_duration()*i/2000;
      const auto before=p.sample(t-h),mid=p.sample(t),after=p.sample(t+h);
      EXPECT_TRUE(p.inside(mid.position));
      EXPECT_LE(std::hypot(mid.velocity[0],mid.velocity[1]),.500001);
      EXPECT_LE(std::hypot(mid.acceleration[0],mid.acceleration[1]),.400001);
      for(size_t k=0;k<2;++k){EXPECT_NEAR((after.position[k]-before.position[k])/(2*h),mid.velocity[k],1e-6);
        EXPECT_NEAR((after.velocity[k]-before.velocity[k])/(2*h),mid.acceleration[k],1e-5);}
    }
    const auto end=p.sample(p.total_duration()+1);
    EXPECT_TRUE(end.finished);EXPECT_EQ(end.phase,"FINISHED");EXPECT_NEAR(end.velocity[0],0,1e-12);
    EXPECT_NEAR(end.velocity[1],0,1e-12);EXPECT_EQ(end.position,p.start_point());
    const auto left=p.sample(p.approach_duration()-h),right=p.sample(p.approach_duration()+h);
    EXPECT_NEAR(left.position[0],right.position[0],1e-6);EXPECT_NEAR(left.velocity[0],right.velocity[0],1e-5);
  }}
}
TEST(Trajectory, FigureEightCanArrangeBothLobesAlongMapY)
{
  auto c=config("figure8");c.figure8_axis="y";HorizontalTrajectory p(c);
  const auto north=p.point(std::acos(-1.)/2),south=p.point(3*std::acos(-1.)/2);
  EXPECT_NEAR(north[0],c.center[0],1e-12);EXPECT_NEAR(north[1],c.center[1]+c.half_extents[1],1e-12);
  EXPECT_NEAR(south[0],c.center[0],1e-12);EXPECT_NEAR(south[1],c.center[1]-c.half_extents[1],1e-12);
  c.figure8_axis="auto";EXPECT_THROW(HorizontalTrajectory unresolved(c),std::invalid_argument);
}
TEST(Trajectory, UnsafePeriodIsSlowedWithoutChangingPathAndOutsideStartRejected)
{
  auto c=config("figure8");c.period_s=1;HorizontalTrajectory p(c);
  EXPECT_GT(p.period(),1);EXPECT_THROW(p.begin({{0,0}}),std::invalid_argument);
  p.begin(p.start_point());EXPECT_EQ(p.approach_duration(),0);
  EXPECT_EQ(p.sample(0).phase,"TRACKING");
  c.half_extents={{20,20}};EXPECT_THROW(HorizontalTrajectory bad(c),std::invalid_argument);
  c=config("line");c.cycles=0;EXPECT_THROW(HorizontalTrajectory bad(c),std::invalid_argument);
}
TEST(Trajectory, MultipleCyclesCloseExactlyAndStayFinite)
{
  auto c=config("figure8");c.cycles=3;HorizontalTrajectory p(c);p.begin(p.start_point());
  EXPECT_GT(p.total_duration(),3*p.period());EXPECT_TRUE(p.sample(p.total_duration()).finished);
  EXPECT_THROW(p.sample(NAN),std::logic_error);
}
