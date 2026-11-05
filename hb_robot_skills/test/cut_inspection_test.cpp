#include <gtest/gtest.h>
#include "hb_robot_skills/motion/cut_inspection.hpp"
#include "hb_robot_perception/profile_library.hpp"

using namespace hb_robot_skills::motion;
TEST(CutInspection, SamplesEveryFeatureIncludingBothWebFaces) {
  auto profile=hb_perception::ProfileLibrary::makeIPN("test",0.20,0.10,0.006,0.01);
  auto stations=inspectionStations(profile,Eigen::Isometry3d::Identity(),0.025,0.30);
  for (const auto& feature:profile.cut_features) {
    std::vector<CutInspectionStation> feature_stations;
    for (const auto& s:stations) if(s.feature==feature.name) feature_stations.push_back(s);
    ASSERT_GE(feature_stations.size(),2u);
    EXPECT_NEAR((feature_stations.front().target.head<2>()-feature.start.cast<double>()).norm(),0,1e-7);
    EXPECT_NEAR((feature_stations.back().target.head<2>()-feature.end.cast<double>()).norm(),0,1e-7);
    for (std::size_t i=1;i<feature_stations.size();++i)
      EXPECT_LE((feature_stations[i].target-feature_stations[i-1].target).norm(),0.025+1e-7);
  }
}
TEST(CutInspection, OpticalAxisAndWorldTransform) {
  auto profile=hb_perception::ProfileLibrary::makeIPN("test",0.20,0.10,0.006,0.01);
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
  t.translate(Eigen::Vector3d(1,2,3));
  t.rotate(Eigen::AngleAxisd(0.7,Eigen::Vector3d::UnitY()));
  for(const auto& s:inspectionStations(profile,t,0.025,0.30)) {
    for(const auto& p:s.camera_poses) {
      EXPECT_NEAR((p.translation()-s.target).norm(),0.30,1e-7);
      EXPECT_NEAR(p.linear().col(2).dot((s.target-p.translation()).normalized()),1,1e-7);
      EXPECT_NEAR(p.linear().determinant(),1,1e-7);
      EXPECT_NEAR((t.inverse()*s.target).z(),0,1e-7);
    }
  }
}
TEST(CutInspection, RejectsInvalidAndExcessiveSampling) {
  auto p=hb_perception::ProfileLibrary::makeIPN("test",0.20,0.10,0.006,0.01);
  EXPECT_THROW(inspectionStations(p,Eigen::Isometry3d::Identity(),0,0.3),std::invalid_argument);
  EXPECT_THROW(inspectionStations(p,Eigen::Isometry3d::Identity(),1e-10,0.3),std::invalid_argument);
  EXPECT_THROW(inspectionStations(p,Eigen::Isometry3d::Identity(),0.02,NAN),std::invalid_argument);
}
TEST(CutInspection, BandRetainsBothSidesAndRejectsRemoteAndInvalidPoints) {
  auto p=hb_perception::ProfileLibrary::makeIPN("test",0.20,0.10,0.006,0.01);
  EXPECT_TRUE(inKerfBand(Eigen::Vector3d(0,0,0.01),p,0.015,0.02));
  EXPECT_TRUE(inKerfBand(Eigen::Vector3d(0,0,-0.01),p,0.015,0.02));
  EXPECT_FALSE(inKerfBand(Eigen::Vector3d(0,0,0.016),p,0.015,0.02));
  EXPECT_FALSE(inKerfBand(Eigen::Vector3d(1,0,0),p,0.015,0.02));
  EXPECT_FALSE(inKerfBand(Eigen::Vector3d(NAN,0,0),p,0.015,0.02));
}

#include "hb_robot_skills/kerf_observation.hpp"
TEST(CutInspection, MillimeterDepthAndRgbArePreserved) {
  hb_perception::Observation ob;
  auto info=std::make_shared<sensor_msgs::msg::CameraInfo>();
  info->width=2; info->height=1; info->k={100,0,0,0,100,0,0,0,1};
  ob.rgb_camera_info=info;
  cv::Mat depth(1,2,CV_16UC1); depth.at<uint16_t>(0,0)=300; depth.at<uint16_t>(0,1)=0;
  cv::Mat rgb(1,2,CV_8UC3,cv::Scalar(12,34,56));
  ob.depth_to_rgb=cv_bridge::CvImage(std_msgs::msg::Header(),"16UC1",depth).toImageMsg();
  ob.rgb=cv_bridge::CvImage(std_msgs::msg::Header(),"rgb8",rgb).toImageMsg();
  auto cloud=hb_robot_skills::kerfObservationCloud(ob,1.0);
  ASSERT_EQ(cloud->size(),1u);
  EXPECT_NEAR(cloud->front().z,0.3,1e-6);
  EXPECT_EQ(cloud->front().r,12); EXPECT_EQ(cloud->front().g,34); EXPECT_EQ(cloud->front().b,56);
  info->width=3;
  EXPECT_THROW(hb_robot_skills::kerfObservationCloud(ob,1.0),std::invalid_argument);
}
