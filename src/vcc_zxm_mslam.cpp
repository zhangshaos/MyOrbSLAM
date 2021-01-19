#include "config.h"

#define SLAM_API_IMPLEMENT_HERE
#include "vcc_zxm_mslam.h"

#define DEBUG // use debug mode
#include "E:/MyOrbSLAM/build/vcc_zxm_utiliy.h"

// use cv::cv2eigen(). convert cv::Mat to Eigen.
#include <opencv2/core/eigen.hpp>


const array<cv::Scalar, N_BUILDING_COLORS> zxm::MSLAM::BUILDING_COLORS{
  cv::Scalar(0., 153., 254.), // orange
  cv::Scalar(0., 255., 255.), // yellow
  cv::Scalar(255., 255., 0.), // blue-green
  cv::Scalar(255., 0., 0.),   // blue
  cv::Scalar(255., 0., 102.), // blue-perse
  cv::Scalar(255., 0., 255.), // perse
  cv::Scalar(102., 0., 255.), // perse-red
  cv::Scalar(0., 0., 255.),   // red
  cv::Scalar(0., 255., 153.), // yellow-green
  cv::Scalar(0., 255., 0.)    // green
};

zxm::MSLAM::MSLAM(const string& path_voc_file, const string& path_setting_file, bool use_viewer)
  : slam_system_(new ORB_SLAM3::System(path_voc_file, path_setting_file, ORB_SLAM3::System::eSensor::MONOCULAR, use_viewer))
{
  init_frame_pose_.setIdentity();
}

zxm::MSLAM::~MSLAM()
{ }

void zxm::MSLAM::track(const cv::Mat& img, const Eigen::Isometry3f& pose, const std::vector<cv::Rect2f>& rects)
{
  using namespace std;
  using vec3 = Eigen::Vector3f;
  
  cv::Mat Tcw = slam_system_->trackMSLAM(img, rects, building_IDs_); // return Tcw

  if (slam_system_->mpTracker->mbInitialFrameReset) {
    init_frame_pose_ = pose;
    cout << "initial camera pose reset to:\n" << pose.matrix() << endl;
  } // current frame is initialized frame

  if (Tcw.empty()) {
    return;
  } // track failed!

  // convert real world coordinate to zxm camera coordinate.
  vec3 world_translation = pose.translation();
  cv::Mat camera_pose = Tcw.inv(); // get Twc matrix.
  vec3 slam_translation;
  cv::cv2eigen(camera_pose.col(3).rowRange(0, 3), slam_translation);

  auto tracking_state = slam_system_->GetTrackingState();
  size_t cur_points_count_ = slam_system_->mpAtlas->MapPointsInMap();
  if (is_initialized_ && isCloudPointsChanged()
      && tracking_state == ORB_SLAM3::Tracking::eTrackingState::OK) {
    // compute distance scale from SLAM to real world.
    vec3 world_dif = world_translation - last_world_translation_,
      slam_dif = slam_translation - last_slam_translation_;
    float new_scale = world_dif.norm() / slam_dif.norm();
    if (scale_ == 0.0f) {
      scale_ = new_scale;
    }
    else {
      scale_ = 0.5f * (new_scale + scale_);
    }
  }
  else {
    last_world_translation_ = world_translation;
    last_slam_translation_ = slam_translation;
    is_initialized_ = true;
  } // return
}

bool zxm::MSLAM::isCloudPointsChanged() {
  bool ans = cur_points_count_ != old_points_count_;
  old_points_count_ = cur_points_count_;
  return ans;
}

std::vector<int> zxm::MSLAM::getBuildingID() {
  return building_IDs_;
}

std::map<int, std::vector<Eigen::Vector3f>> zxm::MSLAM::getAllBuildings()
{
  std::map<int, std::vector<Eigen::Vector3f>> map_buildingID2points;
  auto points = slam_system_->mpAtlas->GetAllMapPoints();
  for (auto pt : points) {
    if (!pt->isBad()) {
      Eigen::Vector3f pos;
      cv::cv2eigen(pt->GetWorldPos(), pos);
      pos *= scale_;
      pos = init_frame_pose_ * pos;
      int id = pt->building_id_;
      if (id < 0) {
        continue;
      } // skip untracking buildings
      map_buildingID2points[id].push_back(move(pos));
    }
  }
  return map_buildingID2points;
}

void zxm::MSLAM::shutDown()
{
  slam_system_->Shutdown();
}

const cv::Scalar& zxm::MSLAM::getBuildingColor(int id) {
  assert(id >= 0 && "we not support default building color now!");
  return BUILDING_COLORS[id % N_BUILDING_COLORS];
}

extern "C"
{
  SLAM_API  zxm::ISLAM* GetInstanceOfSLAM()
  {
    return new zxm::MSLAM(SLAM_VOC_FILE, SLAM_SETTING_FILE, USE_VIEWER);
  }
  SLAM_API  void DesctroySLAM(zxm::ISLAM* obj)
  {
    delete obj;
  }
}