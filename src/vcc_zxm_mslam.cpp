#include "config.h"

#define SLAM_API_IMPLEMENT_HERE
#include "vcc_zxm_mslam.h"

#define __DEBUG__  // use debug mode
#include "vcc_zxm_utility.h"

// use cv::cv2eigen(). convert cv::Mat to Eigen.
#include <opencv2/core/eigen.hpp>

zxm::MSLAM::MSLAM(const string& path_voc_file, const string& path_setting_file,
                  bool use_viewer)
    : slam_system_(new ORB_SLAM3::System(path_voc_file, path_setting_file,
                                         ORB_SLAM3::System::eSensor::MONOCULAR,
                                         use_viewer)) {
  init_frame_pose_.setIdentity();
}

zxm::MSLAM::~MSLAM() {}

bool zxm::MSLAM::track(const cv::Mat& img, const Eigen::Isometry3f& pose,
                       const std::vector<cv::Rect2f>& rects) {
  using namespace std;
  using vec3 = Eigen::Vector3f;

  cv::Mat Tcw = slam_system_->trackMSLAM(img, rects);  // return Tcw

  if (slam_system_->mpTracker->mbInitialFrameReset) {
    init_frame_pose_ = pose;
    cout << "initial camera pose reset to:\n" << pose.matrix() << endl;
  }  // current frame is initialized frame

  if (Tcw.empty()) {
    return false;
  }  // track failed!
  Eigen::Matrix4f t;
  cv::cv2eigen(Tcw, t);
  T_c_w_ = Eigen::Isometry3f(t);

  // 下面计算 SLAM 坐标系到现实坐标系的转换关系
  // convert real world coordinate to zxm camera coordinate.
  vec3 world_translation = pose.translation();
  cv::Mat camera_pose = Tcw.inv();  // get Twc matrix.
  vec3 slam_translation;
  cv::cv2eigen(camera_pose.col(3).rowRange(0, 3), slam_translation);

  auto tracking_state = slam_system_->GetTrackingState();
  cur_points_count_ = slam_system_->mpAtlas->MapPointsInMap();
  if (is_initialized_ && cur_points_count_ != old_points_count_ &&
      tracking_state == ORB_SLAM3::Tracking::eTrackingState::OK) {
    // compute distance scale from SLAM to real world.
    vec3 world_dif = world_translation - last_world_translation_,
         slam_dif = slam_translation - last_slam_translation_;
    float new_scale = world_dif.norm() / slam_dif.norm();
    if (scale_ == 0.0f) {
      scale_ = new_scale;
    } else {
      scale_ = 0.5f * (new_scale + scale_);
    }
  } else {
    last_world_translation_ = world_translation;
    last_slam_translation_ = slam_translation;
    is_initialized_ = true;
  }
  return true;
}

bool zxm::MSLAM::isCloudPointsChanged() {
  bool ans = cur_points_count_ != old_points_count_;
  old_points_count_ = cur_points_count_;
  return ans;
}

Eigen::Matrix3f zxm::MSLAM::getIntrinsicK() {
  Eigen::Matrix3f K; // cv::Mat 是 float 嘛？
  cv::cv2eigen(slam_system_->mpTracker->getK(), K);
  return K;
}

Eigen::Isometry3f zxm::MSLAM::getOutwardTcw() { return T_c_w_; }

std::map<int, std::vector<Eigen::Vector3f>> zxm::MSLAM::getAllBuildings(
    bool use_real_coordinate) {
  std::map<int, std::vector<Eigen::Vector3f>> map_buildingID2points;
  auto buildings = slam_system_->mpAtlas->getAllBuildings();
  for (auto& id_pts : buildings) {
    int id = id_pts.first;
    auto points = id_pts.second;
    for (auto pt : points) {
      if (pt->isBad()) continue;
      Eigen::Vector3f pos;
      cv::cv2eigen(pt->GetWorldPos(), pos);
      if (use_real_coordinate) {
        pos *= scale_;
        pos = init_frame_pose_ * pos;
      }
      map_buildingID2points[id].push_back(move(pos));
    }
  }
  return map_buildingID2points;
}

std::vector<std::vector<Eigen::Vector3f>> zxm::MSLAM::getCurrentBuildings(
    bool use_real_coordinate) {
  auto buildings = slam_system_->mpAtlas->getAllBuildings();
  auto r2b = slam_system_->mpTracker->mCurrentFrame.rect_to_buildings;
  std::vector<std::vector<Eigen::Vector3f>> points(r2b.size());
  for (int r = 0; r < r2b.size(); ++r) {
    int b = r2b[r];
    if (buildings.count(b)) {
      auto& pts = buildings[b];
      // 将好的点加入
      for (auto pt : pts) {
        if (pt->isBad()) continue;
        Eigen::Vector3f w_pos;
        cv::cv2eigen(pt->GetWorldPos(), w_pos);
        // 坐标投影到当前坐标系下，并进行像素化
        Eigen::Vector3f c_pos = T_c_w_ * w_pos;  // 当前相机坐标系下
        cv::Mat1f c_pos_mat;
        cv::eigen2cv(c_pos, c_pos_mat);
        cv::Mat1f h_uv =
            slam_system_->mpTracker->getK() * c_pos_mat;  // 其次像素坐标
        cv::Point2f uv(h_uv[0][0] / h_uv[2][0], h_uv[1][0] / h_uv[2][0]);
        auto& rect = slam_system_->mpTracker->tracking_rects_[r];
        if (rect.contains(uv)) {
          if (use_real_coordinate)
            points[r].emplace_back(w_pos);
          else
            points[r].emplace_back(c_pos);
        }
      }
    }
  }
  return points;
}

// 判断某个点是否在感兴趣区域中
struct FunKeyPointArea {
  FunKeyPointArea(cv::Mat mask,
                  const std::set<std::array<unsigned char, 3>>& colors) {
    _mask = mask.clone();
    _colors = colors;
  }
  bool contain(const cv::Point2f& p) {
    int x = p.x, y = p.y;
    unsigned char* bgr = _mask.ptr<unsigned char>(y, x);
    std::array<unsigned char, 3> color{bgr[2], bgr[1],
                                       bgr[0]};  // 从低位到高位：B，G，R
    return _colors.count(color) > 0;
  }

 private:
  cv::Mat _mask;
  std::set<std::array<unsigned char, 3>> _colors;
};

std::vector<std::vector<Eigen::Vector3f>> zxm::MSLAM::getCurrentBuildings(
    cv::Mat mask, const std::set<std::array<unsigned char, 3>>& colors,
    cv::Mat& toshow) {
  // 构造感兴趣区域
  FunKeyPointArea fun_area(mask, colors);
  // 获取 MapPoint 数据
  auto buildings = slam_system_->mpAtlas->getAllBuildings();
  auto r2b = slam_system_->mpTracker->mCurrentFrame.rect_to_buildings;
  std::vector<std::vector<Eigen::Vector3f>> points(r2b.size());
  for (int r = 0; r < r2b.size(); ++r) {
    int b = r2b[r];
    if (buildings.count(b)) {
      auto& pts = buildings[b];
      // 将好的点加入
      for (auto pt : pts) {
        if (pt->isBad()) continue;
        // 构造不同类型的点数据
        Eigen::Vector3f w_pos;
        cv::cv2eigen(pt->GetWorldPos(), w_pos);
        Eigen::Vector3f c_pos = T_c_w_ * w_pos;  // 当前相机坐标系下
        cv::Mat1f c_pos_mat;
        cv::eigen2cv(c_pos, c_pos_mat);
        cv::Mat1f h_uv =
            slam_system_->mpTracker->getK() * c_pos_mat;  // 其次像素坐标
        cv::Point2f uv(h_uv[0][0] / h_uv[2][0], h_uv[1][0] / h_uv[2][0]);
        auto& rect = slam_system_->mpTracker->tracking_rects_[r];
        if (rect.contains(uv) && fun_area.contain(uv)) {
          points[r].emplace_back(c_pos);
          // 输出图片数据
          cv::circle(toshow, uv, 2, ::GetBuildingColor(b), -1);
        }
      }
    }
  }
  return points;
}

void zxm::MSLAM::shutDown() { slam_system_->Shutdown(); }

extern "C" {
SLAM_API zxm::ISLAM* GetInstanceOfSLAM() {
  return new zxm::MSLAM(SLAM_VOC_FILE, SLAM_SETTING_FILE, USE_VIEWER);
}
SLAM_API void DesctroySLAM(zxm::ISLAM* obj) { delete obj; }
}
