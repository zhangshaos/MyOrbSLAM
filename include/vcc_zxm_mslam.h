#ifndef __VCC_ZXM_MSLAM_H__
#define __VCC_ZXM_MSLAM_H__

#include <System.h>

#include <cstdint>
#include <memory>

#include "vcc_slam_interface.h"

namespace zxm {

// See ISLAM Usage !
class MSLAM : public ISLAM {
 public:
  MSLAM(const string& path_voc_file, const string& path_setting_file,
        bool use_viewer = false);
  ~MSLAM();

  // Track. Main method for creating buildings.
  // See ISLAM to find usage!
  // <img>    input picture.
  // <pose>   original key frame's pose in world coordinate.
  // <rects>  tracking building area(2D rectangel).
  virtual bool track(const cv::Mat& img, const Eigen::Isometry3f& pose,
                     const std::vector<cv::Rect2f>& rects) override;

  // Test whether clould points changed.
  virtual bool isCloudPointsChanged() override;

  // Get all buildings' clould points.
  // Return map <building id>-<building cloud points>.
  virtual std::map<int, std::vector<Eigen::Vector3f>> getAllBuildings(
      bool use_real_coordinate = true) override;

  // Get CURRENT buildings' points.
  virtual std::vector<std::vector<Eigen::Vector3f>> getCurrentBuildings()
      override;

  // Override with mask to decreasing the num of keypoints.
  // And this func will create keypoint pngs.
  // `color` 从低到高分别是 B G R，即 R 在最高位 [2] 上
  // `toshow` 返回展示的图片
  virtual std::vector<std::vector<Eigen::Vector3f>> getCurrentBuildings(
      cv::Mat mask, const std::set<std::array<unsigned char, 3>>& colors,
      cv::Mat& toshow) override;

  // Shut down the slam system.
  virtual void shutDown() override;

  // Get building color whose id is <id>.
  static const cv::Scalar& getBuildingColor(int id);

 private:
  std::shared_ptr<ORB_SLAM3::System> slam_system_;
  Eigen::Isometry3f T_c_w_;  // Tcw of current frame.
  // InitialFrame's pose(like Twc) for convert SLAM coordinate position to Real
  // World coordinate position, see track() for usage.
  Eigen::Isometry3f init_frame_pose_;
  Eigen::Vector3f last_world_translation_,
      last_slam_translation_;  // to compute scale_, see track() for usage.
  size_t old_points_count_ = 0,
         cur_points_count_ = 0;  // valid whether mapoints' count had changed.
  float scale_ = 0.f;  // scale from SLAM coordinate to Real World coordinate.
  bool is_initialized_ =
      false;  // just for initialzing last_world_translation_ and
              // last_slam_translation_, see track() for usage.
};

}  // namespace zxm

#endif  // !__VCC_ZXM_MSLAM_H__