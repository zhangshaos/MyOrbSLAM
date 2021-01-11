#ifndef __VCC_ZXM_MSLAM_H__
#define __VCC_ZXM_MSLAM_H__

#include <cstdint>

#include <System.h>

#include "vcc_slam_interface.h"

namespace slam {

    // wrapper for orb-slam3.
    // you could use this class to get easily map points when runing slam system.
    // EXAMPLE:
    // slam::MSLAM slam_sys{ path_voc_file, path_setting_file };
    // while (...) {
    //   auto im = cv::imread(...);
    //   eigen::Isometry3f pose = ...;
    //   if (slam_sys.track(im, pose)) {
    //   auto map_points = slam_sys.get_points();
    //     // do you stuff...
    //   }
    // }
    // slam_sys.wait_shut_down();
    class MSLAM : public ISLAM
    {
    public:
        MSLAM(const string& path_voc_file, const string& path_setting_file, bool use_viewer = false);
        ~MSLAM();
        
        // check whether map points' num changed. TRUE means changed, FALSE means unchanged.
        // @v_img:  input picture.
        // @v_pose: original key frame's pose in world coordinate.
        bool track(const cv::Mat& v_img, const Eigen::Isometry3f& v_pose);

        // get all map points from slam system and return those points.
        // you must to call track() to check change and then use this method.
        std::vector<Eigen::Vector3f> get_points();

        // shut down the slam system.
        void wait_shut_down();
        
    private:
        ORB_SLAM3::System   slam_system_;
        Eigen::Isometry3f   init_frame_pose_;       // 第一帧在实际场景坐标系中的位置和旋转（类似 Twc)
        Eigen::Vector3f     last_world_translation_,
                            last_slam_translation_; // 之前的世界坐标 or SLAM坐标位移，用来计算 SLAM坐标 => 世界坐标的缩放因子 scale_
        size_t  old_points_count_;  // 检测 SLAM 系统生成的 MapPoints 数量是否改变
        float   scale_;             // SLAM坐标 和 世界坐标 单位的缩放比例
        bool    is_initialized_;    // 初始化 last_world_translation_ 和 last_slam_translation_
    };

}

#endif // !__VCC_ZXM_MSLAM_H__