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
        Eigen::Isometry3f   init_frame_pose_;       // ��һ֡��ʵ�ʳ�������ϵ�е�λ�ú���ת������ Twc)
        Eigen::Vector3f     last_world_translation_,
                            last_slam_translation_; // ֮ǰ���������� or SLAM����λ�ƣ��������� SLAM���� => ����������������� scale_
        size_t  old_points_count_;  // ��� SLAM ϵͳ���ɵ� MapPoints �����Ƿ�ı�
        float   scale_;             // SLAM���� �� �������� ��λ�����ű���
        bool    is_initialized_;    // ��ʼ�� last_world_translation_ �� last_slam_translation_
    };

}

#endif // !__VCC_ZXM_MSLAM_H__