#include "config.h"

#define SLAM_API_IMPLEMENT_HERE
#include "vcc_zxm_mslam.h"

// use cv::cv2eigen(). convert cv::Mat to Eigen.
#include <opencv2/core/eigen.hpp>


slam::MSLAM::MSLAM(const string& path_voc_file, const string& path_setting_file, bool use_viewer)
    : slam_system_{ path_voc_file, path_setting_file, ORB_SLAM3::System::MONOCULAR, use_viewer },
    old_points_count_{ 0 }, scale_{ 0.0f }, is_initialized_{ false }
{
    init_frame_pose_.setIdentity();
}

slam::MSLAM::~MSLAM()
{

}

bool slam::MSLAM::track(const cv::Mat& v_img, const Eigen::Isometry3f& v_pose)
{
    using namespace std;
    using vec3 = Eigen::Vector3f;

    cv::Mat Tcw = slam_system_.TrackMonocular(v_img, 0.0); // return Tcw

    if (slam_system_.mpTracker->mbInitialFrameReset) {
        init_frame_pose_ = v_pose;
        cout << "initial camera pose reset to:\n" << v_pose.matrix() << endl;
    } // current frame is initialized frame

    if (Tcw.empty()) {
        return false;
    } // track failed!

    // ���� real world ƽ�Ʋ����� slam camera ƽ�Ʋ��������Ź�ϵ
    vec3 world_translation = v_pose.translation();
    cv::Mat camera_pose = Tcw.inv(); // get Twc matrix
    vec3 slam_translation;
    cv::cv2eigen(camera_pose.col(3).rowRange(0, 3), slam_translation);

    auto tracking_state = slam_system_.GetTrackingState();
    size_t cur_points_count = slam_system_.mpAtlas->MapPointsInMap();
    if (is_initialized_
        && cur_points_count != old_points_count_
        && tracking_state == ORB_SLAM3::Tracking::eTrackingState::OK) {
        old_points_count_ = cur_points_count;
        // ���� �������� �߶� <= SLAM���� �߶� ��ת��
        vec3 world_dif = world_translation - last_world_translation_,
            slam_dif = slam_translation - last_slam_translation_;
        float new_scale = world_dif.norm() / slam_dif.norm();
        if (scale_ == 0.0f) {
            scale_ = new_scale;
        }
        else {
            scale_ = 0.5f * (new_scale + scale_);
        }
        return true;
    }
    else {
        last_world_translation_ = world_translation;
        last_slam_translation_ = slam_translation;
        is_initialized_ = true;
        return false;
    }
}

std::vector<Eigen::Vector3f> slam::MSLAM::get_points()
{
    auto points = slam_system_.mpAtlas->GetAllMapPoints();
    std::vector<Eigen::Vector3f> pts;
    for (auto pPt : points) {
        if (!pPt->isBad()) {
            Eigen::Vector3f pos;
            cv::cv2eigen(pPt->GetWorldPos(), pos);
            // �� SLAM ����ϵ => ʵ�ʳ�������ϵ
            pos *= scale_;
            //printf("slam pos:(%g, %g, %g)\n", pos.x(), pos.y(), pos.z());
            pos = init_frame_pose_ * pos;
            //printf("world pos:(%g, %g, %g)\n", pos.x(), pos.y(), pos.z());
            pts.emplace_back(pos);
        }
    } // �������е�
    return pts;
}

void slam::MSLAM::wait_shut_down()
{
    slam_system_.Shutdown();
}

extern "C"
{
    SLAM_API  slam::ISlam* GetInstanceOfSLAM() {
        return new slam::MSLAM(SLAM_VOC_FILE, SLAM_SETTING_FILE, USE_VIEWER);
    }
    SLAM_API  void DesctroySLAM(slam::ISlam* obj) {
        delete obj;
    }
}