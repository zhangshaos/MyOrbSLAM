#ifndef __VCC_SLAM_INTERFACE__
#define __VCC_SLAM_INTERFACE__

#ifdef _WIN32
    #ifdef SLAM_API_IMPLEMENT_HERE
        #define SLAM_API __declspec(dllexport)
    #else
        #define SLAM_API __declspec(dllimport)
    #endif
#elif __linux__
    #ifdef SLAM_API_IMPLEMENT_HERE
        #define SLAM_API __attribute__ ((visibility("default")))
    #else
        #define SLAM_API __attribute__ ((visibility("hidden")))
    #endif
#else
    #define SLAM_API
#endif

#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace slam
{

    class ISlam
    {
    public:
        ISlam() {}
        ISlam(const ISlam&) = delete;
        ISlam(ISlam&&)      = delete;
        void operator=(const ISlam&) = delete;
        void operator=(ISlam&&)      = delete;

        // check whether map points' num changed. TRUE means changed, FALSE means unchanged.
        // @v_img:  input picture.
        // @v_pose: original key frame's pose in world coordinate.
        virtual bool track(const cv::Mat& v_img, const Eigen::Isometry3f& v_pose) = 0;

        // get all map points from slam system and return those points.
        // you must to call track() to check change and then use this method.
        virtual std::vector<Eigen::Vector3f> get_points() = 0;

        // shut down the slam system.
        virtual void wait_shut_down() = 0;

        virtual ~ISlam() {}
    };

} // namespace slam

extern "C"
{
    SLAM_API  slam::ISlam*  GetInstanceOfSLAM();
    SLAM_API  void DesctroySLAM(slam::ISlam* obj);
}

#endif