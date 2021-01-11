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

#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace slam
{
    // Interface of a Orb-SLAM system.
    // USAGE:
    //   slam::ISLAM* slam = GetInstanceOfSLAM();
    //   while (true) {
    //     if (slam->track(img, pose)) {
    //       auto points = slam->get_points();
    //       // do your stuff.
    //     }
    //   }
    //   slam->wait_shut_down();
    //   DesctroySLAM(slam::ISLAM* slam);
    class ISLAM
    {
    public:
        ISLAM() {}
        ISLAM(const ISLAM&) = delete;
        ISLAM(ISLAM&&)      = delete;
        void operator=(const ISLAM&) = delete;
        void operator=(ISLAM&&)      = delete;

        // check whether map points' num changed. TRUE means changed, FALSE means unchanged.
        // @v_img:  input picture.
        // @v_pose: original key frame's pose in world coordinate.
        virtual bool track(const cv::Mat& v_img, const Eigen::Isometry3f& v_pose) = 0;

        // get all map points from slam system and return those points.
        // you must to call track() to check change and then use this method.
        virtual std::vector<Eigen::Vector3f> get_points() = 0;

        // shut down the slam system.
        virtual void wait_shut_down() = 0;

        virtual ~ISLAM() {}
    };

} // namespace slam

extern "C"
{
    // see USAGE of class slam::ISLAM ! 
    // alse see DesctroySLAM() for destroying the slam system.
    SLAM_API  slam::ISLAM*  GetInstanceOfSLAM();
    SLAM_API  void DesctroySLAM(slam::ISLAM* obj);
}

#endif