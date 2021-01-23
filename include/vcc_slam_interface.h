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

namespace zxm
{
// Interface of a Orb-SLAM system.
// USAGE:
//    zxm::ISLAM* slam = GetInstanceOfSLAM();
//    while (true) {
//      slam->track(...);
//      if (isCloudPointsChanged()) {
//        auto all_building_pts = getAllBuildings();
//        for (id : building_ids) {
//          vector<Eigen::Vector3f> pts = all_building_pts[id];
//          // DO YOUR STUFF...
//        }
//      }
//    }
//    slam->shutDown();
//    DesctroySLAM(slam);
class ISLAM
{
public:
  ISLAM() {}
  ISLAM(const ISLAM&) = delete;
  ISLAM(ISLAM&&) = delete;
  void operator=(const ISLAM&) = delete;
  void operator=(ISLAM&&) = delete;

  // Track. Main method for creating buildings.
  // See ISLAM to find usage!
  // <img>    input picture.
  // <pose>   original key frame's pose in world coordinate.
  // <rects>  tracking building area(2D rectangel).
  virtual void track(const cv::Mat& img,
                     const Eigen::Isometry3f& pose,
                     const std::vector<cv::Rect2f>& rects) = 0;

  // Test whether clould points changed.
  virtual bool isCloudPointsChanged() = 0;

  // Get corresponding builiding ID of input tracking building area.
  virtual std::vector<int> getBuildingID() = 0;

  // Get all buildings' clould points.
  // Return map <building id>-<building cloud points>.
  // If <use_real_coordinate> is true(default), the points coordinate will be convert to
  // real world, else use the SLAM coordinate.
  virtual std::map<int, std::vector<Eigen::Vector3f>> getAllBuildings(bool use_real_coordinate = true) = 0;

  // Shut down the slam system.
  virtual void shutDown() = 0;

  virtual ~ISLAM() {}
};

} // namespace slam

extern "C"
{
  // See USAGE of class slam::ISLAM ! 
  // Alse see DesctroySLAM() for destroying the slam system.
  SLAM_API  zxm::ISLAM* GetInstanceOfSLAM();
  SLAM_API  void DesctroySLAM(zxm::ISLAM* obj);
}

#endif