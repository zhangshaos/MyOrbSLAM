#pragma warning (disable : 4267)

#include <cmath>

#include <filesystem>
#include <algorithm>
#include <fstream>
#include <memory>

#include <nlohmann/json.hpp>

#include "vcc_slam_interface.h"

namespace fs = std::filesystem; // use C++17, std::experimental::filesystem in the version below c++17
using namespace std;

#define N_BUILDING_COLORS 10 
const array<cv::Scalar, N_BUILDING_COLORS> BUILDING_COLORS{
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

const cv::Scalar& GetBuildingColor(int id) {
  assert(id >= 0);
  return BUILDING_COLORS[id % BUILDING_COLORS.size()];
}

inline
Eigen::Quaternionf euler2quaternion(float yaw, float pitch, float roll) {
  float cosa = cosf(roll / 2.0f), sina = sinf(roll / 2.0f),
    cosb = cosf(pitch / 2.0f), sinb = sinf(pitch / 2.0f),
    cosr = cosf(yaw / 2.0f), sinr = sinf(yaw / 2.0f);
  Eigen::Quaternionf q;
  q.w() = cosa * cosb * cosr + sina * sinb * sinr;
  q.x() = sina * cosb * cosr - cosa * sinb * sinr;
  q.y() = cosa * sinb * cosr + sina * cosb * sinr;
  q.z() = cosa * cosb * sinr - sina * sinb * cosr;
  return q;
}

inline
Eigen::Vector3f quaternion2euler(const Eigen::Quaternionf& q) {
  float q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();
  Eigen::Vector3f v;
  v[0] = atan2f(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
  v[1] = asinf(2 * (q0 * q2 - q1 * q3));
  v[2] = atan2f(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
  return v;
}

struct SLAMDestroyer
{
  void operator()(zxm::ISLAM* p) {
    DesctroySLAM(p);
  }
};

// Reisze 我自己的数据
int ResizePics() {
  vector<string> pic_paths{
    "E:\\YRS_LYL_0122\\1"s,
    "E:\\YRS_LYL_0122\\112MEDIA"s,
    "E:\\DJI_0001\\DJI_0001"s,
  },
    target_dir{
    "E:\\PICS\\1"s,
    "E:\\PICS\\2"s,
    "E:\\PICS\\3"s
  };
  for (int i = 2; i < pic_paths.size(); ++i) {
    for (fs::directory_iterator it(pic_paths[i]), end; it != end; ++it) {
      if (it->is_regular_file()) {
        auto im = cv::imread(it->path().string(), cv::IMREAD_GRAYSCALE);
        cv::Mat target;
        cv::resize(im, target, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
        fs::path dir = target_dir[i];
        dir.append(it->path().filename().string());
        cv::imwrite(dir.string(), target);
      }
    } // load all pictures path
  }
  return 0;
}

// 测试 ORB-SLAM 测试集！
int TestMain() {
  //return ResizePics();
  unique_ptr<zxm::ISLAM, SLAMDestroyer> sys(GetInstanceOfSLAM(), SLAMDestroyer());

  // set pictrues
  vector<string> pics_path;
  for (fs::directory_iterator iter("E:\\ORB_SLAM3\\Examples\\Stereo-Inertial\\mav0\\cam0\\data"), end;
       iter != end; ++iter) {
    if (iter->is_regular_file()) {
      pics_path.emplace_back(iter->path().string());
    }
  } // load all pictures path
  sort(pics_path.begin(), pics_path.end()); // sort by charactor incresing

  // set camera pose
  Eigen::Isometry3f init_pose;

  // SLAM coordinate => meshlab coordinate
  Eigen::Matrix3f slam2meshlab_mat;
  slam2meshlab_mat << 0, 0, 1, -1, 0, 0, 0, -1, 0;

  ofstream f_points("cloud_points.txt");
  map<int, vector<Eigen::Vector3f>> final_buildings;
  f_points.setf(ios_base::fixed);
  f_points.precision(3);

  for (int i = 0; i < pics_path.size(); i += 1) {
    init_pose.setIdentity();
    cv::Mat im = cv::imread(pics_path[i], cv::IMREAD_GRAYSCALE);
    vector<cv::Rect2f> rects;
    rects.emplace_back(0.f, 0.f, 1920.f, 1080.f);
    sys->track(im, init_pose, rects);
    if (sys->isCloudPointsChanged()) {
      auto buildings = sys->getAllBuildings(false);
      for (auto& id_pts : buildings) {
        printf("building ID is %d\n", id_pts.first);
        final_buildings[id_pts.first] = move(id_pts.second);
      }
    }
  }
  // Store building clould points.
  for (auto& id_pts : final_buildings) {
    int id = id_pts.first;
    vector<Eigen::Vector3f>& pts = id_pts.second;
    auto color = GetBuildingColor(id);
    for (auto& pt : pts) {
      pt = slam2meshlab_mat * pt;
      f_points << pt.x() << ';' << pt.y() << ';' << pt.z() << ';'
        << int(color[2]) << ';' << int(color[1]) << ';' << int(color[0])
        << endl;
    }
  }
  sys->shutDown();

  return 0;
}

// 测试我自己的数据
int Main() {
  unique_ptr<zxm::ISLAM, SLAMDestroyer> sys(GetInstanceOfSLAM(), SLAMDestroyer());

  // set pictrues
  vector<string> pics_path;
  for (fs::directory_iterator iter("E:/AirSim/HelloDrone/pics"), end;
       iter != end; ++iter) {
    if (iter->is_regular_file()) {
      pics_path.emplace_back(iter->path().string());
    }
  } // load all pictures path
  sort(pics_path.begin(), pics_path.end()); // sort by charactor incresing
  // set tracking rects
  ifstream f_rects("E:/MyOrbSLAM/example/vs_demo/rects.txt");
  assert(f_rects.is_open());
  vector<vector<cv::Rect2f>> tracking_rects;
  int id = -1;
  while (f_rects >> id) {
    vector<cv::Rect2f> rects;
    for (int i = 0; i < 3; ++i) {
      float x0 = -1., y0 = -1., x1 = -1., y1 = -1.;
      char sep = 0;
      f_rects >> x0 >> sep >> y0 >> sep >> x1 >> sep >> y1;
      rects.emplace_back(x0, y0, x1 - x0, y1 - y0);
    }
    tracking_rects.push_back(move(rects));
  } // rects over!

  // set camera pose
  ifstream fin("E:/AirSim/HelloDrone/config/camera_trajectory.json");
  assert(fin.is_open());
  nlohmann::json conf;
  fin >> conf;
  auto positions = conf["positions"].get<std::vector<pair<int, array<float, 3>>>>(),
    orientations = conf["orientations"].get<std::vector<pair<int, array<float, 3>>>>();
  assert(positions.size() == pics_path.size());

  Eigen::Isometry3f init_pose;

  // SLAM coordinate => meshlab coordinate
  Eigen::Matrix3f slam2meshlab_mat;
  slam2meshlab_mat << 0, 0, 1, -1, 0, 0, 0, -1, 0;

  ofstream f_points("cloud_points.txt");
  map<int, vector<Eigen::Vector3f>> final_buildings;
  f_points.setf(ios_base::fixed);
  f_points.precision(3);

  for (int i = 0; i < pics_path.size(); ++i) {
    auto& pos = positions[i].second, ori = orientations[i].second;
    cv::Mat im = cv::imread(pics_path[i], cv::IMREAD_GRAYSCALE);
    init_pose.setIdentity();
    init_pose.rotate(euler2quaternion(ori[2], ori[1], ori[0]));
    init_pose.pretranslate(Eigen::Vector3f(pos[0], pos[1], pos[2]));
    vector<cv::Rect2f> rects;
    if (i >= tracking_rects.size()) {
      break; // we skip ... just for now!
    }
    else {
      rects = move(tracking_rects[i]);
    }
    //rects.emplace_back(0.f, 0.f, 640.f, 640.f);
    sys->track(im, init_pose, rects);
    if (sys->isCloudPointsChanged()) {
      auto buildings = sys->getAllBuildings();
      for (auto& id_pts : buildings) {
        printf("building ID is %d\n", id_pts.first);
        final_buildings[id_pts.first] = move(id_pts.second);
      }
    }
  }
  for (auto& id_pts : final_buildings) {
    int id = id_pts.first;
    vector<Eigen::Vector3f>& pts = id_pts.second;
    auto color = GetBuildingColor(id);
    for (auto& pt : pts) {
      pt = slam2meshlab_mat * pt;
      f_points << pt.x() << ';' << pt.y() << ';' << pt.z() << ';'
        << int(color[2]) << ';' << int(color[1]) << ';' << int(color[0])
        << endl;
    }
  }
  sys->shutDown();
  return 0;
}

int main() {
  //return TestMain();
  //return ResizePics();
  unique_ptr<zxm::ISLAM, SLAMDestroyer> sys(GetInstanceOfSLAM(), SLAMDestroyer());

  // set pictrues
  vector<string> pics_path;
  //for (fs::directory_iterator iter("E:\\PICS\\3"), end;
  //     iter != end; ++iter) {
  //  if (iter->is_regular_file()) {
  //    pics_path.emplace_back(iter->path().string());
  //  }
  //} // load all pictures path
  for (int name = 38; name <= 272; ++name) {
    string fname = "E:/PICS/3/" + to_string(name) + '_' + to_string(5 * name) + ".png";
    pics_path.emplace_back(move(fname));
  }
  //sort(pics_path.begin(), pics_path.end()); // sort by charactor incresing

  // set camera pose
  Eigen::Isometry3f init_pose;

  // SLAM coordinate => meshlab coordinate
  Eigen::Matrix3f slam2meshlab_mat;
  slam2meshlab_mat << 0, 0, 1, -1, 0, 0, 0, -1, 0;

  ofstream f_points("cloud_points.txt");
  map<int, vector<Eigen::Vector3f>> final_buildings;
  f_points.setf(ios_base::fixed);
  f_points.precision(3);

  for (int i = 0; i < pics_path.size(); i += 1) {
    init_pose.setIdentity();
    cv::Mat im = cv::imread(pics_path[i], cv::IMREAD_GRAYSCALE);
    vector<cv::Rect2f> rects;
    rects.emplace_back(0.f, 0.f, 1920.f, 1080.f);
    sys->track(im, init_pose, rects);
    if (sys->isCloudPointsChanged()) {
      auto buildings = sys->getAllBuildings(false);
      for (auto& id_pts : buildings) {
        printf("building ID is %d\n", id_pts.first);
        final_buildings[id_pts.first] = move(id_pts.second);
      }
    }
  }
  // Store building clould points.
  for (auto& id_pts : final_buildings) {
    int id = id_pts.first;
    vector<Eigen::Vector3f>& pts = id_pts.second;
    auto color = GetBuildingColor(id);
    for (auto& pt : pts) {
      pt = slam2meshlab_mat * pt;
      f_points << pt.x() << ';' << pt.y() << ';' << pt.z() << ';'
        << int(color[2]) << ';' << int(color[1]) << ';' << int(color[0])
        << endl;
    }
  }
  sys->shutDown();

  return 0;
}
