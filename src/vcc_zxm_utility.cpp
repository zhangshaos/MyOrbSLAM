#define DEBUG  // use debug mode!!!
#include "vcc_zxm_utility.h"

#include "config.h"

cv::Scalar GetBuildingColor(int64_t id) {
  using namespace std;
  static std::mutex mtx;
  std::unique_lock<std::mutex> lock(mtx);
  static map<int64_t, cv::Scalar> building_color;
  if (building_color.count(id) == 0)
    building_color[id] = cv::Scalar(rand() % 256, rand() % 256, rand() % 256);
  return building_color[id];
}

#ifdef __DEBUG__
#include <algorithm>
#endif  // DEBUG

bool zxm::BestMatchGraph::AddRelation(int id, int r) {
  key2relations_[id].emplace(r);
  int counts = key2relations_[id].count(r);
  if (counts > key2candinates_[id].vote_count) {
    std::vector<int> candinates_ids(1, r);
    key2candinates_[id].candinate_ids.swap(candinates_ids);
    key2candinates_[id].vote_count = counts;
  }  // better than old candinates and replace them.
  else if (counts == key2candinates_[id].vote_count) {
    key2candinates_[id].candinate_ids.emplace_back(r);
  }  // same as old candinates and join them.
  return counts == 1;
}

std::vector<int> zxm::BestMatchGraph::QueryCandinates(int id) {
  return key2candinates_[id].candinate_ids;
}

void zxm::SimiliarRectSolver::addSimiliarRect(int id, float w, float h) {
  float dx = abs(w - targetW_), dy = abs(h - targetH_);
  q_.emplace(dx * dx + dy * dy, id);
}

int zxm::SimiliarRectSolver::getMostSimiliarRectID() { return q_.top().id_; }

void zxm::sra::PosRefAlgorithm::setKeyPoint(const cv::Point2f& kp) { kp_ = kp; }

int zxm::sra::PosRefAlgorithm::getKPOwner() { return 0; }

zxm::sra::PosRefAlgorithm::PosRef::PosRef(const cv::Rect2f& r1,
                                          const cv::Rect2f& r2,
                                          const cv::Point2f& kp)
    : r1_(r1), r2_(r2), kp_(kp) {
  // Assert the rectangles are overlap.
  // Count the contained corner points to detect the the Overlap Relationship
  // between rectangles.
}

int zxm::sra::PosRefAlgorithm::PosRef::getKPOwner() { return -1; }

/*********************************************************************
   // Compute rectangle match from current KF to ref KF.
    vector<int> rect_match(mpCurrentKeyFrame->tracking_rects_.size(), -1);
    if (!is_tracked) {
      DBG("LocalMappint: start to track KF %d\n", mpCurrentKeyFrame->mnId);

      zxm::BestMatchGraph match;
      for (auto& r : vMatchedIndices) {
        if (r.first < 0 || r.second < 0) {
          continue;
        }  // skip not matched result.
        vector<int> rects1 = mpCurrentKeyFrame->map_key2rectIDs_[r.first],
                    rects2 = pKF2->map_key2rectIDs_[r.second];
        for (int r1 : rects1) {
          if (r1 < 0) {
            continue;
          }  // skip
          for (int r2 : rects2) {
            if (r2 < 0) {
              continue;
            }  // skip
            match.AddRelation(r1, r2);
          }
        }
      }  // fill all matches
      for (int r = 0; r < rect_match.size(); ++r) {
        vector<int> candinates = match.QueryCandinates(r);
        int sz = candinates.size();
        if (sz == 0) {
          rect_match[r] = -1;
        }  // cur KF rectangle r => nothing
        else if (sz == 1) {
          rect_match[r] = candinates.front();
        } else {
          // r => more than 1 rectangles, need to be decreased
          float w = mpCurrentKeyFrame->tracking_rects_[r].width,
                h = mpCurrentKeyFrame->tracking_rects_[r].height;
          zxm::SimiliarRectSolver sv(w, h);
          for (int i : candinates) {
            w = pKF2->tracking_rects_[i].width;
            h = pKF2->tracking_rects_[i].height;
            sv.addSimiliarRect(i, w, h);
          }
          rect_match[r] = sv.getMostSimiliarRectID();
        }
      }                            // compute r => r(ref KF)
      // 纠正没有特征点的矩形匹配关系
      set<int> matched_cur_rects;  // 当前 KF 中已经被匹配的矩形索引
      for (int r = 0; r < rect_match.size(); ++r) {
        if (rect_match[r] >= 0) {
          matched_cur_rects.emplace(rect_match[r]);
          continue;
        }
        // TODO: 没有特征点计算矩形匹配关系时，追踪最像且没被匹配的矩形
        float w = mpCurrentKeyFrame->tracking_rects_[r].width,
              h = mpCurrentKeyFrame->tracking_rects_[r].height;
        zxm::SimiliarRectSolver sv(w, h);
        for (int i = 0; i < pKF2->tracking_rects_.size(); ++i) {
          w = pKF2->tracking_rects_[i].width;
          h = pKF2->tracking_rects_[i].height;
          sv.addSimiliarRect(i, w, h);
        }
        int target = sv.getMostSimiliarRectID();
        // 当前 KF 中的矩形 target 没有被匹配了，则
        if (matched_cur_rects.count(target) == 0) {
          rect_match[r] = target;
          matched_cur_rects.emplace(target);
        }  // else 维持 -1
      } // 纠正完毕
      BLOCK(for (int i = 0; i < rect_match.size(); ++i) {
        printf("LocalMapping: KF-%d => KF-%d: R%d => R%d\n",
               mpCurrentKeyFrame->mnId, pKF2->mnId, i, rect_match[i]);
      });
      // Now rect_match is ready! We should compute Current KF's building ID
      // and update the global buildings.
      for (int r = 0; r < rect_match.size(); ++r) {
        int ref_rect = rect_match[r];
        if (ref_rect < 0) {
          int id = mpAtlas->addBuilding();
          mpCurrentKeyFrame->map_rect2buildingID_[r] = id;
        } else {
          mpCurrentKeyFrame->map_rect2buildingID_[r] =
              pKF2->map_rect2buildingID_[ref_rect];
        }
      }  // detect new region as new building!
      BLOCK(
          for (int i = 0; i < mpCurrentKeyFrame->tracking_rects_.size(); ++i) {
            int building = mpCurrentKeyFrame->map_rect2buildingID_[i];
            printf("LocalMappint KF-%d: Rect %d <=> Building %d\n",
                   mpCurrentKeyFrame->mnId, i, building);
          });

      DBG("LocalMappint: end to track KF %d\n", mpCurrentKeyFrame->mnId);
    }  // rect_match computed over!
**********************************************************************/