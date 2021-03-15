#define DEBUG // use debug mode!!!
#include "vcc_zxm_utility.h"

#ifdef DEBUG
#include <algorithm>
#endif // DEBUG

bool zxm::BestMatchGraph::AddRelation(int id, int r)
{
  key2relations_[id].emplace(r);
  int counts = key2relations_[id].count(r);
  if (counts > key2candinates_[id].vote_count) {
    std::vector<int> candinates_ids(1, r);
    key2candinates_[id].candinate_ids.swap(candinates_ids);
    key2candinates_[id].vote_count = counts;
  } // better than old candinates and replace them.
  else if (counts == key2candinates_[id].vote_count) {
    key2candinates_[id].candinate_ids.emplace_back(r);
  } // same as old candinates and join them.
  return counts == 1;
}

std::vector<int> zxm::BestMatchGraph::QueryCandinates(int id)
{
  return key2candinates_[id].candinate_ids;
}

void zxm::SimiliarRectSolver::addSimiliarRect(int id, float w, float h)
{
  float dx = abs(w - targetW_), dy = abs(h - targetH_);
  q_.emplace(dx * dx + dy * dy, id);
}

int zxm::SimiliarRectSolver::getMostSimiliarRectID()
{
  return q_.top().id_;
}

void zxm::sra::PosRefAlgorithm::setKeyPoint(const cv::Point2f& kp) {
  kp_ = kp;
}

int zxm::sra::PosRefAlgorithm::getKPOwner() {
  return 0;
}

zxm::sra::PosRefAlgorithm::PosRef::PosRef(const cv::Rect2f& r1, const cv::Rect2f& r2, const cv::Point2f& kp)
  : r1_(r1), r2_(r2), kp_(kp) {
  // Assert the rectangles are overlap.
  // Count the contained corner points to detect the the Overlap Relationship between rectangles.
}

int zxm::sra::PosRefAlgorithm::PosRef::getKPOwner() {
  return -1;
}
