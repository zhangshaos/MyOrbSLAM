#ifndef __VCC_ZXM_UTILITY_H__
#define __VCC_ZXM_UTILITY_H__

#include <cassert>
#include <cstdint>

#include <set>
#include <map>
#include <vector>
#include <queue>

#include <opencv2/core.hpp>

namespace zxm
{

// This is a structure for matching vertexes.
// It has 2 methods:
// 1. Add a pair<vertex_id1, vertex_id2>, if the pair exsisted, the increase the pair's count.
// 2. Query a vertex_id1, return the vertex_id2(may be more one vertex) which <vertex_id1, vertex_id2>'s count is maximum
//    among all pairs like <vertex_id1, vertex_id?>.
class BestMatchGraph
{
public:
  struct Candinates
  {
    std::vector<int>  candinate_ids;
    int64_t           vote_count;
    Candinates()
      : candinate_ids(), vote_count(0) {
    }
    Candinates(const std::vector<int>& ids, int64_t votes)
      : candinate_ids(ids), vote_count(votes) {
    }
  };
  // Big-five
  BestMatchGraph() = default;
  BestMatchGraph(const BestMatchGraph& o) = default;
  BestMatchGraph(BestMatchGraph&& o) noexcept = default;
  void operator=(const BestMatchGraph& o) {
    BestMatchGraph copy(o);
    std::swap(copy.key2relations_, this->key2relations_);
    std::swap(copy.key2candinates_, this->key2candinates_);
  }
  void operator=(BestMatchGraph&& o) noexcept {
    BestMatchGraph zero;
    std::swap(zero, o);
    std::swap(zero, *this);
  }

  // Add a relation(id->r) for vertex identified by id.
  // Return true if id->r is a new relationship, else return false.
  bool AddRelation(int id, int r);
  // Query candinate vertex ids of vertex identified by id.
  std::vector<int> QueryCandinates(int id);
private:
  std::map<int, std::multiset<int>> key2relations_;  // vertex id => relation(vertex to another vertex)
  std::map<int, Candinates>         key2candinates_; // vertex id => candinate vertexes
};

// This is a structure for compute the most similiar rectangle of certain rectangle.
// It has 2 methods:
// 0. Set the target rectangle in Constructor.
// 1. Add a similar rectangle with their ID, width and height.
// 2. Get the most similiar rectangle ID.
class SimiliarRectSolver
{
public:
  struct Node
  {
    struct greater
    {
      bool operator()(const Node& a, const Node& b) {
        return a.score_ > b.score_;
      }
    };
    float score_ = 0.f;
    int   id_ = -1;
    Node() = default;
    Node(float score, int id)
      : score_(score), id_(id) {
    }
  };
  SimiliarRectSolver(float w, float h)
    : targetW_(w), targetH_(h), q_(Node::greater()) {
  }
  ~SimiliarRectSolver() {
  }
  void addSimiliarRect(int id, float w, float h);
  int  getMostSimiliarRectID();
private:
  float targetH_, targetW_;
  std::priority_queue<Node, std::vector<Node>, Node::greater> q_;
};

// "srs" means Simplify Rectangles Algorithm.
namespace sra
{
// This a algorithm for simplify the related rectangle areas of a Key Point.
// We call it "Position Reference Algorithm".
// It has ? methods:
// 0. Set related rectangles in Constructor.(this class only hold reference to those rectangles)
// 1. Set a Key Point's position.
// 2. Judge which rectangle the KP should belong to.
class PosRefAlgorithm
{
  // Core implement for only 2 rectangles!
  class PosRef
  {
  public:
    PosRef(const cv::Rect2f& r1, const cv::Rect2f& r2, const cv::Point2f& kp);
    int getKPOwner();

  private:
    const cv::Rect2f &r1_, &r2_;
    const cv::Point2f &kp_;
    enum class OverlapState
    {
      DIAGONAL,
      CONTAIN,
      SIDE
    };
    OverlapState state_;
  };
public:
  PosRefAlgorithm(const std::vector<cv::Rect2f>& overlap)
    : overlap_rects_(overlap)
  {
    assert(overlap.size() > 1);
  }
  void setKeyPoint(const cv::Point2f& kp);
  int getKPOwner();
private:
  const std::vector<cv::Rect2f> &overlap_rects_;
  cv::Point2f kp_;
};


} // namespace zxm::srs


} // namespace zxm

#ifdef DEBUG
#include <cstdio>
#include <thread>
#include <sstream>

#define D_PRINTF(...) do{ \
  std::stringstream ss; \
  ss << std::this_thread::get_id(); \
  printf("\nThread %s\n  ", ss.str().c_str()); \
  printf(__VA_ARGS__); \
}while(0)

#define D_BLOCK(s) do{s}while(0)

#else

#define D_PRINTF(...)

#endif // DEBUG

#endif // !__VCC_ZXM_UTILITY_H__