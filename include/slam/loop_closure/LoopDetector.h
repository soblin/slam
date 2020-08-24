#ifndef LOOP_DETECTOR_H
#define LOOP_DETECTOR_H

#include <slam/geometry/Pose2D.h>
#include <slam/geometry/Scan2D.h>
#include <slam/loop_closure/PoseGraph.h>
#include <vector>

namespace slam {

struct LoopInfo {
  LoopInfo() : arcked(false), cur_ind(-1), ref_ind(-1), score(0.0) {}
  bool arcked;
  int cur_ind;
  int ref_ind;
  Pose2D pose;
  double score;
  Eigen::Matrix3d cov;

  void SetArcked(bool b) { arcked = b; }
};

struct LoopMatch {
  LoopMatch() {}
  LoopMatch(Scan2D &cs, Scan2D &rs, LoopInfo &info) {
    cur_scan = cs;
    ref_scan = rs;
    info = info;
  }

  Scan2D cur_scan;
  Scan2D ref_scan;
  LoopInfo info;
};

class LoopDetector {
public:
  LoopDetector() {}
  virtual ~LoopDetector() {}

  virtual void Initialize() {}
  std::vector<LoopMatch> &loop_matches();
  void SetPoseGraph(PoseGraph *p);
  virtual bool DetectLoop(Scan2D *curScan, Pose2D &curPose, int cnt);

protected:
  PoseGraph *m_pose_graph_ptr;
  std::vector<LoopMatch> m_loop_matches;
};

inline void LoopDetector::SetPoseGraph(PoseGraph *p) { m_pose_graph_ptr = p; }

inline std::vector<LoopMatch> &LoopDetector::loop_matches() {
  return m_loop_matches;
}

} // namespace slam
#endif /* LOOP_DETECTOR_H */
