#ifndef LOOP_DETECTOR_SS_H
#define LOOP_DETECTOR_SS_H

#include <slam/fuser/PoseFuser.h>
#include <slam/geometry/PointCloudMapLP.h>
#include <slam/icp/CostFunction.h>
#include <slam/icp/DataAssociator.h>
#include <slam/icp/PoseEstimatorICP.h>
#include <slam/loop_closure/LoopDetector.h>

namespace slam {

class LoopDetectorSS : public LoopDetector {
public:
  LoopDetectorSS() {}
  ~LoopDetectorSS() {}

  void Initialize();
  virtual bool DetectLoop(Scan2D *curScan, Pose2D &curPose, int cnt) override;
  bool EstimateRevisitPose(const Scan2D *curScan,
                           const std::vector<ScanPoint2D> &refPoints,
                           const Pose2D &initPose, Pose2D &revistPose);
  void MakeLoopArc(LoopInfo &info);

public:
  void SetPoseEstimator(PoseEstimatorICP *p);
  void SetPoseFuser(PoseFuser *p);
  void SetDataAssociator(DataAssociator *p);
  void SetCostFunction(CostFunction *p);
  void SetPointCloudMap(PointCloudMapLP *p);

private:
  double m_radius = 0;
  double m_acc_dist_thresh = 0;
  double m_score_thresh = 0;

  PointCloudMapLP *m_cloud_map_ptr = nullptr;
  CostFunction *m_cost_function_ptr = nullptr;
  PoseEstimatorICP *m_estimator_ptr = nullptr;
  DataAssociator *m_data_associator_ptr = nullptr;
  PoseFuser *m_pose_fuser_ptr = nullptr;
};

inline void LoopDetectorSS::SetPoseEstimator(PoseEstimatorICP *p) {
  m_estimator_ptr = p;
}
inline void LoopDetectorSS::SetPoseFuser(PoseFuser *p) { m_pose_fuser_ptr = p; }
inline void LoopDetectorSS::SetDataAssociator(DataAssociator *p) {
  m_data_associator_ptr = p;
}
inline void LoopDetectorSS::SetCostFunction(CostFunction *p) {
  m_cost_function_ptr = p;
}
inline void LoopDetectorSS::SetPointCloudMap(PointCloudMapLP *p) {
  m_cloud_map_ptr = p;
}

} // namespace slam

#endif /* LOOP_DETECTOR_SS_H */
