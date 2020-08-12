#include <cmath>
#include <slam/icp/PoseEstimatorICP.h>
#include <slam/manager/ParamServer.h>

namespace slam {

void PoseEstimatorICP::Initialize() {
  m_optimizer_ptr->Initialize();
  m_associator_ptr->Initialize();
}

double PoseEstimatorICP::EstimatePose(const Pose2D &initPose,
                                      Pose2D &estimatePose) {
  double eval_min = HUGE_VAL;

  double eval = 0;
  double eval_old = eval_min;
  Pose2D pose = initPose;
  Pose2D pose_min = initPose;

  const double val_diff_thresh =
      ParamServer::Get("PoseEstimatorICP_VAL_DIFF_THRESH");
  const int max_iteration = ParamServer::Get("PoseEstimatorICP_ITERATION");
  for (int i = 0;
       std::abs(eval_old - eval) > val_diff_thresh && i < max_iteration; ++i) {
    if (i > 0)
      eval_old = eval;

    // find the pair of points
    // member function SetScanPair sets the base points to m_asssociator_ptr
    double ratio = m_associator_ptr->FindCorrespondence(m_cur_scan, pose);
    Pose2D newPose;
    // do scan matching between the pairs
    m_optimizer_ptr->SetPoints(m_associator_ptr->cur_points(),
                               m_associator_ptr->ref_points());

    eval = m_optimizer_ptr->OptimizePose(pose, newPose);

    // update
    pose = newPose;

    // update minimal
    if (eval < eval_min) {
      pose_min = newPose;
      eval_min = eval;
    }
  }

  m_optimizer_ptr->GetMatchRate(m_matched_rate);
  m_used_points_num = m_associator_ptr->cur_points().size();
  estimatePose = pose_min;

  return eval_min;
}

} /* namespace slam */
