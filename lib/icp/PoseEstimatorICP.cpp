#include <cmath>
#include <slam/icp/PoseEstimatorICP.h>

namespace slam {

double PoseEstimatorICP::EstimatePose(Pose2D &initPose, Pose2D &estimatePose) {
  double eval_min = HUGE_VAL;
  double eval_diff_thresh = 0.000001;

  m_optimizer_ptr->SetValDiffThresh(eval_diff_thresh);
  m_optimizer_ptr->SetValThresh(0.2);

  double eval = 0;
  double eval_old = eval_min;
  Pose2D pose = initPose;
  Pose2D pose_min = initPose;

  for (int i = 0; std::abs(eval_old - eval) > eval_diff_thresh && i < 100;
       ++i) {
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

    pose = newPose;

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
