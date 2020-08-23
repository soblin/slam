#include <Eigen/Eigen>
#include <cmath>
#include <slam/fuser/CovarianceCalculator.h>
#include <slam/loop_closure/LoopDetectorSS.h>
#include <slam/manager/ParamServer.h>

static double normalize(double th) {
  while (th > M_PI)
    th -= M_PI;
  while (th < -M_PI)
    th += M_PI;

  return th;
}

namespace slam {

void LoopDetectorSS::Initialize() {
  m_radius = ParamServer::Get("LoopDetectorSS_RADIUS");
  m_acc_dist_thresh = ParamServer::Get("LoopDetectorSS_ACC_DIST_THRESH");
  m_score_thresh = ParamServer::Get("LoopDetectorSS_SCORE_THRESH");
}

bool LoopDetectorSS::DetectLoop(Scan2D *curScan, Pose2D &curPose, int cnt) {
  double acc_dist = m_cloud_map_ptr->acc_dist();
  double acc_dist_ = 0;
  const std::vector<Submap> &submaps = m_cloud_map_ptr->submaps();
  const std::vector<Pose2D> &poses = m_cloud_map_ptr->poses();

  double dmin = HUGE_VAL;
  size_t imin = 0, jmin = 0;
  Pose2D prevP;

  auto square = [](double x, double y) { return x * x + y * y; };

  for (size_t i = 0; i < submaps.size() - 1; ++i) {
    const Submap &submap = submaps[i];
    for (size_t j = submap.id_first(); j <= submap.id_last(); ++j) {
      Pose2D p = poses[j];
      acc_dist_ += std::hypot(p.tx() - prevP.tx(), p.ty() - prevP.ty());
      if (acc_dist - acc_dist_ < m_acc_dist_thresh) {
        i = submaps.size();
        break;
      }
      prevP = p;

      double d = square(curPose.tx() - p.tx(), curPose.ty() - p.ty());
      if (d < dmin) {
        dmin = d;
        imin = i;
        jmin = j;
      }
    }
  }

  if (dmin > m_radius * m_radius)
    return false;

  Submap &refSubmap = m_cloud_map_ptr->submaps()[imin];
  const Pose2D &initPose = poses[jmin];

  Pose2D revisitPose;
  bool flag =
      EstimateRevisitPose(curScan, refSubmap.points(), curPose, revisitPose);

  if (flag) {
    Eigen::Matrix3d icpCov;
    double ratio =
        m_pose_fuser_ptr->CalcICPCovariance(revisitPose, curScan, icpCov);

    LoopInfo info;
    info.pose = revisitPose;
    info.cov = icpCov;
    info.cur_ind = cnt;
    info.ref_ind = static_cast<int>(jmin);
    MakeLoopArc(info);

    Scan2D refScan;
    Pose2D spose = poses[refSubmap.id_first()];
    refScan.SetId(info.ref_ind);
    refScan.SetScanedPoints(refSubmap.points());
    refScan.SetPose(spose);
    // LoopMatch lm(*curScan, refScan, info);
    m_loop_matches.emplace_back(*curScan, refScan, info);
  }

  return flag;
}

bool LoopDetectorSS::EstimateRevisitPose(
    const Scan2D *curScan, const std::vector<ScanPoint2D> &refPoints,
    const Pose2D &initPose, Pose2D &revisitPose) {
  m_data_associator_ptr->SetRefBase(refPoints);

  size_t usedNumMin = ParamServer::Get("LoopDetectorSS_USED_NUM_MIN");

  double rangeT = ParamServer::Get("LoopDetectorSS_RANGE_T");
  double rangeA = ParamServer::Get("LoopDetectorSS_RANGE_A");
  double dd = ParamServer::Get("LoopDetectorSS_DD");
  double da = ParamServer::Get("LoopDetectorSS_DA");
  double matchThresh = ParamServer::Get("LoopDetectorSS_MATCH_THRESH");
  double matchRateMax = 0;
  std::vector<double> scores;
  double scoreMin = HUGE_VAL;
  std::vector<Pose2D> candidates;
  for (double dy = -rangeT; dy <= rangeT; dy += dd) {
    double y = initPose.ty() + dy;
    for (double dx = -rangeT; dx <= rangeT; dx += dd) {
      double x = initPose.tx() + dx;
      for (double dth = -rangeA; dth <= rangeA; dth += da) {
        double th = normalize(initPose.th() + th);
        Pose2D pose(x, y, th);
        double mratio =
            m_data_associator_ptr->FindCorrespondence(curScan, pose);
        size_t usedNum = m_data_associator_ptr->cur_points().size();

        if (usedNum < usedNumMin or mratio < 0.9)
          // skip bad score
          continue;

        m_cost_function_ptr->SetPoints(m_data_associator_ptr->cur_points(),
                                       m_data_associator_ptr->ref_points());
        double score = m_cost_function_ptr->CalcValue(x, y, th);
        double match_rate = m_cost_function_ptr->GetMatchRate();
        if (match_rate > matchThresh) {
          candidates.emplace_back(pose);
          if (score < scoreMin)
            scoreMin = score;
          scores.push_back(score);
        }
      }
    }
  }

  if (candidates.size() == 0)
    return false;

  Pose2D best;
  double smin = HUGE_VAL;
  m_estimator_ptr->SetScanPair(curScan, refPoints);
  for (size_t i = 0; i < candidates.size(); ++i) {
    Pose2D p = candidates[i];
    Pose2D estP;
    double score = m_estimator_ptr->EstimatePose(p, estP);
    double match_rate = m_estimator_ptr->GetMatchRate();
    size_t usedNum = m_estimator_ptr->GetUsedNum();
    if (score < smin && match_rate >= 0.9 && usedNum >= usedNumMin) {
      smin = score;
      best = estP;
    }
  }

  if (smin <= m_score_thresh) {
    revisitPose = best;
    return true;
  }

  return false;
}

void LoopDetectorSS::MakeLoopArc(LoopInfo &info) {
  if (info.arcked)
    return;

  info.SetArcked(true);

  Pose2D srcPose = m_cloud_map_ptr->poses()[info.ref_ind];
  Pose2D dstPose(info.pose.tx(), info.pose.ty(), info.pose.th());
  Pose2D relPose;
  Pose2D::CalcRelativePose(dstPose, srcPose, relPose);

  Eigen::Matrix3d cov;
  CovarianceCalculator::RotateCovariance(srcPose, info.cov, cov, true);

  PoseArc *arc = m_pg->MakeArc(info.ref_ind, info.cur_ind, relPose, cov);
  m_pg->AddArc(arc);
}
} // namespace slam
