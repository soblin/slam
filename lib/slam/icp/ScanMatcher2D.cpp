#include <cassert>
#include <cmath>
#include <slam/icp/ScanMatcher2D.h>
#include <slam/manager/CounterServer.h>
#include <slam/manager/ParamServer.h>

namespace slam {

void ScanMatcher2D::Initialize() {
  assert(m_estimator_ptr != nullptr);
  m_estimator_ptr->Initialize();

  assert(m_ref_scan_maker_ptr != nullptr);
  m_ref_scan_maker_ptr->Initialize();

  if (m_scan_point_resampler_ptr != nullptr)
    m_scan_point_resampler_ptr->Initialize();

  if (m_scan_point_analyser_ptr != nullptr)
    m_scan_point_analyser_ptr->Initialize();

  if (m_pose_fuser_ptr != nullptr)
    m_pose_fuser_ptr->Initialize();

  m_cloud_map_ptr = PointCloudMapSingleton::GetCloudMap();

  m_score_thresh = ParamServer::Get("ScanMatcher2D_SCORE_THRESH");
  m_used_num_thresh = ParamServer::Get("ScanMatcher2D_USED_NUM_THRESH");
}

bool ScanMatcher2D::MatchScan(Scan2D &curScan) {
  // uniformalize the scaned points
  if (m_scan_point_resampler_ptr != nullptr)
    m_scan_point_resampler_ptr->ResamplePoints(&curScan);

  // calculate the normal vectors
  if (m_scan_point_analyser_ptr != nullptr)
    m_scan_point_analyser_ptr->AnalysePoints(curScan.scaned_points_ref());

  if (m_is_first) {
    GrowMap(curScan, m_init_pose);
    m_is_first = false;
    return true;
  }

  const auto prev_scan = m_cloud_map_ptr->GetLastScan();
  // compare the previous pose and curent pose by odometry
  // in the frame of previous pose
  Pose2D odoMotion;
  Pose2D::CalcRelativePose(curScan.pose(), prev_scan.pose(), odoMotion);

  // m_point_cloud_ptr stores the optimized (ICP-ed) poses
  Pose2D lastPose = m_cloud_map_ptr->GetLastPose();

  // predictedPose is the predicted pose based on odoMotion from lastPose
  // used as the initial guess in ICP algorithm
  Pose2D predictedPose;
  Pose2D::CalcGlobalPose(odoMotion, lastPose, predictedPose);

  const Scan2D *refScan = m_ref_scan_maker_ptr->MakeRefScan();
  // set refScan as the reference PointClouds in ICP, use
  // DataAssociator->SetRefBase()
  m_estimator_ptr->SetScanPair(&curScan, refScan);

  Pose2D estimatedPose;

  // predictedPose is the initial guess of estimatedPose!!
  double score = m_estimator_ptr->EstimatePose(predictedPose, estimatedPose);

  std::size_t usedNum = m_estimator_ptr->GetUsedNum();

  bool successful =
      (score <= m_score_thresh && usedNum >= m_used_num_thresh) ? true : false;

  if (m_dgcheck) {
    if (successful) {
      Pose2D fusedPose;
      Eigen::Matrix3d fusedCov;
      m_pose_fuser_ptr->SetRefBase(refScan);

      double ratio = m_pose_fuser_ptr->FusePose(
          &curScan, estimatedPose, odoMotion, lastPose, fusedPose, fusedCov);
      estimatedPose = fusedPose;

      m_cov = fusedCov;

      Eigen::Matrix3d covL;
      CovarianceCalculator::RotateCovariance(lastPose, fusedCov, covL, true);
      Eigen::Matrix3d tcov;
      CovarianceCalculator::AccumulateCovariance(lastPose, estimatedPose,
                                                 m_totalcov, covL, tcov);
      m_totalcov = tcov;
    } else {
      estimatedPose = predictedPose;
      m_pose_fuser_ptr->CalcOdometryCovariance(odoMotion, lastPose, m_cov);
    }
  } else {
    // use raw odometry
    if (!successful)
      estimatedPose = predictedPose;
  }
  // add the current scan with the estimated Pose
  GrowMap(curScan, estimatedPose);

  // for validation
  Pose2D estimatedMotion;
  Pose2D::CalcRelativePose(estimatedPose, lastPose, estimatedMotion);

  m_acc_dist = std::hypot(estimatedMotion.tx(), estimatedMotion.ty());

  return successful;
}

void ScanMatcher2D::GrowMap(const Scan2D &scan, const Pose2D &pose) {
  const auto &scaned_points = scan.scaned_points();
  double tx = pose.tx();
  double ty = pose.ty();

  // std::vector<ScanPoint2D> scaned_points_global;
  for (size_t i = 0; i < scaned_points.size(); ++i) {
    const ScanPoint2D &scan_point = scaned_points[i];
    if (scan_point.type() == ScanPoint2D::PointType::ISOLATE)
      continue;

    double x = pose.R00() * scan_point.x() + pose.R01() * scan_point.y() + tx;
    double y = pose.R10() * scan_point.x() + pose.R11() * scan_point.y() + ty;
    double nx = pose.R00() * scan_point.nx() + pose.R01() * scan_point.ny();
    double ny = pose.R10() * scan_point.nx() + pose.R11() * scan_point.ny();

    m_cloud_map_ptr->AddPoint(ScanPoint2D(x, y, nx, ny, scan_point.type()));
  }
  // register the new global points
  m_cloud_map_ptr->AddPose(pose);
  m_cloud_map_ptr->SetLastPose(pose);
  m_cloud_map_ptr->SetLastScan(scan);
  m_cloud_map_ptr->MakeLocalMap();
}
} /* namespace slam */
