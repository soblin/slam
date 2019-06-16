#include <cmath>
#include <iostream>
#include <slam/icp/ScanMatcher2D.h>
#include <slam/parameters.h>

namespace slam {

static bool is_first = true;

bool ScanMatcher2D::MatchScan(Scan2D &curScan) {
  PointCloudMap *cloud_map_ptr = PointCloudMapSingleton::GetCloudMap();

  if (is_first) {
    GrowMap(curScan, m_init_pose);
    is_first = false;
    return true;
  }

  const auto prev_scan = cloud_map_ptr->GetLastScan();
  // compare the previous pose and curent pose by odometry
  // in the frame of previous pose
  Pose2D odoMotion;
  Pose2D::CalcRelativePose(curScan.pose(), prev_scan.pose(), odoMotion);

  // m_point_cloud_ptr stores the optimized (ICP-ed) poses
  Pose2D lastPose = cloud_map_ptr->GetLastPose();

  // predictedPose is the predicted pose based on odoMotion from lastPose
  // used as the initial guess in ICP algorithm
  Pose2D predictedPose;
  Pose2D::CalcGlobalPose(odoMotion, lastPose, predictedPose);

  const Scan2D *refScan = m_ref_scan_maker_ptr->MakeRefScan();
  // set refScan as the reference PointClouds in ICP, use
  // DataAssociator->SetRefBase()
  m_estimator_ptr->SetScanPair(&curScan, refScan);

  Pose2D estimatedPose;

  double score = m_estimator_ptr->EstimatePose(predictedPose, estimatedPose);

  std::size_t usedNum = m_estimator_ptr->GetUsedNum();

  bool successful = (score <= param::ScanMatcher2D_SCORE_THRESH &&
                     usedNum >= param::ScanMatcher2D_SCORE_THRESH)
                        ? true
                        : false;

  // use raw odometry
  if (!successful)
    estimatedPose = predictedPose;

  // add the current scan with the estimated Pose
  GrowMap(curScan, estimatedPose);

  // for validation
  Pose2D estimatedMotion;
  Pose2D::CalcRelativePose(estimatedPose, lastPose, estimatedMotion);

  return successful;
}

void ScanMatcher2D::GrowMap(const Scan2D &scan, const Pose2D &pose) {
  PointCloudMap *cloud_map_ptr = PointCloudMapSingleton::GetCloudMap();

  const auto &scaned_points = scan.scaned_points();
  double tx = pose.tx();
  double ty = pose.ty();

  std::vector<ScanPoint2D> scaned_points_global;
  for (unsigned i = 0; i < scaned_points.size(); ++i) {
    const ScanPoint2D &scan_point = scaned_points[i];

    double x = pose.R00() * scan_point.x() + pose.R01() * scan_point.y() + tx;
    double y = pose.R10() * scan_point.x() + pose.R11() * scan_point.y() + ty;
    double nx = pose.R00() * scan_point.nx() + pose.R01() * scan_point.ny();
    double ny = pose.R10() * scan_point.nx() + pose.R11() * scan_point.ny();

    ScanPoint2D point(x, y);
    point.SetNormal(nx, ny);
    point.SetType(scan_point.type());
    scaned_points_global.emplace_back(point);
  }

  // register the new global points
  cloud_map_ptr->AddPose(pose);
  cloud_map_ptr->AddPoints(scaned_points_global);
  cloud_map_ptr->SetLastPose(pose);
  cloud_map_ptr->SetLastScan(scan);
}

} /* namespace slam */
