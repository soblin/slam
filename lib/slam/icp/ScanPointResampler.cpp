#include <cmath>
#include <slam/geometry/Scan2D.h>
#include <slam/icp/ScanPointResampler.h>
#include <slam/manager/ParamServer.h>

namespace slam {

bool ScanPointResampler::FindInterpolatePoint(const ScanPoint2D &curPoint,
                                              const ScanPoint2D &prevPoint,
                                              ScanPoint2D &insertPoint,
                                              bool &inserted, double &acc_dist,
                                              double interval_thresh,
                                              double interpolate_thresh) {

  double dx = curPoint.x() - prevPoint.x();
  double dy = curPoint.y() - prevPoint.y();

  double L = std::hypot(dx, dy);

  // too dense
  // so still there's no need to interpolate
  if (acc_dist + L < interval_thresh) {
    acc_dist += L;
    return false;
  }
  // too far
  else if (acc_dist + L >= interpolate_thresh) {
    insertPoint.SetData(curPoint.x(), curPoint.y());
  } else {
    // the distance is more than the interval and less than the threshold
    // so interpolate a new point
    const double dist_interval =
        ParamServer::Get("ScanPointResampler_DIST_INTERVAL");
    double ratio = (dist_interval - acc_dist) / L;
    double x2 = dx * ratio + prevPoint.x();
    double y2 = dy * ratio + prevPoint.y();
    insertPoint.SetData(x2, y2);
    inserted = true;
  }

  return true;
}

bool ScanPointResampler::FindInterpolatePoint(const ScanPoint2D &curPoint,
                                              const ScanPoint2D &prevPoint,
                                              ScanPoint2D &insertPoint,
                                              bool &inserted,
                                              double &acc_dist) {
  const double dist_interval =
      ParamServer::Get("ScanPointResampler_DIST_INTERVAL");
  const double interpolate_thresh =
      ParamServer::Get("ScanPointResampler_DIST_INTERPOLATE_THRESH");

  return FindInterpolatePoint(curPoint, prevPoint, insertPoint, inserted,
                              acc_dist, dist_interval, interpolate_thresh);
}

void ScanPointResampler::ResamplePoints(Scan2D *scan) {
  const std::vector<ScanPoint2D> &scaned_points = scan->scaned_points();
  if (scaned_points.size() == 0)
    return;

  std::vector<ScanPoint2D> resampled_points;

  double acc_dist = 0;

  ScanPoint2D point = scaned_points[0];
  decltype(point) prevPoint = point;
  decltype(point) insertPoint(point.x(), point.y());

  resampled_points.emplace_back(point.x(), point.y());

  for (size_t i = 1; i < scaned_points.size(); ++i) {
    point = scaned_points[i];
    bool inserted = false;

    bool exist =
        FindInterpolatePoint(point, prevPoint, insertPoint, inserted, acc_dist);

    if (exist) {
      resampled_points.emplace_back(insertPoint);
      prevPoint = insertPoint;
      acc_dist = 0;
      if (inserted)
        i--;
    } else
      prevPoint = point;
  }

  scan->SetScanedPoints(resampled_points);
}
} // namespace slam
