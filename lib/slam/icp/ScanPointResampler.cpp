#include <cmath>
#include <slam/geometry/Scan2D.h>
#include <slam/icp/ScanPointResampler.h>
#include <slam/parameters.h>

namespace slam {

bool ScanPointResampler::FindInterpolatePointImpl(
    const ScanPoint2D &curPoint, const ScanPoint2D &prevPoint,
    ScanPoint2D &insertPoint, bool &inserted, double &acc_dist,
    double interval_thresh, double interpolate_thresh) {

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
    double ratio = (param::ScanPointResampler_DIST_INTERVAL - acc_dist) / L;
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
  return FindInterpolatePointImpl(
      curPoint, prevPoint, insertPoint, inserted, acc_dist,
      param::ScanPointResampler_DIST_INTERVAL,
      param::ScanPointResampler_DIST_INTERPOLATE_THRESH);
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

  for (unsigned i = 1; i < scaned_points.size(); ++i) {
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
