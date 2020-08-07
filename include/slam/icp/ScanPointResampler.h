#ifndef SCAN_POINT_RESAMPLER_H
#define SCAN_POINT_RESAMPLER_H

#include <slam/geometry/ScanPoint2D.h>

namespace slam {

class ScanPointResampler {
public:
  ScanPointResampler() {}
  ~ScanPointResampler() {}

  void Initialize() {}
  void ResamplePoints(Scan2D *scan);
  bool FindInterpolatePoint(const ScanPoint2D &curPoint,
                            const ScanPoint2D &prevPoint,
                            ScanPoint2D &nextPoint, bool &inserted,
                            double &acc_dist);
  bool FindInterpolatePoint(const ScanPoint2D &curPoint,
                            const ScanPoint2D &prevPoint,
                            ScanPoint2D &nextPoint, bool &inserted,
                            double &acc_dist, double interval_thresh,
                            double interpolate_thresh);
};

} // namespace slam
#endif /* SCAN_POINT_RESAMPLER_H */
