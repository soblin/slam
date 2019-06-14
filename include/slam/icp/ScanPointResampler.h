#ifndef SCAN_POINT_RESAMPLER_H
#define SCAN_POINT_RESAMPLER_H

#include <slam/geometry/ScanPoint2D.h>

namespace slam {

class ScanPointResampler {
public:
  ScanPointResampler() {}
  ~ScanPointResampler() {}

  void ResamplePoints(Scan2D *scan);
  bool FindInterpolatePoint(const ScanPoint2D &cp, const ScanPoint2D &pp,
                            ScanPoint2D &np, bool &inserted, double &acc_dist);
};

} // namespace slam
#endif /* SCAN_POINT_RESAMPLER_H */
