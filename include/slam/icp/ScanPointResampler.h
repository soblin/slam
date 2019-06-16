#ifndef SCAN_POINT_RESAMPLER_H
#define SCAN_POINT_RESAMPLER_H

#include <slam/geometry/ScanPoint2D.h>

namespace slam {

class ScanPointResampler {
private:
  bool FindInterpolatePointImpl(const ScanPoint2D &curPoint,
                                const ScanPoint2D &prevPoint,
                                ScanPoint2D &nextPoint, bool &inserted,
                                double &acc_dist, double interval_thresh,
                                double interpolate_thresh);

public:
  ScanPointResampler() {}
  ~ScanPointResampler() {}

  void ResamplePoints(Scan2D *scan);
  bool FindInterpolatePoint(const ScanPoint2D &curPoint,
                            const ScanPoint2D &prevPoint,
                            ScanPoint2D &nextPoint, bool &inserted,
                            double &acc_dist);

  friend class ScanPointResamplerTestFriend;
};

} // namespace slam
#endif /* SCAN_POINT_RESAMPLER_H */
