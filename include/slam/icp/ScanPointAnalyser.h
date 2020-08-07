#ifndef SCAN_POINT_ANALYSER_H
#define SCAN_POINT_ANALYSER_H

#include <slam/geometry/ScanPoint2D.h>
#include <vector>

namespace slam {

class ScanPointAnalyser {
public:
  ScanPointAnalyser() = default;
  ~ScanPointAnalyser() = default;

  void Initialize(){};
  void AnalysePoints(std::vector<ScanPoint2D> &points);
  bool CalcNormal(int index, const std::vector<ScanPoint2D> &points, int dir,
                  Vector2D &ret);
  void AnalysePoints(std::vector<ScanPoint2D> &points, double invalid_deg,
                     double corner_thresh_deg, double corner_thresh_cos);
  bool CalcNormal(int index, const std::vector<ScanPoint2D> &points, int dir,
                  Vector2D &ret, double fpdmin, double fpdmax);
};

} // namespace slam
#endif /* SCAN_POINT_ANALYSER_H */
