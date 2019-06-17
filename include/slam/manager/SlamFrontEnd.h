#ifndef SLAM_FRONT_END_H
#define SLAM_FRONT_END_H

#include <slam/geometry/PointCloudMap.h>
#include <slam/icp/ScanMatcher2D.h>

namespace slam {

class SlamFrontEnd {
private:
  PointCloudMap *m_point_cloud_map_ptr;
  ScanMatcher2D *m_scan_matcher_ptr;

public:
  inline void SetScanMatcher2D(ScanMatcher2D *p) { m_scan_matcher_ptr = p; }
  inline void SetRefScanMaker(RefScanMaker *p) {
    m_scan_matcher_ptr->SetRefScanMaker(p);
  }

public:
  SlamFrontEnd()
      : m_point_cloud_map_ptr(nullptr), m_scan_matcher_ptr(nullptr) {}
  ~SlamFrontEnd() {}

  void Process(const Scan2D &scan);
};

} /* namespace slam */
#endif /* slam_front_end_h */
