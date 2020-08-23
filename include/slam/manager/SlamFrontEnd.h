#ifndef SLAM_FRONT_END_H
#define SLAM_FRONT_END_H

#include <slam/geometry/PointCloudMap.h>
#include <slam/icp/ScanMatcher2D.h>

namespace slam {

class SlamFrontEnd {
public:
  SlamFrontEnd() {}
  ~SlamFrontEnd() {}

  void Initialize();
  void Process(Scan2D &scan);

public:
  void SetScanMatcher2D(ScanMatcher2D *p);
  void SetDgCheck(bool p);

private:
  // initialized in Initialize()
  PointCloudMap *m_cloud_map_ptr = nullptr;
  ScanMatcher2D *m_scan_matcher_ptr = nullptr;
  void RegisterParams();
};

inline void SlamFrontEnd::SetScanMatcher2D(ScanMatcher2D *p) {
  m_scan_matcher_ptr = p;
}
inline void SlamFrontEnd::SetDgCheck(bool p) {
  m_scan_matcher_ptr->SetDgCheck(p);
}

} /* namespace slam */
#endif /* slam_front_end_h */
