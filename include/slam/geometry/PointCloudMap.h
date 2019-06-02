#ifndef POINT_CLOUD_MAP
#define POINT_CLOUD_MAP

#include <memory>
#include <vector>

#include <slam/geometry/Scan2D.h>
#include <slam/geometry/Pose2D.h>
#include <slam/geometry/ScanPoint2D.h>

namespace slam{

class PointCloudMap{
 protected:
  // accumulated poses and scaned points(global frame)
  std::vector<Pose2D> m_poses;
  std::vector<ScanPoint2D> m_global_map;

 public:
  inline const std::vector<Pose2D>& poses() const { return m_poses; }
  inline const std::vector<ScanPoint2D>& global_map() const { return m_global_map; }
  
 public:
  static const int MAX_POINT_NUM = 1000000;

  PointCloudMap();
  ~PointCloudMap();
  virtual void AddPose(const Pose2D& pose) = 0;
  virtual void AddPoint(const ScanPoint2D& scan) = 0;
  virtual void AddPoints(const std::vector<ScanPoint2D>& scans) = 0;
};

using PointCloudMapPtr = std::shared_ptr<PointCloudMap>;

} /* namespace slam */
#endif
