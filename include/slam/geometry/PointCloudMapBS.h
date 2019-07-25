#ifndef POINT_CLOUD_MAP_BS
#define POINT_CLOUD_MAP_BS

#include <memory>
#include <vector>

#include <slam/geometry/PointCloudMap.h>

namespace slam {

class PointCloudMapBS : public PointCloudMap {
public:
  PointCloudMapBS() : PointCloudMap() {}
  virtual ~PointCloudMapBS() {}

  virtual void AddPose(const Pose2D &pose) override;
  virtual void AddPoint(const ScanPoint2D &scan) override;
  virtual void AddPoint(ScanPoint2D &&scan) override;
  virtual void AddPoints(const std::vector<ScanPoint2D> &scans) override;
  virtual void MakeGlobalMap() override;
  virtual void MakeLocalMap() override;
  virtual void RemakeMaps(const std::vector<Pose2D> &newposes) override;
};

} /* namespace slam */
#endif
