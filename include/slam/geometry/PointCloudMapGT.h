#ifndef POINT_CLOUD_MAP_GT_H
#define POINT_CLOUD_MAP_GT_H

#include <slam/geometry/NNGridTable.h>
#include <slam/geometry/PointCloudMap.h>

namespace slam {

class PointCloudMapGT : public PointCloudMap {
public:
  PointCloudMapGT() : PointCloudMap() {}
  virtual ~PointCloudMapGT() { m_all_points.clear(); }

  virtual void Initialize() override;
  virtual void AddPose(const Pose2D &p) override;
  virtual void AddPoint(const ScanPoint2D &scan) override;
  virtual void AddPoint(ScanPoint2D &&scan) override;
  virtual void AddPoints(const std::vector<ScanPoint2D> &points) override;
  virtual void MakeGlobalMap() override;
  virtual void MakeLocalMap() override;
  virtual void RemakeMaps(const std::vector<Pose2D> &newPoses) override;
  void SubsamplePoints(std::vector<ScanPoint2D> &subs);

private:
  std::vector<ScanPoint2D> m_all_points;
  NNGridTable m_grid_table;
};
} // namespace slam
#endif /* POINT_CLOUD_MAP_GT_H */
