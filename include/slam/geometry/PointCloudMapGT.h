#ifndef POINT_CLOUD_MAP_GT_H
#define POINT_CLOUD_MAP_GT_H

#include <slam/geometry/NNGridTable.h>
#include <slam/geometry/PointCloudMap.h>

namespace slam {

class PointCloudMapGT : public PointCloudMap {
private:
  std::vector<ScanPoint2D> m_all_points;
  NNGridTable m_grid_table;

public:
  PointCloudMapGT() : PointCloudMap() {
    m_all_points.reserve(param::PointCloudMap_MAX_POINT_NUM);
  }
  virtual ~PointCloudMapGT() { m_all_points.clear(); }

  virtual void AddPose(const Pose2D &p) override;
  virtual void AddPoint(const ScanPoint2D &scan) override;
  virtual void AddPoint(ScanPoint2D &&scan) override;
  virtual void AddPoints(const std::vector<ScanPoint2D> &points) override;
  virtual void MakeGlobalMap() override;
  virtual void MakeLocalMap() override;
  virtual void RemakeMaps(const std::vector<Pose2D> &newPoses) override;
  void SubsamplePoints(std::vector<ScanPoint2D> &subs);
};

} // namespace slam
#endif /* POINT_CLOUD_MAP_GT_H */
