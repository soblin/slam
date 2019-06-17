#ifndef POINT_CLOUD_MAP_GT_H
#define POINT_CLOUD_MAP_GT_H

#include <PointCloudMap.h>

namespace slam{

class PointCloudMapGT : public PointCloudMap {
 private:
  std::vector<ScanPoint2D> m_all_points;
  NNGridTable m_grid_table;

 public:
  PointCloudMapGT() : PointCloudMap() {
    m_all_points.reserve(param::PointCloudMap_MAX_POINT_NUM)
  }
  virtual ~PointCloudMappGT() {
    m_all_points.clear();
  }
  
}

} // namespace slam
#endif /* POINT_CLOUD_MAP_GT_H */
