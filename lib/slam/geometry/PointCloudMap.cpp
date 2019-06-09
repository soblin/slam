#include <slam/geometry/PointCloudMap.h>
#include <slam/parameters.h>

namespace slam {

PointCloudMap::PointCloudMap() {
  m_global_map.reserve(param::PointCloudMap_MAX_POINT_NUM);
}

PointCloudMap::~PointCloudMap() {
  m_poses.reserve(0);
  m_global_map.reserve(0);
}

} /* namespace slam */
