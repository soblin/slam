#include <slam/geometry/PointCloudMap.h>

namespace slam {

PointCloudMap::PointCloudMap() { m_global_map.reserve(MAX_POINT_NUM); }

PointCloudMap::~PointCloudMap() {
  m_poses.reserve(0);
  m_global_map.reserve(0);
}

} /* namespace slam */
