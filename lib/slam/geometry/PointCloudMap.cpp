#include <slam/geometry/PointCloudMap.h>

namespace slam {

PointCloudMapSingleton *PointCloudMapSingleton::m_instance_ptr = nullptr;
PointCloudMap *PointCloudMapSingleton::m_point_cloud_map_ptr = nullptr;

} /* namespace slam */
