#ifndef SLAM_BACK_END_H
#define SLAM_BACK_END_H

#include <slam/geometry/PointCloudMap.h>
#include <slam/geometry/Pose2D.h>
#include <slam/loop_closure/PoseGraph.h>
#include <vector>

namespace slam {

class SlamBackEnd {
public:
  SlamBackEnd() {}
  ~SlamBackEnd() {
    m_new_poses.clear();
    m_cloud_map_ptr = nullptr;
    m_pose_graph_ptr = nullptr;
  }
  void Initialize();
  void Initialize(PoseGraph *p);
  Pose2D AdjustPoses();
  void RemakeMaps();

private:
  std::vector<Pose2D> m_new_poses;
  PointCloudMap *m_cloud_map_ptr = nullptr;
  PoseGraph *m_pose_graph_ptr = nullptr;
};

} // namespace slam

#endif /* SLAM_BACK_END_H */
