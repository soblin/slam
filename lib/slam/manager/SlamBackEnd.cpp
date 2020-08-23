#include <slam/geometry/PointCloudMap.h>
#include <slam/loop_closure/P2oDriver2D.h>
#include <slam/manager/ParamServer.h>
#include <slam/manager/SlamBackEnd.h>

namespace slam {

void SlamBackEnd::Initialize() {
  m_cloud_map_ptr = PointCloudMapSingleton::GetCloudMap();
}

void SlamBackEnd::Initialize(PoseGraph *p) {
  Initialize();
  m_pose_graph_ptr = p;
}

Pose2D SlamBackEnd::AdjustPoses() {
  m_new_poses.clear();

  P2oDriver2D p2o;
  const int N = ParamServer::Get("SlamBackEnd_P2O_N");
  p2o.DoP2o(*m_pose_graph_ptr, m_new_poses, N);

  return m_new_poses.back();
}

void SlamBackEnd::RemakeMaps() {
  std::vector<PoseNode *> &pnodes = m_pose_graph_ptr->nodes();
  for (size_t i = 0; i < m_new_poses.size(); ++i) {
    Pose2D &npose = m_new_poses[i];
    PoseNode *pnode = pnodes[i];
    pnode->SetPose(npose);
  }

  m_cloud_map_ptr->RemakeMaps(m_new_poses);
}

} // namespace slam
