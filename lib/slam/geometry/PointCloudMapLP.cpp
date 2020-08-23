#include <cmath>
#include <slam/geometry/NNGridTable.h>
#include <slam/geometry/PointCloudMapLP.h>
#include <slam/manager/ParamServer.h>
#include <utility>

namespace slam {

Submap::Submap() : m_acc_dist(0), m_id_first(0), m_id_last(0) {}
Submap::Submap(double a, size_t f) : m_id_last(-1) {
  m_acc_dist = a;
  m_id_first = f;
}

void Submap::AddPoints(const std::vector<ScanPoint2D> &points) {
  for (auto &&point : points)
    m_points.emplace_back(point);
}

std::vector<ScanPoint2D> Submap::SubsamplePoints(int nthre) {
  NNGridTable nntab;
  for (size_t i = 0; i < m_points.size(); ++i) {
    ScanPoint2D &lp = m_points[i];
    nntab.AddPoint(&lp);
  }

  std::vector<ScanPoint2D> sps;
  nntab.MakeCellPoints(nthre, sps);

  return sps;
}

void PointCloudMapLP::Initialize() {
  int reserve_size = ParamServer::Get("PointCloudMap_MAX_POINT_NUM");
  m_global_map.reserve(reserve_size);
}

void PointCloudMapLP::AddPose(const Pose2D &p) {
  if (m_poses.size() > 0) {
    Pose2D pp = m_poses.back();
    m_acc_dist += std::hypot(p.tx() - pp.tx(), p.ty() - pp.ty());
  } else
    m_acc_dist += std::hypot(p.tx(), p.ty());

  m_poses.emplace_back(p);
}

void PointCloudMapLP::AddPoint(const ScanPoint2D &point) {
  std::vector<ScanPoint2D> tmp;
  tmp.emplace_back(point);
  AddPoints(tmp);
}

void PointCloudMapLP::AddPoint(ScanPoint2D &&point) {
  std::vector<ScanPoint2D> tmp;
  tmp.emplace_back(std::forward<ScanPoint2D>(point));
  AddPoints(tmp);
}

void PointCloudMapLP::AddPoints(const std::vector<ScanPoint2D> &points) {
  Submap &curSubMap = m_submaps.back();
  double acc_dist_thresh = ParamServer::Get("PointCloudMapLP_ACC_DIST_THRESH");
  int n_thresh = ParamServer::Get("PointCloudMapGT_CELL_POINT_NUM_THRESH");

  if (m_acc_dist - curSubMap.acc_dist() >= acc_dist_thresh) {
    size_t size = m_poses.size();
    curSubMap.id_last() = size - 1;

    curSubMap.points() = curSubMap.SubsamplePoints(n_thresh);

    Submap submap(m_acc_dist, size);
    submap.AddPoints(points);
    m_submaps.emplace_back(submap);
  } else
    curSubMap.AddPoints(points);
}

void PointCloudMapLP::MakeGlobalMap() {
  m_global_map.clear();
  m_local_map.clear();

  for (size_t i = 0; i < m_submaps.size() - 1; ++i) {
    Submap &submap = m_submaps[i];
    std::vector<ScanPoint2D> &mps = submap.points();
    for (size_t j = 0; j < mps.size(); ++j)
      m_global_map.emplace_back(mps[j]);

    if (i == m_submaps.size() - 2) {
      for (size_t j = 0; j < mps.size(); ++j)
        m_local_map.emplace_back(mps[j]);
    }
  }

  Submap &curSubMap = m_submaps.back();
  int n_thresh = ParamServer::Get("PointCloudMapGT_CELL_POINT_NUM_THRESH");
  std::vector<ScanPoint2D> sps = curSubMap.SubsamplePoints(n_thresh);
  for (size_t i = 0; i < sps.size(); ++i) {
    m_global_map.emplace_back(sps[i]);
    m_local_map.emplace_back(sps[i]);
  }
}

void PointCloudMapLP::MakeLocalMap() {
  m_local_map.clear();
  if (m_submaps.size() >= 2) {
    Submap &submap = m_submaps[m_submaps.size() - 2];
    std::vector<ScanPoint2D> &mps = submap.points();
    for (size_t i = 0; i < mps.size(); ++i)
      m_local_map.emplace_back(mps[i]);
  }

  Submap &curSubMap = m_submaps.back();
  int n_thresh = ParamServer::Get("PointCloudMapGT_CELL_POINT_NUM_THRESH");
  std::vector<ScanPoint2D> sps = curSubMap.SubsamplePoints(n_thresh);
  for (size_t i = 0; i < sps.size(); ++i)
    m_local_map.emplace_back(sps[i]);
}

void PointCloudMapLP::RemakeMaps(const std::vector<Pose2D> &newPoses) {
  for (size_t i = 0; i < m_submaps.size(); ++i) {
    Submap &submap = m_submaps[i];
    std::vector<ScanPoint2D> &mps = submap.points();
    for (size_t j = 0; j < mps.size(); ++j) {
      ScanPoint2D &mp = mps[j];
      size_t idx = mp.id();
      if (idx >= m_poses.size())
        continue;

      const Pose2D &oldPose = m_poses[idx];
      const Pose2D &newPose = newPoses[idx];
      double R1_00 = oldPose.R00(), R1_01 = oldPose.R01();
      double R1_10 = oldPose.R10(), R1_11 = oldPose.R11();
      double R2_00 = newPose.R00(), R2_01 = newPose.R01();
      double R2_10 = newPose.R10(), R2_11 = newPose.R11();

      ScanPoint2D lp1 = oldPose.ToRelativePoint(mp);
      ScanPoint2D lp2 = newPose.ToGlobalPoint(lp1);

      mp.SetXY(lp2.x(), lp2.y());
      double nx = R1_00 * mp.nx() + R1_10 * mp.ny();
      double ny = R1_01 * mp.nx() + R1_11 * mp.ny();
      double nx2 = R2_00 * nx + R2_01 * ny;
      double ny2 = R2_10 * nx + R2_11 * ny;

      mp.SetNormal(nx2, ny2);
    }
  }

  MakeGlobalMap();

  for (size_t i = 0; i < m_poses.size(); ++i)
    m_poses[i] = newPoses[i];

  m_last_pose = newPoses.back();
}

} // namespace slam
