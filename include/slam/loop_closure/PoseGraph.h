#ifndef POSE_GRAPH_H
#define POSE_GRAPH_H

#include <Eigen/Eigen>
#include <slam/geometry/Pose2D.h>
#include <vector>

namespace slam {

class PoseArc;

class PoseNode {
public:
  PoseNode();
  PoseNode(const Pose2D &pose);
  ~PoseNode() {}

  void Initialize();
  void SetPose(const Pose2D &pose);
  void SetId(int id);
  void AddArc(PoseArc *p);

public:
  Pose2D &pose();
  std::vector<PoseArc *> &arcs();
  int id();

private:
  Pose2D m_pose;                 // the pose of this node
  std::vector<PoseArc *> m_arcs; // the arcs pointing to this node
  int m_id;
};

class PoseArc {
public:
  PoseArc();
  PoseArc(PoseNode *s, PoseNode *d, Pose2D &rel, const Eigen::Matrix3d inf);
  ~PoseArc() {}

  void Setup(PoseNode *s, PoseNode *d, const Pose2D &rel,
             const Eigen::Matrix3d inf);

public:
  PoseNode *src();
  PoseNode *dst();
  Pose2D &rel_pose();
  Eigen::Matrix3d &inf_mat();

private:
  PoseNode *m_src = nullptr;
  PoseNode *m_dst = nullptr;
  Pose2D m_rel_pose;
  Eigen::Matrix3d m_inf_mat; // Information matrix
};

class PoseGraph {
public:
  PoseGraph() {}
  ~PoseGraph();
  void Initiaize();
  void Reset();
  PoseNode *AllocNode();
  PoseArc *AllocArc();
  PoseNode *AddNode(const Pose2D &pose);
  void AddNode(PoseNode *n1, const Pose2D &pose);
  PoseNode *FindNode(int id);
  void AddArc(PoseArc *arc);
  PoseArc *MakeArc(int srcId, int dstId, const Pose2D &relPose,
                   const Eigen::Matrix3d &cov);
  PoseArc *FindArc(int srcId, int dstId);

public:
  std::vector<PoseNode *> &nodes();
  std::vector<PoseArc *> &arcs();

private:
  int m_pool_sz = -1;
  std::vector<PoseNode> m_nodes_pool;
  std::vector<PoseArc> m_arcs_pool;
  std::vector<PoseNode *> m_nodes;
  std::vector<PoseArc *> m_arcs;
};

inline void PoseNode::Initialize() {
  m_id = -1;
  m_arcs.clear();
}

inline void PoseNode::SetPose(const Pose2D &pose) { m_pose = pose; }

inline void PoseNode::SetId(int id) { m_id = id; }

inline void PoseNode::AddArc(PoseArc *p) { m_arcs.emplace_back(p); }

inline Pose2D &PoseNode::pose() { return m_pose; }

inline std::vector<PoseArc *> &PoseNode::arcs() { return m_arcs; }

inline int PoseNode::id() { return m_id; }

inline PoseNode *PoseArc::src() { return m_src; }

inline PoseNode *PoseArc::dst() { return m_dst; }

inline Pose2D &PoseArc::rel_pose() { return m_rel_pose; }

inline Eigen::Matrix3d &PoseArc::inf_mat() { return m_inf_mat; }

inline std::vector<PoseNode *> &PoseGraph::nodes() { return m_nodes; }

inline std::vector<PoseArc *> &PoseGraph::arcs() { return m_arcs; }

} // namespace slam
#endif /* POSE_GRAPH_H */
