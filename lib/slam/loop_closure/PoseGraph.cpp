#include <slam/loop_closure/PoseGraph.h>
#include <slam/manager/ParamServer.h>

static Eigen::Matrix3d svdInverse(const Eigen::Matrix3d &A) {

  size_t m = A.rows();
  size_t n = A.cols();

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU |
                                               Eigen::ComputeThinV);

  Eigen::MatrixXd eU = svd.matrixU();
  Eigen::MatrixXd eV = svd.matrixV();
  Eigen::VectorXd eS = svd.singularValues();

  Eigen::MatrixXd M1(m, n);
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      M1(i, j) = eU(j, i) / eS[i];
    }
  }

  Eigen::Matrix3d IA;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      IA(i, j) = 0;
      for (size_t k = 0; k < n; ++k) {
        IA(i, j) += eV(i, k) * M1(k, j);
      }
    }
  }

  return IA;
}

namespace slam {

PoseNode::PoseNode(const Pose2D &pose) { m_pose = pose; }

PoseArc::PoseArc() : m_src(nullptr), m_dst(nullptr) {}
PoseArc::PoseArc(PoseNode *s, PoseNode *d, Pose2D &rel,
                 const Eigen::Matrix3d inf) {
  Setup(s, d, rel, inf);
}

void PoseArc::Setup(PoseNode *s, PoseNode *d, const Pose2D &rel,
                    const Eigen::Matrix3d inf) {
  m_src = s;
  m_dst = d;
  m_rel_pose = rel;
  m_inf_mat = inf;
}

PoseGraph::~PoseGraph() {
  m_nodes_pool.clear();
  m_arcs_pool.clear();
}

void PoseGraph::Initialize() {
  m_pool_sz = ParamServer::Get("PoseGraph_POOL_SIZE");
  m_nodes_pool.reserve(m_pool_sz);
  m_arcs_pool.reserve(m_pool_sz);
}

void PoseGraph::Reset() {
  m_nodes_pool.clear();
  m_arcs_pool.clear();
  m_nodes.clear();
  m_arcs.clear();
}

PoseNode *PoseGraph::AllocNode() {
  if (m_nodes_pool.size() >= m_pool_sz)
    return nullptr;

  m_nodes_pool.emplace_back(PoseNode());
  return &(m_nodes_pool.back());
}

PoseArc *PoseGraph::AllocArc() {
  if (m_arcs_pool.size() >= m_pool_sz)
    return nullptr;

  m_arcs_pool.emplace_back(PoseArc());
  return &(m_arcs_pool.back());
}

PoseNode *PoseGraph::AddNode(const Pose2D &pose) {
  PoseNode *n1 = AllocNode();
  AddNode(n1, pose);

  return n1;
}

void PoseGraph::AddNode(PoseNode *n1, const Pose2D &pose) {
  n1->SetId(static_cast<int>(m_nodes.size()));
  n1->SetPose(pose);
  m_nodes.emplace_back(n1);
}

PoseNode *PoseGraph::FindNode(int id) {
  for (size_t i = 0; i < m_nodes.size(); ++i) {
    PoseNode *n = m_nodes[i];
    if (n->id() == id)
      return n;
  }

  return nullptr;
}

void PoseGraph::AddArc(PoseArc *arc) {
  arc->src()->AddArc(arc);
  arc->dst()->AddArc(arc);
  m_arcs.emplace_back(arc);
}

PoseArc *PoseGraph::MakeArc(int srcId, int dstId, const Pose2D &relPose,
                            const Eigen::Matrix3d &cov) {
  Eigen::Matrix3d inf = svdInverse(cov);

  PoseNode *src = m_nodes[srcId];
  PoseNode *dst = m_nodes[dstId];

  PoseArc *arc = AllocArc();
  arc->Setup(src, dst, relPose, inf);

  return arc;
}

PoseArc *PoseGraph::FindArc(int srcId, int dstId) {
  for (size_t i = 0; i < m_arcs.size(); ++i) {
    PoseArc *a = m_arcs[i];
    if (a->src()->id() == srcId && a->dst()->id() == dstId)
      return a;
  }

  return nullptr;
}

} // namespace slam
