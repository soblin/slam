#include <slam/loop_closure/P2oDriver2D.h>
#include <slam/p2o/p2o.h>

static double normalize(double th) {
  while (th > M_PI)
    th -= M_PI;
  while (th < -M_PI)
    th += M_PI;

  return th;
}

namespace slam {

void P2oDriver2D::DoP2o(PoseGraph &graph, std::vector<Pose2D> &newPoses,
                        int N) {
  std::vector<PoseNode *> &nodes = graph.nodes();
  std::vector<PoseArc *> &arcs = graph.arcs();

  std::vector<p2o::Pose2D> pnodes;
  for (size_t i = 0; i < nodes.size(); ++i) {
    PoseNode *node = nodes[i];
    Pose2D pose = node->pose();
    pnodes.emplace_back(pose.tx(), pose.ty(), normalize(pose.th()));
  }

  p2o::Con2DVec pcons;
  for (size_t i = 0; i < arcs.size(); ++i) {
    PoseArc *arc = arcs[i];
    PoseNode *src = arc->src();
    PoseNode *dst = arc->dst();
    Pose2D &relPose = arc->rel_pose();
    p2o::Con2D con;
    con.id1 = src->id();
    con.id2 = dst->id();
    con.t = p2o::Pose2D(relPose.tx(), relPose.ty(), normalize(relPose.th()));
    for (int k = 0; k < 3; ++k) {
      for (int m = 0; m < 3; ++m)
        con.info(k, m) = arc->inf_mat()(k, m);
    }
    pcons.push_back(con);
  }

  p2o::Optimizer2D opt;
  std::vector<p2o::Pose2D> result = opt.optimizePath(pnodes, pcons, N);

  for (size_t i = 0; i < result.size(); ++i) {
    p2o::Pose2D newPose = result[i];
    newPoses.emplace_back(newPose.x, newPose.y, normalize(newPose.th));
  }
}

} // namespace slam
