#ifndef P2O_DRIVER2D_H
#define P2O_DRIVER2D_H

#include <slam/loop_closure/PoseGraph.h>
#include <vector>

namespace slam {

class P2oDriver2D {
public:
  P2oDriver2D() {}
  ~P2oDriver2D() {}

  void DoP2o(PoseGraph &graph, std::vector<Pose2D> &newPoses, int N);
};

} // namespace slam
#endif /* P2O_DRIVER2D_H */
