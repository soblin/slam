#ifndef POSE_OPTIMIZER_SL_H
#define POSE_OPTIMIZER_SL_H

#include <slam/icp/PoseOptimizer.h>

namespace slam {

class PoseOptimizerSL : public PoseOptimizer {
public:
  PoseOptimizerSL() : PoseOptimizer() {}
  virtual ~PoseOptimizerSL() {}

  virtual double OptimizePose(const Pose2D &initPose,
                              Pose2D &estimatedPose) override;

  virtual void Initialize() override;

private:
  double LineSearch(double ev0, Pose2D &pose, const Pose2D &direction);
  double ObjFunc(double tt, Pose2D &pose, const Pose2D &direction);
};

} // namespace slam
#endif /* POSE_OPTIMIZER_SL_H */
