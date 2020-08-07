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

private:
  double m_val_diff_thresh = 0;
  int m_max_iteration = -1;
  double m_dd = 0;
  double m_da = 0;
  double m_error_thresh = 0;
  double m_search_range = 0;
};

} // namespace slam
#endif /* POSE_OPTIMIZER_SL_H */
