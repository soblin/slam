#ifndef POSE_OPTIMIZER_SD_H
#define POSE_OPTIMIZER_SD_H

#include <slam/icp/PoseOptimizer.h>

namespace slam {

class PoseOptimizerSD : public PoseOptimizer {
public:
  PoseOptimizerSD() : PoseOptimizer() {}
  virtual ~PoseOptimizerSD() {}

  virtual double OptimizePose(Pose2D &intiPose, Pose2D &estimatePose) override;
};

} /* namespace slam */
#endif /* pose_optimizer_sd_h */
