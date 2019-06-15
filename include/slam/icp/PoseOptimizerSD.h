#ifndef POSE_OPTIMIZER_SD_H
#define POSE_OPTIMIZER_SD_H

#include <slam/icp/PoseOptimizer.h>

namespace slam {

class PoseOptimizerSD : public PoseOptimizer {
private:
  double OptimizePoseImpl(Pose2D &initPose, Pose2D &estimatePose,
                          double val_diff_thresh, double ds, double dth,
                          double err_thresh, double descent);

public:
  PoseOptimizerSD() : PoseOptimizer() {}
  virtual ~PoseOptimizerSD() {}

  virtual double OptimizePose(Pose2D &initPose, Pose2D &estimatePose) override;

  friend class PoseOptimizerSDTestFriend;
};

} /* namespace slam */
#endif /* pose_optimizer_sd_h */
