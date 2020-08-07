#ifndef POSE_OPTIMIZER_SD_H
#define POSE_OPTIMIZER_SD_H

#include <slam/icp/PoseOptimizer.h>

namespace slam {

class PoseOptimizerSD : public PoseOptimizer {
public:
  PoseOptimizerSD() : PoseOptimizer() {}
  virtual ~PoseOptimizerSD() {}

  virtual double OptimizePose(const Pose2D &initPose,
                              Pose2D &estimatePose) override;

  virtual void Initialize() override;

  friend class PoseOptimizerSDTestFriend;

private:
  double OptimizePoseImpl(const Pose2D &initPose, Pose2D &estimatePose,
                          double val_diff_thresh, double ds, double dth,
                          double err_thresh, double descent);

private:
  double m_val_diff_thresh = 0;
  int m_max_iteration = -1;
  double m_dd = 0;
  double m_da = 0;
  double m_error_thresh = 0;
  double m_descent_coeff = 0;
};

} /* namespace slam */
#endif /* pose_optimizer_sd_h */
