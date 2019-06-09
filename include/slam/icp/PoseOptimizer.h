#ifndef POSE_OPTIMIZER_H
#define POSE_OPTIMIZER_H

#include <slam/geometry/Pose2D.h>
#include <slam/icp/CostFunction.h>

namespace slam {

class PoseOptimizer {
protected:
  int m_repeat_num;
  double m_error_sum;

  CostFunction *m_cost_func_ptr;

public:
  inline void SetCostFunction(CostFunction *f) { m_cost_func_ptr = f; }
  inline void SetPoints(std::vector<const ScanPoint2D *> &cur,
                        std::vector<const ScanPoint2D *> &ref) {
    m_cost_func_ptr->SetPoints(cur, ref);
  }
  inline void GetMatchRate(double &val) {
    val = m_cost_func_ptr->GetMatchRate();
  }

public:
  PoseOptimizer() : m_cost_func_ptr(nullptr) {
    m_repeat_num = 0;
    m_error_sum = 0;
  }
  ~PoseOptimizer() {}

  virtual double OptimizePose(Pose2D &initPose, Pose2D &estimatePose) = 0;

  friend class PoseOptimizerTestFriend;
};

} /* namespace slam */
#endif /* pose_optimizer_h */
