#ifndef POSE_OPTIMIZER_H
#define POSE_OPTIMIZER_H

#include <slam/geometry/Pose2D.h>
#include <slam/icp/CostFunction.h>

namespace slam {

class PoseOptimizer {
public:
  PoseOptimizer() {}
  ~PoseOptimizer() {}

  virtual void Initialize() = 0;
  virtual double OptimizePose(const Pose2D &initPose, Pose2D &estimatePose) = 0;

public:
  void SetCostFunction(CostFunction *f);
  void SetPoints(const std::vector<const ScanPoint2D *> &cur,
                 const std::vector<const ScanPoint2D *> &ref);
  void GetMatchRate(double &val);

protected:
  int m_repeat_num = 0;
  double m_error_sum = 0;

  CostFunction *m_cost_func_ptr = nullptr;

public:
  friend class PoseOptimizerTestFriend;
};

inline void PoseOptimizer::SetCostFunction(CostFunction *f) {
  m_cost_func_ptr = f;
}
inline void
PoseOptimizer::SetPoints(const std::vector<const ScanPoint2D *> &cur,
                         const std::vector<const ScanPoint2D *> &ref) {
  m_cost_func_ptr->SetPoints(cur, ref);
}
inline void PoseOptimizer::GetMatchRate(double &val) {
  val = m_cost_func_ptr->GetMatchRate();
}

} /* namespace slam */
#endif /* POSE_OPTIMIZER_H */
