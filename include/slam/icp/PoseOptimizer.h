#ifndef POSE_OPTIMIZER_H
#define POSE_OPTIMIZER_H

#include <slam/geometry/Pose2D.h>
#include <slam/icp/CostFunction.h>

namespace slam {

class PoseOptimizer {
protected:
  int m_repeat_num;
  double m_error_sum;

  double m_val_diff_thresh;
  double m_dd;  // differentiation ticks for translation
  double m_dth; // differentiaion ticks for rotation
  CostFunction *m_cost_func_ptr;

public:
  inline void SetCostFunction(CostFunction *f) { m_cost_func_ptr = f; }
  inline void SetValThresh(double l) { m_cost_func_ptr->SetValThresh(l); }
  inline void SetPoints(std::vector<const ScanPoint2D *> &cur,
                        std::vector<const ScanPoint2D *> &ref) {
    m_cost_func_ptr->SetPoints(cur, ref);
  }
  inline void SetValDiffThresh(double thresh) { m_val_diff_thresh = thresh; }
  inline void GetMatchRate(double &val) {
    double temp;
    m_cost_func_ptr->GetMatchRate(temp);
    val = temp;
  }
  inline void SetDiff(double d, double a) {
    m_dd = d;
    m_dth = a;
  }

public:
  PoseOptimizer()
      : m_val_diff_thresh(0.000001), m_dd(0.00001), m_dth(0.00001),
        m_cost_func_ptr(nullptr) {
    m_repeat_num = 0;
    m_error_sum = 0;
  }
  ~PoseOptimizer() {}

  virtual double OptimizePose(Pose2D &initPose, Pose2D &estimatePose) = 0;

  friend class PoseOptimizerTestFriend;
};

} /* namespace slam */
#endif /* pose_optimizer_h */
