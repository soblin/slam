#include <cmath>
#include <slam/icp/PoseOptimizerSD.h>
#include <slam/parameters.h>

namespace slam {

double PoseOptimizerSD::OptimizePoseImpl(const Pose2D &initPose,
                                         Pose2D &estimatePose,
                                         double val_diff_thresh, double ds,
                                         double dtheta, double err_thresh,
                                         double descent) {
  double tx = initPose.tx();
  double ty = initPose.ty();
  double th = initPose.th();

  double tx_min = tx, ty_min = ty,
         th_min = th;         // the solution that minimizes the cost function
  double eval_min = HUGE_VAL; // the minimum value of cost function
  double eval_old = eval_min; // previous value of cost function

  double eval = m_cost_func_ptr->CalcValue(tx, ty, th);
  int number_of_iteration = 0;

  double dd = ds;
  double dth = dtheta;

  while (std::abs(eval_old - eval) > val_diff_thresh) {
    number_of_iteration++;
    eval_old = eval;

    // partial differentiatin
    double dE1dtx = (m_cost_func_ptr->CalcValue(tx + dd, ty, th) - eval) / dd;
    double dE1dty = (m_cost_func_ptr->CalcValue(tx, ty + dd, th) - eval) / dd;
    double dE1dth = (m_cost_func_ptr->CalcValue(tx, ty, th + dth) - eval) / dth;

    // steepest descent
    double dx = -descent * dE1dtx;
    double dy = -descent * dE1dty;
    double dth = -descent * dE1dth;

    tx += dx;
    ty += dy;
    th += dth;

    // check the updated cost
    eval = m_cost_func_ptr->CalcValue(tx, ty, th);

    if (eval < eval_min) {
      eval_min = eval;
      tx_min = tx;
      ty_min = ty;
      th_min = th;
    }
  }

  m_repeat_num++;
  if (m_repeat_num > 0 && eval_min < err_thresh)
    m_error_sum += eval_min;

  estimatePose.SetVal(tx_min, ty_min, th_min);

  return eval_min;
}

double PoseOptimizerSD::OptimizePose(const Pose2D &initPose,
                                     Pose2D &estimatePose) {
  return OptimizePoseImpl(
      initPose, estimatePose, param::PoseOptimizer_VAL_DIFF_THRESH,
      param::PoseOptimizer_TickDist, param::PoseOptimizer_TickTheta,
      param::PoseOptimizer_ERROR_THRESH, param::PoseOptimizer_DescentCoeff);
}

} /* namespace slam */
