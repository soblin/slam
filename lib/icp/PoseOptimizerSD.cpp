#include <slam/icp/PoseOptimizerSD.h>

#include <cmath>

namespace slam {

double PoseOptimizerSD::OptimizePose(Pose2D &initPose, Pose2D &estimatePose) {
  double tx = initPose.tx();
  double ty = initPose.ty();
  double th = initPose.th();

  double tx_min = tx, ty_min = ty,
         th_min = th;         // the solution that minimizes the cost function
  double eval_min = HUGE_VAL; // the minimum value of cost function
  double eval_old = eval_min; // previous value of cost function

  double eval = m_cost_func->CalcValue(tx, ty, th);
  int number_of_iteration = 0;
  double kk = 0.00001;

  while (std::abs(eval_old - eval) > m_val_diff_thresh) {
    number_of_iteration++;
    eval_old = eval;

    // partial differentiatin
    double dE1dtx = (m_cost_func->CalcValue(tx + m_dd, ty, th) - eval) / m_dd;
    double dE1dty = (m_cost_func->CalcValue(tx, ty + m_dd, ty) - eval) / m_dd;
    double dE1dth = (m_cost_func->CalcValue(tx, ty, th + m_dth) - eval) / m_dth;

    // steepest descent
    double dx = -kk * dE1dtx;
    double dy = -kk * dE1dty;
    double dth = -kk * dE1dth;

    tx += dx;
    ty += dy;
    th += dth;

    // check the updated cost
    eval = m_cost_func->CalcValue(tx, ty, th);

    if (eval < eval_min) {
      eval_min = eval;
      tx_min = tx;
      ty_min = ty;
      th_min = th;
    }
  }

  m_repeat_num++;
  if (m_repeat_num > 0 && eval_min < 100)
    m_error_sum += eval_min;

  estimatePose.SetVal(tx_min, ty_min, th_min);

  return eval_min;
}

} /* namespace slam */
