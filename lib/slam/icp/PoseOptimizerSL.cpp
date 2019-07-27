#include <numeric>
#include <slam/icp/PoseOptimizerSL.h>
#include <slam/manager/ParamServer.h>

#include <boost/math/tools/minima.hpp>

namespace slam {

double PoseOptimizerSL::OptimizePose(const Pose2D &initPose,
                                     Pose2D &estimatePose) {
  double th = initPose.th();
  double tx = initPose.tx();
  double ty = initPose.ty();

  double tx_min = tx, ty_min = ty, th_min = th;
  double min_cost = HUGE_VAL;
  double prev_cost = min_cost;
  Pose2D pose, direction;

  double cost = m_cost_func_ptr->CalcValue(tx, ty, th);
  int number_of_iteration = 0;

  static const double val_diff_thresh =
      ParamServer::Get("PoseOptimizer_VAL_DIFF_THRESH");
  static const int max_iteration = ParamServer::Get("PoseOptimizerSL_ITERTION");

  while (std::fabs(prev_cost - cost) > val_diff_thresh &&
         number_of_iteration < max_iteration) {
    number_of_iteration++;
    prev_cost = cost;

    static const double dd = ParamServer::Get("PoseOptimizer_TickDist");
    static const double da = ParamServer::Get("PoseOptimizer_TickTheta");
    double dx = (m_cost_func_ptr->CalcValue(tx + dd, ty, th) - cost) / dd;
    double dy = (m_cost_func_ptr->CalcValue(tx, ty + dd, th) - cost) / dd;
    double dth = (m_cost_func_ptr->CalcValue(tx, ty, th + da) - cost) / da;

    tx += dx;
    ty += dy;
    th += dth;

    // linear search
    // start linear search from here
    pose.SetVal(tx, ty, th);
    // search direction
    direction.SetVal(dx, dy, dth);

    LineSearch(cost, pose, direction);
    tx = pose.tx();
    ty = pose.ty();
    th = pose.th();

    cost = m_cost_func_ptr->CalcValue(tx, th, th);

    if (cost < min_cost) {
      min_cost = cost;
      tx_min = tx;
      ty_min = ty;
      th_min = th;
    }
  }

  m_repeat_num++;

  static double error_thresh = ParamServer::Get("PoseOptimizer_ERROR_THRESH");
  if (m_repeat_num > 0 && min_cost < error_thresh)
    m_error_sum += min_cost;

  estimatePose.SetVal(tx_min, ty_min, th_min);

  return min_cost;
}

double PoseOptimizerSL::LineSearch(double ev0, Pose2D &pose,
                                   const Pose2D &direction) {
  static const double search_range =
      ParamServer::Get("PoseOptimizer_SEARCH_RANGE");

  int bits = std::numeric_limits<double>::digits;
  boost::uintmax_t maxIter = 40;
  std::pair<double, double> result = boost::math::tools::brent_find_minima(
      [this, &pose, &direction](double step) {
        return ObjFunc(step, pose, direction);
      },
      -search_range, search_range, bits, maxIter);

  double step = result.first; // the step width
  double min_val = result.second;

  double tx = pose.tx() + step * direction.tx();
  double ty = pose.ty() + step * direction.ty();
  double th = pose.th() + step * direction.th();

  pose.SetVal(tx, ty, th);

  return min_val;
}

double PoseOptimizerSL::ObjFunc(double step, Pose2D &pose,
                                const Pose2D &direction) {
  double tx = pose.tx() + step * direction.tx();
  double ty = pose.ty() + step * direction.ty();
  double th = pose.th() + step * direction.th();
  if (th > M_PI)
    th -= 2 * M_PI;
  else if (th <= -M_PI)
    th += 2 * M_PI;

  return m_cost_func_ptr->CalcValue(tx, ty, th);
}

} // namespace slam
