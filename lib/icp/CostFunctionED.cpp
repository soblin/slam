#include <cmath>
#include <slam/icp/CostFunctionED.h>

namespace slam {

double CostFunctionED::CalcValue(double tx, double ty, double th /*rad*/) {
  double error = 0;
  int matched_num = 0;
  int total_num = 0;

  for (unsigned i = 0; i < m_cur_points.size(); ++i) {
    const ScanPoint2D *cur_point_iter = m_cur_points[i];
    const ScanPoint2D *ref_point_iter = m_ref_points[i];

    double cx = cur_point_iter->x();
    double cy = cur_point_iter->y();
    double x = std::cos(th) * cx - std::sin(th) * cy + tx;
    double y = std::sin(th) * cx + std::cos(th) * cy + ty;

    double dx = x - ref_point_iter->x();
    double dy = y - ref_point_iter->y();
    double dist = dx * dx + dy * dy;

    if (dist <= m_val_thresh * m_val_thresh)
      matched_num++;

    error += dist;

    total_num++;
  }

  error = (total_num > 0) ? error / total_num : HUGE_VAL;

  m_match_rate = 1.0 * matched_num / total_num;

  error *= 100;

  return error;
}

} /* namespace slam */
