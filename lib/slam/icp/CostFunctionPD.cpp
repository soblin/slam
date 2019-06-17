#include <slam/geometry/ScanPoint2D.h>
#include <slam/icp/CostFunctionPD.h>
#include <slam/parameters.h>

namespace slam {

double CostFunctionPD::CalcValue(double tx, double ty, double th /*rad*/) {
  double error = 0;
  int total_num = 0;
  int valid_num = 0;

  for (unsigned i = 0; i < m_cur_points.size(); ++i) {
    const auto cur_point = m_cur_points[i];
    const auto ref_point = m_ref_points[i];

    if (ref_point->type() != ScanPoint2D::PointType::LINE)
      continue;

    // local frame
    double cx = cur_point->x();
    double cy = cur_point->y();
    // to global frame
    double Cos = std::cos(th), Sin = std::sin(th);
    double x = cx * Cos - cy * Sin + tx;
    double y = cx * Sin + cy * Cos + ty;

    double dx = x - ref_point->x();
    double dy = y - ref_point->y();
    double perpendic_dist = dx * ref_point->nx() + dy * ref_point->ny();

    double err = perpendic_dist * perpendic_dist;

    if (std::sqrt(err) <= param::CostFunction_VAL_THRESH)
      valid_num++;

    error += err;

    total_num++;
  }

  error = (total_num > 0) ? error / total_num : HUGE_VAL;
  m_match_rate = 1.0 * valid_num / total_num;

  return error;
}

} // namespace slam
