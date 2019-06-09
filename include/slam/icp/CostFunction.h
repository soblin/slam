#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H

#include <slam/geometry/ScanPoint2D.h>
#include <vector>

namespace slam {

class CostFunction {
protected:
  // the set of points matched with DataAssiciator
  std::vector<const ScanPoint2D *> m_cur_points;
  std::vector<const ScanPoint2D *> m_ref_points;

  double m_match_rate;

public:
  inline double GetMatchRate() const { return m_match_rate; }

public:
  CostFunction() {}
  ~CostFunction() {}

  virtual double CalcValue(double tx, double ty, double th) = 0;

  void SetPoints(std::vector<const ScanPoint2D *> &cur,
                 std::vector<const ScanPoint2D *> &ref) {
    m_cur_points = cur;
    m_ref_points = ref;
  }
};

} /* namespace slam */
#endif /* cost_function_h */
