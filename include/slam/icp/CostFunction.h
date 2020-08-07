#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H

#include <slam/geometry/ScanPoint2D.h>
#include <vector>

namespace slam {

class CostFunction {
public:
  CostFunction() {}
  ~CostFunction() {}

  virtual void Initialize() = 0;
  virtual double CalcValue(double tx, double ty, double th) = 0;

public:
  inline double GetMatchRate() const { return m_match_rate; }
  inline void SetPoints(const std::vector<const ScanPoint2D *> &cur,
                        const std::vector<const ScanPoint2D *> &ref) {
    m_cur_points = cur;
    m_ref_points = ref;
  }

protected:
  // the set of points matched with DataAssiciator
  std::vector<const ScanPoint2D *> m_cur_points;
  std::vector<const ScanPoint2D *> m_ref_points;

  double m_match_rate = -1;
};

} /* namespace slam */
#endif /* COST_FUNCTION_H */
