#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H

#include <slam/geometry/ScanPoint2D.h>
#include <vector>

namespace slam {

class CostFunction {
protected:
  std::vector<const ScanPoint2D *> m_cur_points;
  std::vector<const ScanPoint2D *> m_ref_points;

  double val_thresh;
  double match_rate;

public:
  inline void SetValThresh(double e) { val_thresh = e; }
  inline void GetMatchRate(double &rate) { rate = match_rate; }

public:
  CostFunction() : val_thresh(0), match_rate(0) {}
  ~CostFunction() {}
};

} /* namespace slam */
#endif /* cost_function_h */
