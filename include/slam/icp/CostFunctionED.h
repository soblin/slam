#ifndef COST_FUNCTION_ED_H
#define COST_FUNCTION_ED_H

#include <slam/icp/CostFunction.h>

namespace slam {

class CostFunctionED : public CostFunction {
public:
  CostFunctionED() : CostFunction() {}
  virtual ~CostFunctionED() {}

  virtual double CalcValue(double tx, double ty, double th /*rad*/) override;
  double CalcValue(double tx, double ty, double th, double val_threseh);

  friend class CostFunctionEDTestFriend;
};

} /* namespace slam */
#endif /* cost_function_ed_h */
