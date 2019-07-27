#ifndef COST_FUNCTION_ED_H
#define COST_FUNCTION_ED_H

#include <slam/icp/CostFunction.h>

namespace slam {

class CostFunctionED : public CostFunction {
private:
  double CalcValueImpl(double tx, double ty, double th, double val_threseh);

public:
  CostFunctionED() : CostFunction() {}
  virtual ~CostFunctionED() {}

  virtual double CalcValue(double tx, double ty, double th /*rad*/) override;

  virtual void Initialize() override;

  friend class CostFunctionEDTestFriend;
};

} /* namespace slam */
#endif /* cost_function_ed_h */
