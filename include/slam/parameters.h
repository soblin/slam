#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <map>
#include <string>

namespace slam{

static const double Scan2D_MAX_SCAN_RANGE = 6.0;
static const double Scan2D_MIN_SCAN_RANGE = 0.1;
static const double PointCloudMapBS_SKIP = 5.0;
static const double SensorDataReader_ANGLE_OFFSET = 180;
static const double MapDrawer_STEP_POINT = 1.0;
static const double MapDrawer_STEP_POSE = 10;
static const double MapDrawer_DD= 0.4;
static const double SlamLauncher_SLEEP_TIM = 100000;
static const double DataAssociatorLS_DIST_THRESH = 0.2;
static const double CostFunction_VAL_THRESH = 0.2;
static const double PoseOptimizer_VAL_DIFF_THRESH = 0.000001;
static const double PoseOptimizer_TickDist = 0.00001;
static const double PoseOptimizer_TickTheta = 0.0001;
static const double PoseOptimizer_DescentCoeff = 0.00001;
static const double PoseEstimatorICP_VAL_DIFF_THRESH = 0.000001;
static const double PoseEstimatorICP_VAL_THRESH = 0.2;

} /* namespace slam */
#endif /* parameters_h */
