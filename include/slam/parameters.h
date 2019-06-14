#ifndef PARAMETERS_H
#define PARAMETERS_H

namespace slam{
namespace param{

static constexpr double Scan2D_MAX_SCAN_RANGE = 6.0;
static constexpr double Scan2D_MIN_SCAN_RANGE = 0.1;
static constexpr double PointCloudMapBS_SKIP = 5.0;
static constexpr double PointCloudMap_MAX_POINT_NUM = 1000000;
static constexpr double SensorDataReader_ANGLE_OFFSET = 180;
static constexpr double MapDrawer_STEP_POINT = 1.0;
static constexpr double MapDrawer_STEP_POSE = 10;
static constexpr double MapDrawer_DD= 0.4;
static constexpr int SlamLauncher_SLEEP_TIME = 100000;
static constexpr int SlamLauncher_PLOT_SKIP = 10;
static constexpr double DataAssociatorLS_DIST_THRESH = 0.2;
static constexpr double CostFunction_VAL_THRESH = 0.2;
static constexpr double PoseOptimizer_VAL_DIFF_THRESH = 0.000001;
static constexpr double PoseOptimizer_TickDist = 0.00001;
static constexpr double PoseOptimizer_TickTheta = 0.0001;
static constexpr double PoseOptimizer_ERROR_THRESH = 100;
static constexpr double PoseOptimizer_DescentCoeff = 0.00001;
static constexpr double PoseEstimatorICP_VAL_DIFF_THRESH = 0.000001;
static constexpr double PoseEstimatorICP_VAL_THRESH = 0.2;
static constexpr double ScanMatcher2D_SCORE_THRESH = 1.0;
static constexpr double ScanMatche2D_USE_DNUM_THRESH = 50;
static constexpr double ScanPointResampler_DIST_INTERVAL = 0.05;
static constexpr double ScanPointResampler_DIST_INTERPOLATE_THRESH = 0.25;

} /* namespace param */
} /* namespace slam */
#endif /* parameters_h */
