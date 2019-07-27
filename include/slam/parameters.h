#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <cmath>

namespace slam {
namespace param {

// SensorDataReader.cpp, line33
static constexpr double Scan2D_MAX_SCAN_RANGE = 6.0;
static constexpr double Scan2D_MIN_SCAN_RANGE = 0.1;

// PointCloudMapBS.cpp line13
static constexpr double PointCloudMapBS_SKIP = 5.0;

// PointCloudMap.h, ctor
static constexpr double PointCloudMap_MAX_POINT_NUM = 1000000;

// NNGridTable.cpp, line75
static constexpr int PointCloudMapGT_CELL_POINT_NUM_THRESH1 = 1;
static constexpr int PointCloudMapGT_CELL_POINT_NUM_THRESH2 = 5;

// SensorDataReader.h, ctor
static constexpr double SensorDataReader_ANGLE_OFFSET = 180;

// MapDrawer.cpp, line85, 94
static constexpr double MapDrawer_STEP_POINT = 1.0;
static constexpr double MapDrawer_STEP_POSE = 10;

// MapDrawer.cpp, line101
static constexpr double MapDrawer_DD = 0.4;

// SlamLauncher.cpp, 26
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
static constexpr double ScanPointAnalyser_FPDMIN = 0.06;
static constexpr double ScanPointAnalyser_FPDMAX = 1.0;
static constexpr int ScanPointAnalyser_CORNER_DEG_THRESH = 45;
static constexpr int ScanPointAnalyser_INVALID_DEG = -1;
static const double ScanPointAnalyer_CORNER_COS_THRESH = std::cos(M_PI / 4);

// NNGridTable.h, ctor
static constexpr double NNGridTable_CELL_SIZE = 0.05;
static constexpr double NNGridTable_DOMAIN_SIZE = 40;

// NNGridTable.cpp, line69
static constexpr double NNGridTable_MIN_DIST_THRESH = 0.2;

static constexpr double PoseOptimizer_SEARCH_RANGE = 2.0;

} /* namespace param */
} /* namespace slam */
#endif /* parameters_h */
