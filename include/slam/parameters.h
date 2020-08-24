#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <cmath>

namespace slam {
namespace param {

// SensorDataReader.h, line27
static constexpr double Scan2D_MAX_SCAN_RANGE = 6.0;
// SensorDataReader.h, line28
static constexpr double Scan2D_MIN_SCAN_RANGE = 0.1;
// SensorDataReader.cpp, line11
static constexpr double SensorDataReader_ANGLE_OFFSET = 180;

// MapDrawer.cpp, line20
static constexpr double MapDrawer_STEP_POINT = 1.0;
// MapDrawer.cpp, line21
static constexpr double MapDrawer_STEP_POSE = 10;
// MapDrawer.cpp, line22
static constexpr double MapDrawer_DD = 0.4;

// SlamLauncher.cpp, line26
static constexpr int SlamLauncher_PLOT_SKIP = 10;
// SlamLauncher.cpp, line27
static constexpr int SlamLauncher_SLEEP_TIME = 100000;

// geometry
// PointCloudMapBS.cpp line23
static constexpr double PointCloudMapBS_SKIP = 5.0;
// PointCloudMapBS.cpp, line8
// PointCloudMapGT.cpp, line7
static constexpr double PointCloudMap_MAX_POINT_NUM = 1000000;
// PointCloudMapLP.cpp, line45
static constexpr int PointCloudMapLP_ACC_DIST_THRESH = 10;

// SlamFrontEnd.cpp, line31
static constexpr int PointCloudMapGT_CELL_POINT_NUM_COUNTER_THRESH = 10;

// NNGridTable.cpp, line30
// SlamFrontEnd.cpp. line32, 34
static constexpr int PointCloudMapGT_CELL_POINT_NUM_THRESH1 = 1;
static constexpr int PointCloudMapGT_CELL_POINT_NUM_THRESH2 = 5;
// NNGridTable.cpp, line8
static constexpr double NNGridTable_CELL_SIZE = 0.05;
// NNGridTable.cpp, line9
static constexpr double NNGridTable_DOMAIN_SIZE = 40;
// NNGridTable.cpp, line109
static constexpr double NNGridTable_MIN_DIST_THRESH = 0.2;

// DataAssociatorLS.cpp, line24
static constexpr double DataAssociatorLS_DIST_THRESH = 0.2;

// CostFunctionED.cpp, line47
// CostFunctionPD.cpp, line37
static constexpr double CostFunction_VAL_THRESH = 0.2;

// PoseOptimizerSD.cpp, line9
// PoseOptimzierSL.cpp, line11
static constexpr double PoseOptimizer_VAL_DIFF_THRESH = 0.0001; // 0.000001
// PoseOptimizerSD.cpp, line10
static constexpr double PoseOptimizerSL_ITERATION = 50; // 100
// PoseOptimizerSL.cpp, line13
static constexpr double PoseOptimizerSD_ITERATION = 50; // 100
// PoseOptimizerSD.cpp, line11
// PoseOptimizerSL.cpp, line14
static constexpr double PoseOptimizer_TICK_DIST = 0.0001; // 0.00001
// PoseOptimizerSD.cpp, line12
// PoseOptimizerSL.cpp, line15
static constexpr double PoseOptimizer_TICK_THETA = 0.001; // 0.001
// PoseOptimizerSD.cpp, line13
// PoseOptimizerSL.cpp, line16
static constexpr double PoseOptimizer_ERROR_THRESH = 50; // 100
// PoseOptimizerSD.cpp, line14
static constexpr double PoseOptimizer_DESCENT_COEFF = 0.0001; // 0.00001
// PoseOptimizerSL.cpp, line17
static constexpr double PoseOptimizer_SEARCH_RANGE = 2.0;

// PoseEstimatorICP.cpp, line16
static constexpr double PoseEstimatorICP_VAL_DIFF_THRESH = 0.0001; // 0.000001
// PoseEstimatorICP.cpp, line18
static constexpr int PoseEstimatorICP_ITERATION = 50; // 100

// static constexpr double PoseEstimatorICP_VAL_THRESH = 0.2;
// ScanMatcher2D.cpp, line23
static constexpr double ScanMatcher2D_SCORE_THRESH = 1.0;
// ScanMatcher2D.cpp, line24
static constexpr double ScanMatcher2D_USED_NUM_THRESH = 50;

// ScanPointResampler.cpp, line32
// ScanPointResampler.cpp, line48
static constexpr double ScanPointResampler_DIST_INTERVAL = 0.05;
// ScanPointResampler.cpp, line49
static constexpr double ScanPointResampler_DIST_INTERPOLATE_THRESH = 0.25;

// ScanPointAnalyser.cpp, line34
static constexpr double ScanPointAnalyser_FPDMIN = 0.06;
// ScanPointAnalyser.cpp, line35
static constexpr double ScanPointAnalyser_FPDMAX = 1.0;
// ScanPointAnalyser.cpp, line87
static constexpr int ScanPointAnalyser_CORNER_DEG_THRESH = 45;
// ScanPointAnalyser.cpp, line88
static constexpr int ScanPointAnalyser_INVALID_DEG = -1;

// ScanPointResampler.cpp, line89
static const double ScanPointAnalyser_COS_THRESH = std::cos(M_PI / 4);

// CovarianceCalculator.cpp, line7
static const double CovarianceCalculator_TICK_DIST = 0.00001;
// CovarianceCalculator.cpp, line8
static const double CovarianceCalculator_TICK_THETA = 0.00001;
// CovarianceCalculator.cpp, line9
static const double CovarianceCalculator_ALPHA1 = 1.0;
// CovarianceCalculator.cpp, line10
static const double CovarianceCalculator_ALPHA2 = 5.0;
// CovarianceCalculator.cpp, line98
static const double CovarianceCalculator_ICP_COV_SCALE1 = 0.1;
// CovarianceCalculator.cpp, line117
static const double CovarianceCalculator_VEL_THRESH1 = 0.02;
// CovarianceCalculator.cpp, line118
static const double CovarianceCalculator_OMEGA_THRESH1 = 0.05;
// CovarianceCalculator.cpp, line131
static const double CovarianceCalculator_MCOV_COEFF_X = 0.001;
// CovarianceCalculator.cpp, line132
static const double CovarianceCalculator_MCOV_COEFF_Y = 0.005;
// CovarianceCalculator.cpp, line133
static const double CovarianceCalculator_MCOV_COEFF_TH = 0.05;
// CovarianceCalculator.cpp, line138
static const double CovarianceCalculator_ICP_COV_SCALE2 = 1.0;
// CovarianceCalculator.cpp, line146
static const double CovarianceCalculator_VEL_THRESH2 = 0.001;
// CovarianceCalculator.cpp, line147
static const double CovarianceCalculator_OMEGA_THRESH2 = 0.01;

// PoseGraph.cpp, line7
static const int PoseGraph_POOL_SIZE = 100000;

// SlamBackEnd.cpp, line20
static const int SlamBackEnd_P2O_N = 5;

// LoopDetectorSS.cpp, line7
static const double LoopDetectorSS_RADIUS = 4.0;
// LoopDetectorSS.cpp, line8
static const double LoopDetectorSS_ACC_DIST_THRESH = 10;
// LoopDetectorSS.cpp, line9
static const double LoopDetectorSS_SCORE_THRESH = 0.2;
// LoopDetectorSS.cpp, line95
static const int LoopDetectorSS_USED_NUM_MIN = 50;
// LoopDetectorSS.cpp, line97
static const double LoopDetectorSS_RANGE_T = 1.0;
// LoopDetectorSS.cpp, line98
static constexpr double LoopDetectorSS_RANGE_A = M_PI / 4;
// LoopDetectorSS.cpp, line99
static const double LoopDetectorSS_DD = 0.2;
// LoopDetectorSS.cpp, line100
static constexpr double LoopDetectorSS_DA = 2 * M_PI / 180;
// LoopDetectorSS.cpp, line101
static const double LoopDetectorSS_MATCH_THRESH = 0.8;
// LoopDetectorSS.cpp, line102
static const int LoopDetectorSS_MATCH_RATIO_THRESH = 0.9;

} /* namespace param */
} /* namespace slam */
#endif /* parameters_h */
