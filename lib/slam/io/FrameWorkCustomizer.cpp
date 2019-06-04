#include <slam/io/FrameWorkCustomizer.h>

namespace slam {

void FrameWorkCustomizer::MakeFrameWork() {
  m_scan_matcher.SetEstimatorICP(&m_pose_estimator);
}

} /* namespace slam */
