#include <slam/geometry/Scan2D.h>

namespace slam {
Scan2D::Scan2D() : m_id(-1) {}

Scan2D::~Scan2D() { m_scaned_points.reserve(0); }

void Scan2D::SetId(int id) { m_id = id; }

void Scan2D::SetScanedPoints(const std::vector<ScanPoint2D> &scaned_points) {
  // copy ctor
  m_scaned_points = scaned_points;
}

void Scan2D::SetPose(const slam::Pose2D &pose) { m_pose = pose; }

void Scan2D::SetAngle(double rad) { m_pose.SetAngle(rad); }

double Scan2D::MAX_SCAN_RANGE = 6;
double Scan2D::MIN_SCAN_RANGE = 0.1;

} // namespace slam
