#include <math.h>
#include <slam/geometry/ScanPoint2D.h>

namespace slam {

void ScanPoint2D::init() {
  m_travel = 0;
  m_type = UNKNOWN;
  m_nx = 0;
  m_ny = 0;
}

void ScanPoint2D::SetData(double x, double y) {
  init();
  m_x = x;
  m_y = y;
}

void ScanPoint2D::SetXY(double x, double y) {
  m_x = x;
  m_y = y;
}

void ScanPoint2D::CalcXY(double range, double rad) {
  m_x = range * cos(rad);
  m_y = range * sin(rad);
}

void ScanPoint2D::SetTravel(double travel) { m_travel = travel; }

void ScanPoint2D::SetType(ScanPoint2D::PointType type) { m_type = type; }

void ScanPoint2D::SetNormal(double nx, double ny) {
  m_nx = nx;
  m_ny = ny;
}

} // namespace slam
