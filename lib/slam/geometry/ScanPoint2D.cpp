#include <math.h>
#include <slam/geometry/ScanPoint2D.h>

namespace slam {
ScanPoint2D::ScanPoint2D() : m_id(-1), m_x(0), m_y(0) {}

ScanPoint2D::ScanPoint2D(int id, double x, double y)
    : m_id(id), m_x(x), m_y(y) {}

void ScanPoint2D::SetData(int id, double x, double y) {
  m_id = id;
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

void ScanPoint2D::SetId(int id) { m_id = id; }

} // namespace slam
