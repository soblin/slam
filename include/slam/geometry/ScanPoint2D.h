#ifndef SCANPOINT2D_H
#define SCANPOINT2D_H

namespace slam {

class ScanPoint2D {
public:
  enum PointType { UNKNOWN, LINE, CORNER, ISOLATE };

public:
  ScanPoint2D() : m_x(0), m_y(0) { init(); }
  ScanPoint2D(double x, double y) : m_x(x), m_y(y) { init(); }
  ScanPoint2D(double x, double y, double nx, double ny, PointType type)
      : m_x(x), m_y(y), m_nx(nx), m_ny(ny), m_travel(0), m_type(type) {}

  void init();
  void SetData(double x, double y);
  void SetXY(double x, double y);
  void CalcXY(double range, double rad);
  void SetTravel(double travel);
  void SetType(PointType type);
  void SetNormal(double nx, double ny);

public:
  inline double x() const { return m_x; }
  inline double y() const { return m_y; }
  inline double nx() const { return m_nx; }
  inline double ny() const { return m_ny; }
  inline double travel() const { return m_travel; }
  inline PointType type() const { return m_type; }

private:
  double m_x, m_y;
  double m_nx, m_ny;
  double m_travel;
  PointType m_type;
};

struct Vector2D {
  double x;
  double y;
};

} // namespace slam

#endif
