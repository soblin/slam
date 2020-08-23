#ifndef SCANPOINT2D_H
#define SCANPOINT2D_H

namespace slam {

class ScanPoint2D {
public:
  enum PointType { UNKNOWN, LINE, CORNER, ISOLATE };

public:
  ScanPoint2D(){};
  ScanPoint2D(double x, double y) : m_x(x), m_y(y){};
  ScanPoint2D(double x, double y, double nx, double ny, PointType type,
              int id = -1)
      : m_x(x), m_y(y), m_nx(nx), m_ny(ny), m_acc_dist(0), m_type(type),
        m_id(id) {}

  //  void init();
  void SetData(double x, double y);
  void SetXY(double x, double y);
  void CalcXY(double range, double rad);
  void SetAccDist(double acc_dist);
  void SetType(PointType type);
  void SetNormal(double nx, double ny);
  void SetId(int id);

public:
  double x() const;
  double y() const;
  double nx() const;
  double ny() const;
  double acc_dist() const;
  PointType type() const;
  int id() const;

private:
  double m_x = 0, m_y = 0;
  double m_nx = 0, m_ny = 0;
  double m_acc_dist = 0;
  PointType m_type = UNKNOWN;
  int m_id = -1;
};

struct Vector2D {
  double x;
  double y;
};

inline double ScanPoint2D::x() const { return m_x; }
inline double ScanPoint2D::y() const { return m_y; }
inline double ScanPoint2D::nx() const { return m_nx; }
inline double ScanPoint2D::ny() const { return m_ny; }
inline double ScanPoint2D::acc_dist() const { return m_acc_dist; }
inline ScanPoint2D::PointType ScanPoint2D::type() const { return m_type; }
inline int ScanPoint2D::id() const { return m_id; }

} // namespace slam

#endif
