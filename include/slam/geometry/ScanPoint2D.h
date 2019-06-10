#ifndef SCANPOINT2D_H
#define SCANPOINT2D_H

namespace slam {

class ScanPoint2D {
private:
  int m_id;
  double m_x, m_y;

public:
  inline int id() const { return m_id; }
  inline double x() const { return m_x; }
  inline double y() const { return m_y; }
  inline double &x() { return m_x; }
  inline double &y() { return m_y; }

public:
  ScanPoint2D();
  ScanPoint2D(int id, double x, double y);
  void init();
  void SetData(int id, double x, double y);
  void SetXY(double x, double y);
  void CalcXY(double range, double rad);
  void SetId(int i);
};

} // namespace slam

#endif
