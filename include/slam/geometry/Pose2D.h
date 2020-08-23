#ifndef POSE2D_H
#define POSE2D_H

#include <slam/geometry/ScanPoint2D.h>

namespace slam {

class Pose2D {
public:
  Pose2D();
  Pose2D(double tx, double ty, double th /*[deg]*/);

  void Reset();
  void SetVal(double x, double y, double th /*[rad]*/);
  void SetTranslation(double tx, double ty);
  void SetAngle(double th /*[rad]*/);
  void CalcRmat();

  ScanPoint2D ToRelativePoint(const ScanPoint2D &global) const;
  ScanPoint2D ToGlobalPoint(const ScanPoint2D &relative) const;
  void ToGlobalPoint(const ScanPoint2D &local, ScanPoint2D &global) const;

  static void CalcRelativePose(const Pose2D &global, const Pose2D &base,
                               Pose2D &relative);
  static void CalcGlobalPose(const Pose2D &relative, const Pose2D &base,
                             Pose2D &global);

public:
  double tx() const;
  double ty() const;
  double th() const;
  double &tx();
  double &ty();
  double R00() const;
  double R01() const;
  double R10() const;
  double R11() const;

private:
  double m_tx;
  double m_ty;
  // angle is restricted to (-M_PI, M_PI]
  double m_th; // [rad]
  double m_Rmat[2][2];
};

inline double Pose2D::tx() const { return m_tx; }
inline double Pose2D::ty() const { return m_ty; }
inline double Pose2D::th() const { return m_th; }
inline double &Pose2D::tx() { return m_tx; }
inline double &Pose2D::ty() { return m_ty; }
inline double Pose2D::R00() const { return m_Rmat[0][0]; }
inline double Pose2D::R01() const { return m_Rmat[0][1]; }
inline double Pose2D::R10() const { return m_Rmat[1][0]; }
inline double Pose2D::R11() const { return m_Rmat[1][1]; }

} // namespace slam

#endif
