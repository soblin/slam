#include <math.h>
#include <slam/geometry/Pose2D.h>

static double normalize(double x) {
  if (x > M_PI)
    return (x - 2 * M_PI);
  else if (x <= -M_PI)
    return (x + 2 * M_PI);
  return x;
}

namespace slam {

Pose2D::Pose2D() : m_tx(0), m_ty(0), m_th(0) {
  m_Rmat[0][0] = m_Rmat[1][1] = 1.0;
  m_Rmat[0][1] = m_Rmat[1][0] = 0.0;
}

Pose2D::Pose2D(double tx, double ty, double th) : m_tx(tx), m_ty(ty) {
  double rad = th / 180.0 * M_PI;
  m_th = normalize(rad);
  CalcRmat();
}

void Pose2D::Reset() {
  m_tx = m_ty = m_th = 0;
  CalcRmat();
}

void Pose2D::SetVal(double x, double y, double th) {
  m_tx = x;
  m_ty = y;
  m_th = normalize(th);
  CalcRmat();
}

void Pose2D::CalcRmat() {
  /*
    +cos -sin
    +sin +cos
   */
  double Cos = cos(m_th);
  double Sin = sin(m_th);
  m_Rmat[0][0] = m_Rmat[1][1] = Cos;
  m_Rmat[0][1] = -Sin;
  m_Rmat[1][0] = Sin;
}

void Pose2D::SetTranslation(double tx, double ty) {
  m_tx = tx;
  m_ty = ty;
}

void Pose2D::SetAngle(double th) {
  m_th = normalize(th);
  CalcRmat();
}

// convert local point src to global frame(dst)
ScanPoint2D Pose2D::ToRelativePoint(const ScanPoint2D &global) const {
  // global = this + this->Rmat * return
  // return = Rmat^{-1} * (global - this)
  double dx = global.x() - m_tx;
  double dy = global.y() - m_ty;
  // multiply R(-\theta)
  double x = m_Rmat[0][0] * dx + m_Rmat[1][0] * dy;
  double y = m_Rmat[0][1] * dx + m_Rmat[1][1] * dy;
  return ScanPoint2D(x, y);
}

ScanPoint2D Pose2D::ToGlobalPoint(const ScanPoint2D &local) const {
  double x = m_Rmat[0][0] * local.x() + m_Rmat[0][1] * local.y() + m_tx;
  double y = m_Rmat[1][0] * local.x() + m_Rmat[1][1] * local.y() + m_ty;
  return ScanPoint2D(x, y);
}

void Pose2D::ToGlobalPoint(const ScanPoint2D &local,
                           ScanPoint2D &global) const {
  global.x() = m_Rmat[0][0] * local.x() + m_Rmat[0][1] * local.y() + m_tx;
  global.y() = m_Rmat[1][0] * local.x() + m_Rmat[1][1] * local.y() + m_ty;
}

void Pose2D::CalcRelativePose(const Pose2D &global, const Pose2D &base,
                              Pose2D &relative) {
  double dx = global.tx() - base.tx();
  double dy = global.ty() - base.ty();

  relative.tx() = dx * base.R00() + dy * base.R10();
  relative.ty() = dx * base.R01() + dy * base.R11();
  double th = normalize(global.th() - base.th());
  relative.SetAngle(th);
}

void Pose2D::CalcGlobalPose(const Pose2D &relative, const Pose2D &base,
                            Pose2D &global) {
  double tx = relative.tx();
  double ty = relative.ty();
  global.tx() = tx * base.R00() + ty * base.R01() + base.tx();
  global.ty() = tx * base.R10() + ty * base.R11() + base.ty();
  double th = normalize(base.th() + relative.th());
  global.SetAngle(th);
}
} // namespace slam
