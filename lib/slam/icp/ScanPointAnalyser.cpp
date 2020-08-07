#include <cmath>
#include <slam/icp/ScanPointAnalyser.h>
#include <slam/manager/ParamServer.h>

namespace slam {

bool ScanPointAnalyser::CalcNormal(int index,
                                   const std::vector<ScanPoint2D> &points,
                                   int dir, Vector2D &ret, double fpdmin,
                                   double fpdmax) {
  const ScanPoint2D &point_of_interest = points[index];
  for (unsigned i = index + dir; i >= 0 && i < points.size(); i += dir) {
    const ScanPoint2D &point_iter = points[i];
    double dx = point_of_interest.x() - point_iter.x();
    double dy = point_of_interest.y() - point_iter.y();
    double d = std::hypot(dx, dy);

    if (d >= fpdmin && d <= fpdmax) {
      ret.x = dy / d;
      ret.y = -dx / d;
      return true;
    }

    if (d > fpdmax)
      break;
  }

  return false;
}

bool ScanPointAnalyser::CalcNormal(int index,
                                   const std::vector<ScanPoint2D> &points,
                                   int dir, Vector2D &ret) {
  static const double fpdmin = ParamServer::Get("ScanPointAnalyser_FPDMIN");
  static const double fpdmax = ParamServer::Get("ScanPointAnalyser_FPDMAX");

  return CalcNormal(index, points, dir, ret, fpdmin, fpdmax);
}

void ScanPointAnalyser::AnalysePoints(std::vector<ScanPoint2D> &points,
                                      double invalid_deg,
                                      double corner_thresh_deg,
                                      double corner_thresh_cos) {
  for (unsigned i = 0; i < points.size(); ++i) {
    auto &point = points[i];
    ScanPoint2D::PointType type;
    Vector2D nL, nR, normal;

    bool flagL = CalcNormal(i, points, -1, nL);
    bool flagR = CalcNormal(i, points, 1, nR);
    nR.x *= -1;
    nR.y *= -1;

    if (flagL) {
      if (flagR) {
        if (std::fabs(nL.x * nR.x + nL.y * nR.y) >= corner_thresh_cos)
          type = ScanPoint2D::PointType::LINE;
        else
          type = ScanPoint2D::PointType::CORNER;

        double dx = nL.x + nR.x;
        double dy = nL.y + nR.y;
        double L = std::hypot(dx, dy);
        normal.x = dx / L;
        normal.y = dy / L;
      } else {
        type = ScanPoint2D::PointType::LINE;
        normal = nL;
      }
    } else {
      if (flagR) {
        type = ScanPoint2D::PointType::LINE;
        normal = nR;
      } else {
        type = ScanPoint2D::PointType::ISOLATE;
        normal.x = invalid_deg;
        normal.y = invalid_deg;
      }
    }

    point.SetNormal(normal.x, normal.y);
    point.SetType(type);
  }
}

void ScanPointAnalyser::AnalysePoints(std::vector<ScanPoint2D> &points) {
  static const double invalid_deg =
      ParamServer::Get("ScanPointAnalyser_INVALID_DEG");
  static const double corner_deg_thresh =
      ParamServer::Get("ScanPointAnalyser_CORNER_DEG_THRESH");
  static const double corner_cos_thresh =
      ParamServer::Get("ScanPointAnalyser_COS_THRESH");

  AnalysePoints(points, invalid_deg, corner_deg_thresh, corner_cos_thresh);
}

} // namespace slam
