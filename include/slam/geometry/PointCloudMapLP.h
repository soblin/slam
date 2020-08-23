#include <slam/geometry/PointCloudMap.h>

#ifndef POITCLOUDMAP_LP_H
#define POITCLOUDMAP_LP_H

namespace slam {

class Submap {
public:
  Submap();
  Submap(double a, size_t f);

  void AddPoints(const std::vector<ScanPoint2D> &points);
  std::vector<ScanPoint2D> SubsamplePoints(int nthre);

  const double acc_dist() const;
  const size_t id_first() const;
  const size_t id_last() const;
  double &acc_dist();
  size_t &id_first();
  size_t &id_last();
  std::vector<ScanPoint2D> &points();

private:
  double m_acc_dist = 0;
  size_t m_id_first = 0;
  size_t m_id_last = 0;

  std::vector<ScanPoint2D> m_points;
};

class PointCloudMapLP : public PointCloudMap {
public:
  PointCloudMapLP() : PointCloudMap() {
    Submap submap;
    m_submaps.emplace_back(submap);
  }
  virtual ~PointCloudMapLP(){};
  virtual void Initialize() override;
  virtual void AddPose(const Pose2D &p) override;
  virtual void AddPoint(const ScanPoint2D &scan) override;
  virtual void AddPoint(ScanPoint2D &&scan) override;
  virtual void AddPoints(const std::vector<ScanPoint2D> &points) override;
  virtual void MakeGlobalMap() override;
  virtual void MakeLocalMap() override;
  virtual void RemakeMaps(const std::vector<Pose2D> &newPoses) override;
  double acc_dist();
  std::vector<Submap> &submaps();

private:
  double m_acc_dist = 0;
  std::vector<Submap> m_submaps;
};

inline const double Submap::acc_dist() const { return m_acc_dist; }
inline const size_t Submap::id_first() const { return m_id_first; }
inline const size_t Submap::id_last() const { return m_id_last; }
inline double &Submap::acc_dist() { return m_acc_dist; }
inline size_t &Submap::id_first() { return m_id_first; }
inline size_t &Submap::id_last() { return m_id_last; }
inline std::vector<ScanPoint2D> &Submap::points() { return m_points; }
inline double PointCloudMapLP::acc_dist() { return m_acc_dist; }
inline std::vector<Submap> &PointCloudMapLP::submaps() { return m_submaps; }

} // namespace slam
#endif /* POITCLOUDMAP_LP_H */
