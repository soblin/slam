#ifndef NN_GRID_TABLE_H
#define NN_GRID_TABLE_H

#include <slam/geometry/Pose2D.h>
#include <slam/geometry/ScanPoint2D.h>
#include <slam/manager/ParamServer.h>
#include <vector>

namespace slam {

struct NNGridCell {
  std::vector<const ScanPoint2D *> points;

  inline void Clear() { points.clear(); }
};

class NNGridTable {
public:
  NNGridTable() {}
  ~NNGridTable() { Clear(); }

  void Initialize();
  void AddPoint(const ScanPoint2D *point);
  void MakeCellPoints(int cell_point_num_thresh,
                      std::vector<ScanPoint2D> &points);
  const ScanPoint2D *FindClosestPoint(const ScanPoint2D *query,
                                      const Pose2D &basePose);
  const ScanPoint2D *FindClosestPoint(const ScanPoint2D *query,
                                      const Pose2D &basePose,
                                      double dist_thresh);
  void MakeCellPoints(std::vector<ScanPoint2D> &points);

public:
  inline void Clear() {
    for (auto &cell : m_table) {
      cell.Clear();
    }
  }

private:
  double m_cell_size = 0;
  double m_domain_size = 0;
  int m_table_size = 0;

  std::vector<NNGridCell> m_table;
};

} // namespace slam
#endif /* NN_GRID_TABLE_H */
