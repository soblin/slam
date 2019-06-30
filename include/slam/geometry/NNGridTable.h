#ifndef NN_GRID_TABLE_H
#define NN_GRID_TABLE_H

#include <slam/geometry/Pose2D.h>
#include <slam/geometry/ScanPoint2D.h>
#include <slam/parameters.h>
#include <vector>

namespace slam {

struct NNGridCell {
  std::vector<const ScanPoint2D *> points;

  inline void Clear() { points.clear(); }
};

class NNGridTable {
public:
  NNGridTable() {
    int width = 2 * m_table_size + 1;
    m_table.resize(width * width);
    Clear();
  }
  ~NNGridTable() { Clear(); }

  inline void Clear() {
    for (auto &cell : m_table) {
      cell.Clear();
    }
  }

  void AddPoint(const ScanPoint2D *point);
  const ScanPoint2D *FindClosestPoint(const ScanPoint2D *query,
                                      const Pose2D &basePose);
  void MakeCellPoints(std::vector<ScanPoint2D> &points);

private:
  const double m_cell_size = param::NNGridTable_CELL_SIZE;
  const double m_domain_size = param::NNGridTable_DOMAIN_SIZE;
  const int m_table_size = static_cast<int>(param::NNGridTable_DOMAIN_SIZE /
                                            param::NNGridTable_CELL_SIZE);

  std::vector<NNGridCell> m_table;

private:
  void MakeCellPointsImpl(int cell_point_num_thresh,
                          std::vector<ScanPoint2D> &points);
  const ScanPoint2D *FindClosestPointImpl(const ScanPoint2D *query,
                                          const Pose2D &basePose,
                                          double dist_thresh);
};

} // namespace slam
#endif /* NN_GRID_TABLE_H */
