#ifndef NN_GRID_TABLE_H
#define NN_GRID_TABLE_H

namespace slam{

struct NNGridCell {
  std::vector<const ScanPoint2D *> m_points;

  inline void clear() { m_point.clear(); }
};

class NNGridTable {
 private:
  const double m_cell_size;
  const double m_domain_size;
  const int m_table_size;

  std::vector<NNGridCell> m_table;;
}

} // namespace slam
#endif /* NN_GRID_TABLE_H */
