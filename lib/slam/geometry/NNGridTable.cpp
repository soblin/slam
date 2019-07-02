#include <cmath>
#include <slam/geometry/NNGridTable.h>
#include <slam/manager/ParamServer.h>

namespace slam {

void NNGridTable::AddPoint(const ScanPoint2D *point) {
  int x_index = static_cast<int>(point->x() / m_cell_size) + m_table_size;
  if (x_index < 0 || x_index > 2 * m_table_size)
    return;

  int y_index = static_cast<int>(point->y() / m_cell_size) + m_table_size;
  if (y_index < 0 || y_index > 2 * m_table_size)
    return;

  int index = static_cast<int>(y_index * (2 * m_table_size + 1) + x_index);

  m_table[index].points.emplace_back(point);
}

const ScanPoint2D *NNGridTable::FindClosestPointImpl(const ScanPoint2D *query,
                                                     const Pose2D &basePose,
                                                     double dist_thresh) {
  ScanPoint2D query_glob;
  basePose.ToGlobalPoint(*query, query_glob);

  int x_index = static_cast<int>(query_glob.x() / m_cell_size) + m_table_size;
  if (x_index < 0 || x_index > 2 * m_table_size)
    return nullptr;

  int y_index = static_cast<int>(query_glob.y() / m_cell_size) + m_table_size;
  if (y_index < 0 || y_index > 2 * m_table_size)
    return nullptr;

  double min_dist = HUGE_VAL;
  const ScanPoint2D *min_point = nullptr;

  const int R = static_cast<int>(dist_thresh / m_cell_size);

  for (int i = -R; i <= R; ++i) {
    int cur_y_ind = y_index + i;
    if (cur_y_ind < 0 || cur_y_ind > 2 * m_table_size)
      continue;
    for (int j = -R; j <= R; ++j) {
      int cur_x_ind = x_index + j;
      if (cur_x_ind < 0 || cur_x_ind > 2 * m_table_size)
        continue;

      int index = cur_y_ind * (2 * m_table_size + 1) + cur_x_ind;
      auto &cell = m_table[index];
      auto &points = cell.points;
      for (const auto iter : points) {
        double dist =
            std::hypot(iter->x() - query_glob.x(), iter->y() - query_glob.y());

        if (dist < min_dist && dist < dist_thresh) {
          min_dist = dist;
          min_point = iter;
        }
      }
    }
  }

  return min_point;
}

const ScanPoint2D *NNGridTable::FindClosestPoint(const ScanPoint2D *query,
                                                 const Pose2D &basePose) {
  double min_dist_thresh = ParamServer::Get("NNGridTable_MIN_DIST_THRESH");

  return FindClosestPointImpl(query, basePose, min_dist_thresh);
}

void NNGridTable::MakeCellPoints(std::vector<ScanPoint2D> &points) {
  double cell_thresh =
      ParamServer::Get("PointCloudMapGT_CELL_POINT_NUM_THRESH");

  MakeCellPointsImpl(cell_thresh, points);
}

void NNGridTable::MakeCellPointsImpl(int cell_point_num_thresh,
                                     std::vector<ScanPoint2D> &cellPoints) {
  for (const auto &cell : m_table) {
    auto &points = cell.points;
    if (points.size() >= cell_point_num_thresh) {
      double gx = 0, gy = 0;
      double nx = 0, ny = 0;

      for (const auto iter : points) {
        gx += iter->x();
        gy += iter->y();
        nx += iter->nx();
        ny += iter->ny();
      }
      gx /= points.size();
      gy /= points.size();
      double L = std::hypot(nx, ny);
      nx /= L;
      ny /= L;

      cellPoints.emplace_back(gx, gy, nx, ny, ScanPoint2D::PointType::LINE);
    }
  }
}
} // namespace slam
