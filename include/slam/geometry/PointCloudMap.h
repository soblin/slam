#ifndef POINT_CLOUD_MAP
#define POINT_CLOUD_MAP

#include <memory>
#include <vector>

#include <slam/geometry/Scan2D.h>
#include <slam/geometry/Pose2D.h>
#include <slam/geometry/ScanPoint2D.h>

namespace slam{

class PointCloudMap{
 protected:
  // accumulated poses and scaned points(global frame)
  std::vector<Pose2D> m_poses;
  std::vector<ScanPoint2D> m_global_map;
  std::vector<ScanPoint2D> m_local_map;

  // for ICP
  Pose2D m_last_pose;
  Scan2D m_last_scan;
  
 public:
  inline const std::vector<Pose2D>& poses() const { return m_poses; }
  inline const std::vector<ScanPoint2D>& global_map() const { return m_global_map; }

  inline void GetLastPose(Pose2D& pose) const { pose = m_last_pose; }
  inline void GetLastScan(Scan2D& scan) const { scan = m_last_scan; }
  inline const Scan2D& GetLastScan() const { return m_last_scan; }
  inline const Pose2D& GetLastPose() const { return m_last_pose; }
  inline void SetLastPose(const Pose2D& pose) { m_last_pose = pose; }
  inline void SetLastScan(const Scan2D& scan) { m_last_scan = scan; }
  //  inline void SetCellThresh(int n) { m_cell_thresh = n; }
  //  inline void GetCellThresh(int& n) const { n = m_cell_thresh; }
  
  PointCloudMap();
  ~PointCloudMap();
  virtual void AddPose(const Pose2D& pose) = 0;
  virtual void AddPoint(const ScanPoint2D& scan) = 0;
  virtual void AddPoints(const std::vector<ScanPoint2D>& scans) = 0;
  virtual void MakeGlobalMap() = 0;
  virtual void MakeLocalMap() = 0;
  virtual void RemakeMaps(const std::vector<Pose2D>& newposes) = 0;
};

class PointCloudMapSingleton{
 public:
  static PointCloudMap *GetCloudMap(){
    return m_point_cloud_map_ptr;
  }
  static void Create(PointCloudMap *ptr){
    if(m_instance_ptr == nullptr){
      m_instance_ptr = new PointCloudMapSingleton;
    }
    m_point_cloud_map_ptr = ptr;
  }

 private:
  PointCloudMapSingleton() {};
  static PointCloudMapSingleton *m_instance_ptr;
  static PointCloudMap *m_point_cloud_map_ptr;
};

} /* namespace slam */
#endif
