#ifndef POSE_FUSER_H
#define POSE_FUSER_H

#include <Eigen/Eigen>

#include <slam/icp/DataAssociator.h>

class PoseFuser {
 public:
  PoseFuser(){}
  ~PoseFuser(){}

 private:
  Eigen::Matrix3d m_ecov;
  Eigen::Matrix3d m_mcov;
  Eigen::Matrix3d m_totalcov;
  
  DataAssociator *m_data_associator_ptr;
}
#endif /* POSE_FUSER_H */
