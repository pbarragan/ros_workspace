#ifndef __EE_FORCE_PUB_HH__
#define __EE_FORCE_PUB_HH__
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <sensor_msgs/JointState.h>

class EEForcePub {
protected:
  KDL::Chain chain_;
  KDL::ChainJntToJacSolver *jac_solver_;
  KDL::JntArray q_in_;
  KDL::Jacobian jac_;
  Eigen::Matrix<double,7,6> jac_T_;
  unsigned int numJnts;
  Eigen::Matrix<double,7,1> jointEfforts_;
  Eigen::Matrix<double,6,1> eeEffort_;
  Eigen::Matrix<double,6,1> eeEffort2_;
  std::string linkNames_[7];

public:
  EEForcePub();
  ~EEForcePub();

  void joint_states_callback(sensor_msgs::JointState msg);


};

#endif //ee_force_pub.hh
