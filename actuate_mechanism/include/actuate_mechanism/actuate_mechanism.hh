#ifndef __ACTUATE_MECHANISM_HH__
#define __ACTUATE_MECHANISM_HH__
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <sensor_msgs/JointState.h>
#include <ee_force/eeForceMsg.h>

class ActuateMechanism {
protected:
  /*
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
  */

  //actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> *move_arm_;
  arm_navigation_msgs::MoveArmGoal goalB_;  
  arm_navigation_msgs::SimplePoseConstraint desired_pose_;
  ros::NodeHandle nh_;
  double x_position_;


public:
  ActuateMechanism();
  ~ActuateMechanism();
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm_;

  void move_arm_to_pose(double x_position);
  void ee_force_callback(ee_force::eeForceMsg msg);


};

#endif //actuate_mechanism.hh
