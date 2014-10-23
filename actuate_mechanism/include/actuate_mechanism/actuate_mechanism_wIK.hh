#ifndef __ACTUATE_MECHANISM_WIK_HH__
#define __ACTUATE_MECHANISM_WIK_HH__
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <sensor_msgs/JointState.h>
#include <ee_force/eeForceMsg.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <vector>
#include <iostream>


typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class ActuateMechanismwIK {
protected:
  arm_navigation_msgs::MoveArmGoal goalB_;  
  arm_navigation_msgs::SimplePoseConstraint desired_pose_;
  ros::NodeHandle nh_;
  double x_position_;
  double y_position_;
  std::vector<double> sol_;

public:
  ActuateMechanismwIK();
  ~ActuateMechanismwIK();
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;
  //actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm_;

  void move_arm_to_pose(double x_position);
  void ee_force_callback(ee_force::eeForceMsg msg);
  void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal);
  actionlib::SimpleClientGoalState getState();
  pr2_controllers_msgs::JointTrajectoryGoal createJointGoal(double x_position, double y_position, double current[]);
  void ActuateMechanismwIK::move_arm_and_wait(double x_position, double y_position, double current[]);
  std::vector<double> calculateSol(double x_position, double y_position, double currentTemp[]);
  void ActuateMechanismwIK::calcNewPose(ee_force::eeForceMsg msg);



};

#endif //actuate_mechanism_wIK.hh
