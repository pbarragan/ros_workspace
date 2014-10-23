#ifndef __SLIDE_BOX_HH__
#define __SLIDE_BOX_HH__
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <sensor_msgs/JointState.h>
#include <ee_force/eeForceMsg.h>
//#include "/home/barragan/pr2_repo/ros_workspace/ee_force/msg_gen/cpp/include/ee_force/eeForceMsg.h"
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <vector>
#include <iostream>

#include "slide_box/robot_actuate_object.h"
#include <tf/transform_listener.h>

//#include <string>
//#include "std_msgs/String.h"
//#include <stdlib.h>
//#include <stdio.h>




typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class ActuateMechanismwIK {
protected:
  arm_navigation_msgs::MoveArmGoal goalB_;  
  arm_navigation_msgs::SimplePoseConstraint desired_pose_;
  ros::NodeHandle nh_;
  //start position
  double start_x_position_;
  double start_y_position_;
  double start_z_position_;
  //start rotation
  double start_x_rotation_;
  double start_y_rotation_;
  double start_z_rotation_;
  double start_w_rotation_;

  //transformation matrix
  double T_ [4][4];

  std::vector<double> sol_;
  std::vector<double> currentJointAngles_;
  tf::TransformListener TFlistener_;
  tf::StampedTransform TFtransform_;
  std::string chain_root_;
  std::string chain_tip_;
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
  pr2_controllers_msgs::JointTrajectoryGoal createJointGoal(double x_position, double y_position, std::vector<double> current);
  void move_arm_and_wait(double x_position, double y_position, std::vector<double> current);
  std::vector<double> calculateSol(double x_position, double y_position, std::vector<double> current);
  void calcNewPose(ee_force::eeForceMsg msg);
  bool actuate(slide_box::robot_actuate_object::Request  &req,
	       slide_box::robot_actuate_object::Response &res);


};

#endif //slide_box.hh