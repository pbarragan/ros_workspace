#ifndef __SLIDE_BOX_HH__
#define __SLIDE_BOX_HH__
//#include <actionlib/client/simple_action_client.h>
//#include <arm_navigation_msgs/MoveArmAction.h>
//#include <sensor_msgs/JointState.h>
//#include <ee_force/eeForceMsg.h>
//#include "/home/barragan/pr2_repo/ros_workspace/ee_force/msg_gen/cpp/include/ee_force/eeForceMsg.h"
//#include <pr2_controllers_msgs/JointTrajectoryAction.h>

#include <ee_cart_imped_action/ee_cart_imped_arm.hh>
#include <ee_cart_imped_msgs/EECartImpedGoal.h>

#include <vector>
#include <iostream>

#include "slide_box/robot_actuate_object.h"
#include <tf/transform_listener.h>

class ActuateMechanismwIK {
protected:
  EECartImpedArm arm_;//("l_arm_cart_imped_controller");

  //start position
  double start_x_position_;
  double start_y_position_;
  double start_z_position_;
  //start rotation
  double start_x_rotation_;
  double start_y_rotation_;
  double start_z_rotation_;
  double start_w_rotation_;



  // current stiffnesses
  double Kx_;
  double Ky_;

  // current force direction
  double fdx_;
  double fdy_;
  double fdz_;
  double fdw_;

  tf::TransformListener TFlistener_;
  tf::StampedTransform TFtransform_;
  std::string chain_root_;
  std::string chain_tip_;

public:

  // current position
  double x_position_;
  double y_position_;

  ActuateMechanismwIK();
  //~ActuateMechanismwIK();
  
  void createGoal(double x_position,double y_position,ee_cart_imped_msgs::EECartImpedGoal &traj);

  void move_arm_and_wait(double x_position,double y_position);
  void calcStiffnessesAndDirection(double aX, double aY);
  bool actuate(slide_box::robot_actuate_object::Request  &req,
	       slide_box::robot_actuate_object::Response &res);

};

#endif //slide_box.hh
