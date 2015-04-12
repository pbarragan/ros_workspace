#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/client/simple_action_client.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <sensor_msgs/JointState.h>
#include <slide_box/slide_box.hh>
//#include <ee_force/eeForceMsg.h>
//#include <pr2_controllers_msgs/JointTrajectoryAction.h>
//#include <vector>
//#include <iostream>

//#include "slide_box/robot_actuate_object.h"
//#include <tf/transform_listener.h>

//#include <string>
//#include "std_msgs/String.h"
//#include <stdlib.h>
//#include <stdio.h>

//#include <typeinfo>

#include <math.h> // cos, sin, atan2

ActuateMechanismwIK::ActuateMechanismwIK():arm_("l_arm_cart_imped_controller"){
  //EECartImpedArm armTemp("l_arm_cart_imped_controller");
  //arm_ = armTemp;
  //arm_ = new EECartImpedArm("r_arm_cart_imped_controller");
  //for TF
  std::cout << "hello" << std::endl;
  chain_root_ = "/torso_lift_link";
  chain_tip_ = "/l_gripper_tool_frame";

  //get start position of the arm to set the zero on the coordinate system
  TFlistener_.waitForTransform(chain_root_, chain_tip_,ros::Time(0),ros::Duration(3.0));
  TFlistener_.lookupTransform(chain_root_, chain_tip_, ros::Time(0), TFtransform_);
  
  start_x_position_ = TFtransform_.getOrigin().x();
  std::cout << "start x: " << start_x_position_ << std::endl;
  start_y_position_ = TFtransform_.getOrigin().y();
  std::cout << "start y: " << start_y_position_ << std::endl;
  start_z_position_ = TFtransform_.getOrigin().z();
  std::cout << "start z: " << start_z_position_ << std::endl;

  //get start rotation
  start_x_rotation_ = TFtransform_.getRotation().x();  
  std::cout << "start x rot: " << start_x_rotation_ << std::endl;
  start_y_rotation_ = TFtransform_.getRotation().y();
  std::cout << "start y rot: " << start_y_rotation_ << std::endl;
  start_z_rotation_ = TFtransform_.getRotation().z();
  std::cout << "start z rot: " << start_z_rotation_ << std::endl;
  start_w_rotation_ = TFtransform_.getRotation().w();
  std::cout << "start w rot: " << start_w_rotation_ << std::endl;

  ee_cart_imped_msgs::EECartImpedGoal traj;

  EECartImpedArm::addTrajectoryPoint(traj, 
				     0.75, 0.20, start_z_position_, 
				     start_x_rotation_, start_y_rotation_,
				     start_z_rotation_, start_w_rotation_,
                                     1000, 1000, 1000, 30, 30, 30,
                                     false, false, false, false, false,
                                     false, 2, "/torso_lift_link",
				     0,0,0,1);

  arm_.startTrajectory(traj); // Wait for trajectory

  // Repeat after initial move
  //get start position of the arm to set the zero on the coordinate system
  TFlistener_.waitForTransform(chain_root_, chain_tip_,ros::Time(0),ros::Duration(3.0));
  TFlistener_.lookupTransform(chain_root_, chain_tip_, ros::Time(0), TFtransform_);
  
  start_x_position_ = TFtransform_.getOrigin().x();
  std::cout << "start x: " << start_x_position_ << std::endl;
  start_y_position_ = TFtransform_.getOrigin().y();
  std::cout << "start y: " << start_y_position_ << std::endl;
  start_z_position_ = TFtransform_.getOrigin().z();
  std::cout << "start z: " << start_z_position_ << std::endl;

  //get start rotation
  start_x_rotation_ = TFtransform_.getRotation().x();  
  std::cout << "start x rot: " << start_x_rotation_ << std::endl;
  start_y_rotation_ = TFtransform_.getRotation().y();
  std::cout << "start y rot: " << start_y_rotation_ << std::endl;
  start_z_rotation_ = TFtransform_.getRotation().z();
  std::cout << "start z rot: " << start_z_rotation_ << std::endl;
  start_w_rotation_ = TFtransform_.getRotation().w();
  std::cout << "start w rot: " << start_w_rotation_ << std::endl;

  // set running current position
  x_position_ = start_x_position_;
  y_position_ = start_y_position_;

}

/*
ActuateMechanismwIK::~ActuateMechanismwIK(){
  delete traj_client_;
}
*/

void ActuateMechanismwIK::createGoal(double x_position,double y_position,ee_cart_imped_msgs::EECartImpedGoal &traj){
  // Note: always making Z stiffness really low as well
  EECartImpedArm::addTrajectoryPoint(traj, 
				     x_position, y_position, start_z_position_, 
				     start_x_rotation_, start_y_rotation_,
				     start_z_rotation_, start_w_rotation_,
                                     Kx_, Ky_, 30, 30, 30, 30,
                                     false, false, false, false, false,
                                     false, 2, "/torso_lift_link",
				     fdx_,fdy_,fdz_,fdw_);
}

void ActuateMechanismwIK::move_arm_and_wait(double x_position, 
					    double y_position){
  ee_cart_imped_msgs::EECartImpedGoal traj;
  // creat the goal
  createGoal(x_position,y_position,traj);
  
  // start the goal
  arm_.startTrajectory(traj,false); // Don't wait for trajectory

  // wait for the goal
  for (size_t i = 0; i<40; i++){
    usleep(100000);
  }
  
  // stop the goal
  arm_.stopTrajectory();
}


void ActuateMechanismwIK::calcStiffnessesAndDirection(double aX, double aY){
  // these are hardcoded parameters
  double KxP = 400; // high stiffness
  double KyP = 100; // low stiffness

  // Cacluate angle of action
  double theta = atan2(aY,aX);

  // Calculate stiffnesses aligned with action
  Kx_ = KxP;
  Ky_ = KyP;
  
  // Calculate quaternion of action direction
  // Assume for now that we only rotate around positive z axis (0,0,1)
  fdx_ = 0;
  fdy_ = 0;
  fdz_ = sin(theta*0.5);
  fdw_ = cos(theta*0.5);

  std::cout << "Kx" << Kx_ << std::endl;
  std::cout << "Ky" << Ky_ << std::endl;
  std::cout << "fdx" << fdx_ << std::endl;
  std::cout << "fdy" << fdy_ << std::endl;
  std::cout << "fdz" << fdz_ << std::endl;
  std::cout << "fdw" << fdw_ << std::endl;
}

bool ActuateMechanismwIK::actuate(slide_box::robot_actuate_object::Request  &req,
         slide_box::robot_actuate_object::Response &res)
{
  ROS_INFO("Recieved actuation command\n");

  // BIG CHANGE to RELATIVE ACTIONS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // this should wait until the move is done to return
  // the action in the request should be relative to the gripper pose of the 
  // robot

  std::cout << "moving to x: " << x_position_+req.action[0] << std::endl;
  std::cout << "moving to y: " << y_position_+req.action[1] << std::endl;

  calcStiffnessesAndDirection(req.action[0],req.action[1]); // sets class variables

  move_arm_and_wait(x_position_+req.action[0],
		    y_position_+req.action[1]);

  //get the position of the hand
  try{
    //ros::Time now;
    TFlistener_.waitForTransform(chain_root_, chain_tip_,
				 ros::Time(0),ros::Duration(3.0));

    ROS_INFO("Can transform from %s to %s is %d", 
	     chain_root_.c_str(), chain_tip_.c_str(), 
	     TFlistener_.canTransform(chain_root_, chain_tip_, ros::Time(0)));
    TFlistener_.lookupTransform(chain_root_, chain_tip_,ros::Time(0), 
				TFtransform_);
    ROS_INFO_STREAM("Transform is " << TFtransform_.getOrigin().x()
		    << ", " << TFtransform_.getOrigin().y() << ", "
		    << TFtransform_.getOrigin().z() << ", "
		    << TFtransform_.getRotation().x() << ", "
		    << TFtransform_.getRotation().y() << ", "
		    << TFtransform_.getRotation().z() << ", "
		    << TFtransform_.getRotation().w());
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  x_position_ = TFtransform_.getOrigin().x();
  y_position_ = TFtransform_.getOrigin().y();

  std::cout << "Got to x: " << x_position_ << std::endl;
  std::cout << "Got to y: " << y_position_ << std::endl;
  std::cout << "Relative to start x: " << 
    x_position_-start_x_position_ << std::endl;
  std::cout << "Relative to start y: " << 
    y_position_-start_y_position_<< std::endl;


  //req.action;
  //res.obs;
  //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  //ROS_INFO("sending back response: [%ld]", (long int)res.sum);

  //try this out
  // relax step
  Kx_ = 100;
  Ky_ = 100;
  move_arm_and_wait(x_position_,y_position_);


  //get the position of the hand again
  try{
    //ros::Time now;
    TFlistener_.waitForTransform(chain_root_, chain_tip_,
				 ros::Time(0),ros::Duration(3.0));

    ROS_INFO("Can transform from %s to %s is %d", 
	     chain_root_.c_str(), chain_tip_.c_str(), 
	     TFlistener_.canTransform(chain_root_, chain_tip_, ros::Time(0)));
    TFlistener_.lookupTransform(chain_root_, chain_tip_,ros::Time(0), 
				TFtransform_);
    ROS_INFO_STREAM("Transform is " << TFtransform_.getOrigin().x()
		    << ", " << TFtransform_.getOrigin().y() << ", "
		    << TFtransform_.getOrigin().z() << ", "
		    << TFtransform_.getRotation().x() << ", "
		    << TFtransform_.getRotation().y() << ", "
		    << TFtransform_.getRotation().z() << ", "
		    << TFtransform_.getRotation().w());
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  x_position_ = TFtransform_.getOrigin().x();
  y_position_ = TFtransform_.getOrigin().y();

  std::cout << "Got to x: " << x_position_ << std::endl;
  std::cout << "Got to y: " << y_position_ << std::endl;
  std::cout << "Relative to start x: " << 
    x_position_-start_x_position_ << std::endl;
  std::cout << "Relative to start y: " << 
    y_position_-start_y_position_<< std::endl;


  // Write observation

  res.obs.push_back(x_position_-start_x_position_);
  //std::cout << "x: " << (double) TFtransform_.getOrigin().x() << std::endl;
  //std::cout<<"x is of type: "<<typeid(TFtransform_.getOrigin().x()).name()<<std::endl;
  res.obs.push_back(y_position_-start_y_position_); 
  //res.obs.push_back(0.25);  
  res.obs.push_back(TFtransform_.getOrigin().z()-start_z_position_);







  return true;
}

int main(int argc, char **argv){ 
  std::cout << "I'm running" << std::endl;
  ros::init (argc, argv, "move_arm_pose_goal_test");
  ros::NodeHandle node;
  std::cout << "I'm running" << std::endl;
  ActuateMechanismwIK actMechwIK;
  std::cout << "I'm running" << std::endl;


  /*
  actMechwIK.calcStiffnessesAndDirection(0.0,0.10); // sets class variables

  actMechwIK.move_arm_and_wait(actMechwIK.x_position_+0.0,
			       actMechwIK.y_position_+0.10);

  actMechwIK.calcStiffnessesAndDirection(0.0,0.01); // sets class variables

  actMechwIK.move_arm_and_wait(actMechwIK.x_position_+0.00,
			       actMechwIK.y_position_+0.0);
  */

  ros::ServiceServer service = node.advertiseService("robot_actuate_object", &ActuateMechanismwIK::actuate, &actMechwIK);

  //subscribers
  /*
  ros::Subscriber sub_ee_force = node.subscribe("/ee_force", 1, &ActuateMechanismwIK::ee_force_callback, &actMechwIK);
  */

  //actMech.move_arm_to_pose(0.70);
  ros::spin();

  return 0;
}
