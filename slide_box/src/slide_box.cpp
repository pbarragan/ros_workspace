#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/client/simple_action_client.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <sensor_msgs/JointState.h>
#include <slide_box/slide_box.hh>
//#include <ee_force/eeForceMsg.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <vector>
#include <iostream>

#include "slide_box/robot_actuate_object.h"
#include <tf/transform_listener.h>

//#include <string>
//#include "std_msgs/String.h"
//#include <stdlib.h>
//#include <stdio.h>

#include <typeinfo>


typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

std::vector<double> pr2KinIKfast(double T[4][4], std::vector<double> current);


ActuateMechanismwIK::ActuateMechanismwIK(){
  //move_arm_ = new actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction>("move_left_arm",true);

  traj_client_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);

  // wait for action server to come up
  while(!traj_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the joint_trajectory_action server");
  }

  //for TF
  chain_root_ = "/torso_lift_link";
  chain_tip_ = "/l_gripper_tool_frame";

  //get current position of the arm to set the zero on the coordinate system
  TFlistener_.waitForTransform(chain_root_, chain_tip_,ros::Time(0),ros::Duration(3.0));
  TFlistener_.lookupTransform(chain_root_, chain_tip_, ros::Time(0), TFtransform_);
  
  start_x_position_ = TFtransform_.getOrigin().x();
  std::cout << "start x: " << start_x_position_ << std::endl;
  start_y_position_ = TFtransform_.getOrigin().y();
  std::cout << "start y: " << start_y_position_ << std::endl;
  start_z_position_ = TFtransform_.getOrigin().z();
  std::cout << "start z: " << start_z_position_ << std::endl;

  
  //get current rotation
  start_x_rotation_ = TFtransform_.getRotation().x();  
  std::cout << "start x rot: " << start_x_rotation_ << std::endl;
  start_y_rotation_ = TFtransform_.getRotation().y();
  std::cout << "start y rot: " << start_y_rotation_ << std::endl;
  start_z_rotation_ = TFtransform_.getRotation().z();
  std::cout << "start z rot: " << start_z_rotation_ << std::endl;
  start_w_rotation_ = TFtransform_.getRotation().w();
  std::cout << "start w rot: " << start_w_rotation_ << std::endl;

  //this is a huge mess. compensate for the differences by rotating around the +y axis 90 degrees which seems to be offset from open rave to pr2

  double aR = start_w_rotation_;
  double bR = start_x_rotation_;
  double cR = start_y_rotation_;
  double dR = start_z_rotation_;

  double aC = 0.70710678118;
  double bC = 0;
  double cC = 0.70710678118;
  double dC = 0;

  double a = aR*aC - bR*bC - cR*cC - dR*dC;
  double b = aR*bC + bR*aC - cR*dC - dR*cC;
  double c = aR*cC + bR*dC + cR*aC - dR*bC;
  double d = aR*dC - bR*cC + cR*bC + dR*aC;
  
  //convert to a translation matrix
  //from http://renderfeather.googlecode.com/hg-history/034a1900d6e8b6c92440382658d2b01fc732c5de/Doc/optimized%20Matrix%20quaternion%20conversion.pdf
  //T_[0][0] = 1-2*start_y_rotation_*start_y_rotation_-2*start_z_rotation_*start_z_rotation_;
  T_[0][0] = a*a+b*b-c*c-d*d;
  std::cout << "00: " << T_[0][0] << std::endl;
  //T_[0][1] = 2*start_x_rotation_*start_y_rotation_+2*start_w_rotation_*start_z_rotation_;
  T_[0][1] = 2*b*c-2*a*d;
  std::cout << "01: " << T_[0][1] << std::endl;
  //T_[0][2] = 2*start_x_rotation_*start_z_rotation_-2*start_w_rotation_*start_y_rotation_;
  T_[0][2] = 2*b*d+2*a*c;
  std::cout << "02: " << T_[0][2] << std::endl;
  T_[0][3] = start_x_position_;
  std::cout << "03: " << T_[0][3] << std::endl;
  //T_[1][0] =2*start_x_rotation_*start_y_rotation_-2*start_w_rotation_*start_z_rotation_;
  T_[1][0] = 2*b*c+2*a*d;
  std::cout << "10: " << T_[1][0] << std::endl;
  //T_[1][1] =1-2*start_x_rotation_*start_x_rotation_-2*start_z_rotation_*start_z_rotation_;
  T_[1][1] = a*a-b*b+c*c-d*d;
  std::cout << "11: " << T_[1][1] << std::endl;
  //T_[1][2] =2*start_y_rotation_*start_z_rotation_+2*start_w_rotation_*start_x_rotation_;
  T_[1][2] = 2*c*d-2*a*b;
  std::cout << "12: " << T_[1][2] << std::endl;
  T_[1][3] = start_y_position_;
  std::cout << "13: " << T_[1][3] << std::endl;
  //T_[2][0] =2*start_x_rotation_*start_z_rotation_+2*start_w_rotation_*start_y_rotation_;
  T_[2][0] = 2*b*d-2*a*c;
  std::cout << "20: " << T_[2][0] << std::endl;
  //T_[2][1] =2*start_y_rotation_*start_z_rotation_-2*start_w_rotation_*start_x_rotation_;
  T_[2][1] = 2*c*d+2*a*b;
  std::cout << "21: " << T_[2][1] << std::endl;
  //T_[2][2] =1-2*start_x_rotation_*start_x_rotation_-2*start_y_rotation_*start_y_rotation_;
  T_[2][2] = a*a-b*b-c*c+d*d;
  std::cout << "22: " << T_[2][2] << std::endl;
  T_[2][3] = start_z_position_;  
  std::cout << "23: " << T_[2][3] << std::endl;
  T_[3][0] = 0;
  T_[3][1] = 0;
  T_[3][2] = 0;
  T_[3][3] = 1;


  /*
  T_[0][0] = 0.70710678118;
  T_[0][1] = 0;
  T_[0][2] = 0.70710678118;
  T_[1][0] = 0;
  T_[1][1] = 1;
  T_[1][2] = 0;
  T_[2][0] = -0.70710678118;
  T_[2][1] = 0;
  T_[2][2] = 0.70710678118;
  */
}

ActuateMechanismwIK::~ActuateMechanismwIK(){
  delete traj_client_;
}

//! Sends the command to start a given trajectory
void ActuateMechanismwIK::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal){
  // When to start the trajectory: 0.1s from now
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);
  traj_client_->sendGoal(goal);
}

std::vector<double> ActuateMechanismwIK::calculateSol(double x_position, double y_position, std::vector<double> current){
  //double T [4][4] = {{1,0,0,x_position},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
  //  double T [4][4] = {{0.707,0,0.707,x_position},{0,1,0,0.10},{-.707,0,.707,0},{0,0,0,1}};
  
  //double T [4][4] = {{0,0,1,x_position},{0,1,0,y_position},{-1,0,0,0},{0,0,0,1}};

  T_[0][3]=x_position;
  T_[1][3]=y_position;

  //static const double currentTemp[] = {0,0,0,0,0,0,0};
  //std::vector<double> current(currentTemp, currentTemp + sizeof(currentTemp)/sizeof(currentTemp[0]) );
  std::vector<double> sol = pr2KinIKfast(T_, current);
  
  if(sol.size()>0){
    std::cout << "Best Solution:" << std::endl;
    
    for (std::vector<float>::size_type ii = 0; ii < sol.size(); ii++) {
      std::cout << sol[ii] << std::endl;
    }
  }
  else{
    std::cout << "No Solution Found. RUN FOR THE HILLS!" << std::endl;
    exit(1); //We are totally screwed
  }

  return sol; 
  //cout << "Did that work" << endl;
}

pr2_controllers_msgs::JointTrajectoryGoal ActuateMechanismwIK::createJointGoal(double x_position, double y_position, std::vector<double> current){
  
  //calculate the solution
  sol_ = calculateSol(x_position, y_position, current);

  //our goal variable
  pr2_controllers_msgs::JointTrajectoryGoal goal;
  
  // First, the joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
  goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
  goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
  goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
  goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
  goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
  goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
  

  //Save this for later. This is the robot giving the chop. Heck yes!
  /*
  // We will have one waypoint in this goal trajectory
  goal.trajectory.points.resize(2);
  
  // Positions
  int ind = 0;
  goal.trajectory.points[ind].positions.resize(7);
  goal.trajectory.points[ind].positions[0] = 0.0;
  goal.trajectory.points[ind].positions[1] = 0.0;
  goal.trajectory.points[ind].positions[2] = 0.0;
  goal.trajectory.points[ind].positions[3] = 0.0;
  goal.trajectory.points[ind].positions[4] = 0.0;
  goal.trajectory.points[ind].positions[5] = 0.0;
  goal.trajectory.points[ind].positions[6] = 0.0;
  // Velocities
  goal.trajectory.points[ind].velocities.resize(7);
  for (size_t j = 0; j < 7; ++j){
    goal.trajectory.points[ind].velocities[j] = 0.0;
  }
  // To be reached 1 second after starting along the trajectory
  goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);
  
  // Second trajectory point
  // Positions
  ind += 1;
  goal.trajectory.points[ind].positions.resize(7);
  goal.trajectory.points[ind].positions[0] = -0.3;
  goal.trajectory.points[ind].positions[1] = 0.2;
  goal.trajectory.points[ind].positions[2] = -0.1;
  goal.trajectory.points[ind].positions[3] = -1.2;
  goal.trajectory.points[ind].positions[4] = 1.5;
  goal.trajectory.points[ind].positions[5] = -0.3;
  goal.trajectory.points[ind].positions[6] = 0.5;
  // Velocities
  goal.trajectory.points[ind].velocities.resize(7);
  for (size_t j = 0; j < 7; ++j){
    goal.trajectory.points[ind].velocities[j] = 0.0;
  }
  // To be reached 2 seconds after starting along the trajectory
  goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);
  */

  // We will have one waypoint in this goal trajectory
  goal.trajectory.points.resize(1);
  
  // Positions
  goal.trajectory.points[0].positions.resize(7);
  goal.trajectory.points[0].positions[0] = sol_[0];
  goal.trajectory.points[0].positions[1] = sol_[1];
  goal.trajectory.points[0].positions[2] = sol_[2];
  goal.trajectory.points[0].positions[3] = sol_[3];
  goal.trajectory.points[0].positions[4] = sol_[4];
  goal.trajectory.points[0].positions[5] = sol_[5];
  goal.trajectory.points[0].positions[6] = sol_[6];
  // Velocities
  goal.trajectory.points[0].velocities.resize(7);
  for (size_t j = 0; j < 7; ++j){
    goal.trajectory.points[0].velocities[j] = 0.0;
  }
  // To be reached 1 second after starting along the trajectory
  goal.trajectory.points[0].time_from_start = ros::Duration(1.0);
  
  //we are done; return the goal
  return goal;
}

//! Returns the current state of the action
actionlib::SimpleClientGoalState ActuateMechanismwIK::getState(){
  return traj_client_->getState();
}

/*
void ActuateMechanismwIK::move_arm_to_pose(double x_position){
  arm_navigation_msgs::MoveArmGoal goalA_;

  goalA_.motion_plan_request.group_name = "left_arm";
  goalA_.motion_plan_request.num_planning_attempts = 10;
  goalA_.motion_plan_request.planner_id = std::string("");
  goalA_.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalA_.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  desired_pose_.pose.position.x = x_position;
  desired_pose_.pose.position.y = 0.10;
  desired_pose_.pose.position.z = 0;

  desired_pose_.pose.orientation.x = 0.0;
  desired_pose_.pose.orientation.y = 0.0;
  desired_pose_.pose.orientation.z = 0.0;
  desired_pose_.pose.orientation.w = 1.0;

  arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose_,goalA_);

  if (nh_.ok())
    {
      bool finished_within_time = false;
      move_arm_.sendGoal(goalA_);
      finished_within_time = move_arm_.waitForResult(ros::Duration(200.0));
      if (!finished_within_time)
	{
	  move_arm_.cancelGoal();
	  ROS_INFO("Timed out achieving goal A");
	}
      else
	{
	  actionlib::SimpleClientGoalState state = move_arm_.getState();
	  bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
	  if(success)
	    ROS_INFO("Action finished: %s",state.toString().c_str());
	  else
	    ROS_INFO("Action failed: %s",state.toString().c_str());
	}
    }
  
}
*/

void ActuateMechanismwIK::move_arm_and_wait(double x_position, double y_position, std::vector<double> current){
  startTrajectory(createJointGoal(x_position,y_position,current));
  for (size_t i = 0; i<30; i++){
    std::cout << getState().toString() << std::endl;
    usleep(100000);
  }
  traj_client_->cancelGoal();
  // Wait for trajectory completion
  while(!getState().isDone() && ros::ok())
    {
      //std::cout << getState().toString() << std::endl;
      usleep(50000);
    }
}

void ActuateMechanismwIK::calcNewPose(ee_force::eeForceMsg msg){

}


void ActuateMechanismwIK::ee_force_callback(ee_force::eeForceMsg msg){
  //ROS_INFO("Got ee_force msg\n");

  //in this function, all we want for now is the joint angles
  currentJointAngles_= msg.jointAngles;

}



bool ActuateMechanismwIK::actuate(slide_box::robot_actuate_object::Request  &req,
         slide_box::robot_actuate_object::Response &res)
{
  ROS_INFO("Recieved actuation command\n");

  //this should wait until the move is done to return
  //the action in the request should be relative to the start pose of the robot
  //this is because the start pose will "zero" the coordinate frame

  std::cout << "moving to x: " << start_x_position_+req.action[0] << std::endl;
  std::cout << "moving to y: " << start_y_position_+req.action[1] << std::endl;


  move_arm_and_wait(start_x_position_+req.action[0],start_y_position_+req.action[1],currentJointAngles_);
  //get the position of the hand

  try{
    //ros::Time now;
    TFlistener_.waitForTransform(chain_root_, chain_tip_,ros::Time(0),ros::Duration(3.0));

    ROS_INFO("Can transform from %s to %s is %d", 
	     chain_root_.c_str(), chain_tip_.c_str(), 
	     TFlistener_.canTransform(chain_root_, chain_tip_, ros::Time(0)));
    TFlistener_.lookupTransform(chain_root_, chain_tip_,ros::Time(0), TFtransform_);
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

  res.obs.push_back(TFtransform_.getOrigin().x()-start_x_position_);
  //std::cout << "x: " << (double) TFtransform_.getOrigin().x() << std::endl;
  //std::cout<<"x is of type: "<<typeid(TFtransform_.getOrigin().x()).name()<<std::endl;
  res.obs.push_back(TFtransform_.getOrigin().y()-start_y_position_); 
  //res.obs.push_back(0.25);  
  res.obs.push_back(TFtransform_.getOrigin().z()-start_z_position_);

  //req.action;
  //res.obs;
  //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  //ROS_INFO("sending back response: [%ld]", (long int)res.sum);

  //try this out
  move_arm_and_wait(TFtransform_.getOrigin().x(),TFtransform_.getOrigin().y(),currentJointAngles_);

  return true;
}

int main(int argc, char **argv){
  std::cout << "I'm running" << std::endl;
  ros::init (argc, argv, "move_arm_pose_goal_test");
  ros::NodeHandle node;
  //actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("copy move_left_arm",true);

  ActuateMechanismwIK actMechwIK;

  ros::ServiceServer service = node.advertiseService("robot_actuate_object", &ActuateMechanismwIK::actuate, &actMechwIK);

  //subscribers
  ros::Subscriber sub_ee_force = node.subscribe("/ee_force", 1, &ActuateMechanismwIK::ee_force_callback, &actMechwIK);
  
  //actMech.move_arm_to_pose(0.70);
  ros::spin();

  return 0;
}
