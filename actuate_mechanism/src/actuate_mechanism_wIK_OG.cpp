#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <sensor_msgs/JointState.h>
#include <actuate_mechanism/actuate_mechanism_wIK.hh>
#include <ee_force/eeForceMsg.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <vector>
#include <iostream>

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

std::vector<double> pr2KinIKfast(double T[4][4], std::vector<double> current);


ActuateMechanismwIK::ActuateMechanismwIK(){
  //move_arm_ = new actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction>("move_left_arm",true);

  traj_client_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);

  // wait for action server to come up
  while(!traj_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the joint_trajectory_action server");
  }
  
  x_position_ = 0.55;
  std::vector<double> sol_;

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

std::vector<double> ActuateMechanismwIK::calculateSol(double x_position){
  //double T [4][4] = {{1,0,0,x_position},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
  double T [4][4] = {{0.707,0,0.707,x_position},{0,1,0,0.10},{-.707,0,.707,0},{0,0,0,1}};
  static const double currentTemp[] = {0,0,0,0,0,0,0};
  std::vector<double> current(currentTemp, currentTemp + sizeof(currentTemp)/sizeof(currentTemp[0]) );
  std::vector<double> sol = pr2KinIKfast(T, current);
  
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

pr2_controllers_msgs::JointTrajectoryGoal ActuateMechanismwIK::armExtensionTrajectory(double x_position){
  
  //calculate the solution
  sol_ = calculateSol(x_position);

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

void ActuateMechanismwIK::move_arm_and_wait(double x_position){
  startTrajectory(armExtensionTrajectory(x_position));
  // Wait for trajectory completion
  while(!getState().isDone() && ros::ok())
    {
      usleep(50000);
    }
}

void ActuateMechanismwIK::ee_force_callback(ee_force::eeForceMsg msg){
  ROS_INFO("Got ee_force msg\n");

  double xForce = msg.data[0];

  if (x_position_ < 0.70){
    if (xForce > 10){
      ROS_INFO("I found something!\n");
    }
    else if (xForce <= 10){
      ROS_INFO("Nothing yet...\n");
      move_arm_and_wait(x_position_);
      x_position_ += 0.03;   
    }
  }
  else {
    x_position_ = 0.55;
    move_arm_and_wait(x_position_);
  }
}


int main(int argc, char **argv){
  ros::init (argc, argv, "move_arm_pose_goal_test");
  ros::NodeHandle node;
  //actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_left_arm",true);

  ActuateMechanismwIK actMechwIK;

  //subscribers
  ros::Subscriber sub_ee_force = node.subscribe("/ee_force", 1, &ActuateMechanismwIK::ee_force_callback, &actMechwIK);

  //actMech.move_arm_to_pose(0.70);
  ros::spin();

  return 0;
}
