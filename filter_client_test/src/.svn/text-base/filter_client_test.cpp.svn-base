#include "ros/ros.h"
#include "slide_box/robot_actuate_object.h"
#include <cstdlib>

#include <typeinfo>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "filter_client_test");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<slide_box::robot_actuate_object>("robot_actuate_object");
  slide_box::robot_actuate_object srv;
  srv.request.action.push_back(0.70);
  srv.request.action.push_back(0.10);  
  srv.request.action.push_back(0.00);

  if (client.call(srv))
  {
    //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    std::cout << "observation:" << std::endl;
    for (size_t i = 0; i < srv.response.obs.size(); i++){
      std::cout << srv.response.obs[i] << ",";
    }
    std::cout << std::endl;
    std::cout<<"x is of type: "<<typeid(srv.response.obs[0]).name()<<std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service robot_actuate_mechanism");
    return 1;
  }

  return 0;
}
