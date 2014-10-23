#include <math.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <ee_impulse_try1/Cylinder.h>
#include <ee_impulse_try1/Cylinders.h>

#include <vector>

using namespace ee_impulse_try1;
using namespace std;

double markerX[] = {0, 0, 0};
double markerY[] = {0, 0, 0};
double markerZ[] = {0.69, 0.69, 0.69};
double markerR[] = {0.1, 0.1, 0.1};
string frame_id = "";
int num_cyl = 0;

//ros::Publisher marker_pub;
//ros::Rate r(1);


/* handle an /cylinders message */
void cylinders_callback(ee_impulse_try1::Cylinders msg)
{
  ROS_INFO("Got /cylinders msg\n");

  frame_id = msg.header.frame_id;
  num_cyl = msg.cylinders.size();

  for (int i = 0; i < num_cyl; i++){
    markerX[i] = msg.cylinders[i].x;
    markerY[i] = msg.cylinders[i].y;
    markerR[i] = msg.cylinders[i].r;
    }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);

  // publishers
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // subscribers
  ros::Subscriber Cylinders = n.subscribe("/cylinders", 1, cylinders_callback);

  // Set our shape type to be a cylinder
  uint32_t shape = visualization_msgs::Marker::CYLINDER;
  
  //ROS_INFO("The init ran\n");

  while (ros::ok())
  {
    //ROS_INFO("I'm inside the while loop \n");
    ROS_INFO("number of cylinders is %d \n", num_cyl);

    for (int i = 0; i < num_cyl; i++){
      //ROS_INFO("I'm inside the for loop \n");
      visualization_msgs::Marker marker;
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = frame_id;
      marker.header.stamp = ros::Time::now();
      
      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "basic_shapes";
      marker.id = i;
      
      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = shape;
      
      // Set the marker action.  Options are ADD and DELETE
      marker.action = visualization_msgs::Marker::ADD;
      
      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = markerX[i];
      marker.pose.position.y = markerY[i];
      marker.pose.position.z = markerZ[i];
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 2*markerR[i];
      marker.scale.y = 2*markerR[i];
      marker.scale.z = 0.25; //markerR[i];
      
      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 221.0f;
      marker.color.g = 160.0f;
      marker.color.b = 221.0f;
      marker.color.a = 1.0;
      
      marker.lifetime = ros::Duration();
      
      // Publish the marker
      marker_pub.publish(marker);
      ROS_INFO("Just published a marker\n");

    }
    r.sleep();
    ros::spinOnce();
  }
  
}
