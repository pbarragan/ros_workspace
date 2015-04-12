/* Auto-generated by genmsg_cpp for file /home/barragan/ros_packages/mit-ros-pkg/stacks/fuerte/trunk/ee_cart_imped/ee_cart_imped_msgs/msg/EECartImpedActionGoal.msg */
#ifndef EE_CART_IMPED_MSGS_MESSAGE_EECARTIMPEDACTIONGOAL_H
#define EE_CART_IMPED_MSGS_MESSAGE_EECARTIMPEDACTIONGOAL_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "ee_cart_imped_msgs/EECartImpedGoal.h"

namespace ee_cart_imped_msgs
{
template <class ContainerAllocator>
struct EECartImpedActionGoal_ {
  typedef EECartImpedActionGoal_<ContainerAllocator> Type;

  EECartImpedActionGoal_()
  : header()
  , goal_id()
  , goal()
  {
  }

  EECartImpedActionGoal_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , goal_id(_alloc)
  , goal(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef  ::actionlib_msgs::GoalID_<ContainerAllocator>  _goal_id_type;
   ::actionlib_msgs::GoalID_<ContainerAllocator>  goal_id;

  typedef  ::ee_cart_imped_msgs::EECartImpedGoal_<ContainerAllocator>  _goal_type;
   ::ee_cart_imped_msgs::EECartImpedGoal_<ContainerAllocator>  goal;


  typedef boost::shared_ptr< ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct EECartImpedActionGoal
typedef  ::ee_cart_imped_msgs::EECartImpedActionGoal_<std::allocator<void> > EECartImpedActionGoal;

typedef boost::shared_ptr< ::ee_cart_imped_msgs::EECartImpedActionGoal> EECartImpedActionGoalPtr;
typedef boost::shared_ptr< ::ee_cart_imped_msgs::EECartImpedActionGoal const> EECartImpedActionGoalConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace ee_cart_imped_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "09251e687b897c24590030b27467eb9f";
  }

  static const char* value(const  ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x09251e687b897c24ULL;
  static const uint64_t static_value2 = 0x590030b27467eb9fULL;
};

template<class ContainerAllocator>
struct DataType< ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ee_cart_imped_msgs/EECartImpedActionGoal";
  }

  static const char* value(const  ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
EECartImpedGoal goal\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: ee_cart_imped_msgs/EECartImpedGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#goal definition\n\
Header header\n\
ee_cart_imped_msgs/StiffPoint[] trajectory\n\
\n\
================================================================================\n\
MSG: ee_cart_imped_msgs/StiffPoint\n\
Header header\n\
#The pose to achieve in the stiffness directions\n\
geometry_msgs/Pose pose\n\
#Wrench or stiffness for each dimension\n\
geometry_msgs/Wrench wrench_or_stiffness\n\
#The following are True if a force/torque should\n\
#be exerted and False if a stiffness should be used.\n\
bool isForceX\n\
bool isForceY\n\
bool isForceZ\n\
bool isTorqueX\n\
bool isTorqueY\n\
bool isTorqueZ\n\
#The time from the start of the trajectory that this\n\
#point should be achieved.\n\
duration time_from_start\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Wrench\n\
# This represents force in free space, seperated into \n\
# it's linear and angular parts.  \n\
Vector3  force\n\
Vector3  torque\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const  ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.goal_id);
    stream.next(m.goal);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct EECartImpedActionGoal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "goal_id: ";
s << std::endl;
    Printer< ::actionlib_msgs::GoalID_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_id);
    s << indent << "goal: ";
s << std::endl;
    Printer< ::ee_cart_imped_msgs::EECartImpedGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
  }
};


} // namespace message_operations
} // namespace ros

#endif // EE_CART_IMPED_MSGS_MESSAGE_EECARTIMPEDACTIONGOAL_H
