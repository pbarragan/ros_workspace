/* Auto-generated by genmsg_cpp for file /home/barragan/pr2_repo/ros_workspace/ee_cart_imped/ee_cart_imped_msgs/msg/EECartImpedResult.msg */
#ifndef EE_CART_IMPED_MSGS_MESSAGE_EECARTIMPEDRESULT_H
#define EE_CART_IMPED_MSGS_MESSAGE_EECARTIMPEDRESULT_H
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
#include "ee_cart_imped_msgs/StiffPoint.h"
#include "geometry_msgs/Pose.h"

namespace ee_cart_imped_msgs
{
template <class ContainerAllocator>
struct EECartImpedResult_ {
  typedef EECartImpedResult_<ContainerAllocator> Type;

  EECartImpedResult_()
  : header()
  , success(false)
  , desired()
  , actual_pose()
  , effort_sq_error(0.0)
  {
  }

  EECartImpedResult_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , success(false)
  , desired(_alloc)
  , actual_pose(_alloc)
  , effort_sq_error(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef uint8_t _success_type;
  uint8_t success;

  typedef  ::ee_cart_imped_msgs::StiffPoint_<ContainerAllocator>  _desired_type;
   ::ee_cart_imped_msgs::StiffPoint_<ContainerAllocator>  desired;

  typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _actual_pose_type;
   ::geometry_msgs::Pose_<ContainerAllocator>  actual_pose;

  typedef double _effort_sq_error_type;
  double effort_sq_error;


  typedef boost::shared_ptr< ::ee_cart_imped_msgs::EECartImpedResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ee_cart_imped_msgs::EECartImpedResult_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct EECartImpedResult
typedef  ::ee_cart_imped_msgs::EECartImpedResult_<std::allocator<void> > EECartImpedResult;

typedef boost::shared_ptr< ::ee_cart_imped_msgs::EECartImpedResult> EECartImpedResultPtr;
typedef boost::shared_ptr< ::ee_cart_imped_msgs::EECartImpedResult const> EECartImpedResultConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::ee_cart_imped_msgs::EECartImpedResult_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::ee_cart_imped_msgs::EECartImpedResult_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace ee_cart_imped_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::ee_cart_imped_msgs::EECartImpedResult_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::ee_cart_imped_msgs::EECartImpedResult_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::ee_cart_imped_msgs::EECartImpedResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c1f32db3be6f680c9824a668ba099471";
  }

  static const char* value(const  ::ee_cart_imped_msgs::EECartImpedResult_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc1f32db3be6f680cULL;
  static const uint64_t static_value2 = 0x9824a668ba099471ULL;
};

template<class ContainerAllocator>
struct DataType< ::ee_cart_imped_msgs::EECartImpedResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ee_cart_imped_msgs/EECartImpedResult";
  }

  static const char* value(const  ::ee_cart_imped_msgs::EECartImpedResult_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::ee_cart_imped_msgs::EECartImpedResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#result definition\n\
#whether it was successful\n\
#the pose and force we ended with\n\
Header header\n\
bool success\n\
ee_cart_imped_msgs/StiffPoint desired\n\
geometry_msgs/Pose actual_pose\n\
float64 effort_sq_error\n\
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
geometry_msgs/Quaternion forceDirection\n\
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

  static const char* value(const  ::ee_cart_imped_msgs::EECartImpedResult_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::ee_cart_imped_msgs::EECartImpedResult_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::ee_cart_imped_msgs::EECartImpedResult_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::ee_cart_imped_msgs::EECartImpedResult_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.success);
    stream.next(m.desired);
    stream.next(m.actual_pose);
    stream.next(m.effort_sq_error);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct EECartImpedResult_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ee_cart_imped_msgs::EECartImpedResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::ee_cart_imped_msgs::EECartImpedResult_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "desired: ";
s << std::endl;
    Printer< ::ee_cart_imped_msgs::StiffPoint_<ContainerAllocator> >::stream(s, indent + "  ", v.desired);
    s << indent << "actual_pose: ";
s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.actual_pose);
    s << indent << "effort_sq_error: ";
    Printer<double>::stream(s, indent + "  ", v.effort_sq_error);
  }
};


} // namespace message_operations
} // namespace ros

#endif // EE_CART_IMPED_MSGS_MESSAGE_EECARTIMPEDRESULT_H

