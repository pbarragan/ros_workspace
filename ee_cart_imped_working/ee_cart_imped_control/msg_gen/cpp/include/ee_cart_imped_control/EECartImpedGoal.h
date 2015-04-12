/* Auto-generated by genmsg_cpp for file /u/barragan/ros_packages/mit-ros-pkg/trunk/ee_cart_imped/ee_cart_imped_control/msg/EECartImpedGoal.msg */
#ifndef EE_CART_IMPED_CONTROL_MESSAGE_EECARTIMPEDGOAL_H
#define EE_CART_IMPED_CONTROL_MESSAGE_EECARTIMPEDGOAL_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "std_msgs/Header.h"
#include "ee_cart_imped_control/StiffPoint.h"

namespace ee_cart_imped_control
{
template <class ContainerAllocator>
struct EECartImpedGoal_ : public ros::Message
{
  typedef EECartImpedGoal_<ContainerAllocator> Type;

  EECartImpedGoal_()
  : header()
  , trajectory()
  {
  }

  EECartImpedGoal_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , trajectory(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector< ::ee_cart_imped_control::StiffPoint_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::ee_cart_imped_control::StiffPoint_<ContainerAllocator> >::other >  _trajectory_type;
  std::vector< ::ee_cart_imped_control::StiffPoint_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::ee_cart_imped_control::StiffPoint_<ContainerAllocator> >::other >  trajectory;


  ROS_DEPRECATED uint32_t get_trajectory_size() const { return (uint32_t)trajectory.size(); }
  ROS_DEPRECATED void set_trajectory_size(uint32_t size) { trajectory.resize((size_t)size); }
  ROS_DEPRECATED void get_trajectory_vec(std::vector< ::ee_cart_imped_control::StiffPoint_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::ee_cart_imped_control::StiffPoint_<ContainerAllocator> >::other > & vec) const { vec = this->trajectory; }
  ROS_DEPRECATED void set_trajectory_vec(const std::vector< ::ee_cart_imped_control::StiffPoint_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::ee_cart_imped_control::StiffPoint_<ContainerAllocator> >::other > & vec) { this->trajectory = vec; }
private:
  static const char* __s_getDataType_() { return "ee_cart_imped_control/EECartImpedGoal"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "0b1ff60300ab63f83c2158e930c32bb2"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#goal definition\n\
Header header\n\
ee_cart_imped_control/StiffPoint[] trajectory\n\
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
MSG: ee_cart_imped_control/StiffPoint\n\
Header header\n\
geometry_msgs/Pose pose\n\
geometry_msgs/Wrench wrench_or_stiffness\n\
bool isForceX\n\
bool isForceY\n\
bool isForceZ\n\
bool isTorqueX\n\
bool isTorqueY\n\
bool isTorqueZ\n\
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
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, header);
    ros::serialization::serialize(stream, trajectory);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, trajectory);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(trajectory);
    return size;
  }

  typedef boost::shared_ptr< ::ee_cart_imped_control::EECartImpedGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ee_cart_imped_control::EECartImpedGoal_<ContainerAllocator>  const> ConstPtr;
}; // struct EECartImpedGoal
typedef  ::ee_cart_imped_control::EECartImpedGoal_<std::allocator<void> > EECartImpedGoal;

typedef boost::shared_ptr< ::ee_cart_imped_control::EECartImpedGoal> EECartImpedGoalPtr;
typedef boost::shared_ptr< ::ee_cart_imped_control::EECartImpedGoal const> EECartImpedGoalConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::ee_cart_imped_control::EECartImpedGoal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::ee_cart_imped_control::EECartImpedGoal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace ee_cart_imped_control

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::ee_cart_imped_control::EECartImpedGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0b1ff60300ab63f83c2158e930c32bb2";
  }

  static const char* value(const  ::ee_cart_imped_control::EECartImpedGoal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x0b1ff60300ab63f8ULL;
  static const uint64_t static_value2 = 0x3c2158e930c32bb2ULL;
};

template<class ContainerAllocator>
struct DataType< ::ee_cart_imped_control::EECartImpedGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ee_cart_imped_control/EECartImpedGoal";
  }

  static const char* value(const  ::ee_cart_imped_control::EECartImpedGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::ee_cart_imped_control::EECartImpedGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#goal definition\n\
Header header\n\
ee_cart_imped_control/StiffPoint[] trajectory\n\
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
MSG: ee_cart_imped_control/StiffPoint\n\
Header header\n\
geometry_msgs/Pose pose\n\
geometry_msgs/Wrench wrench_or_stiffness\n\
bool isForceX\n\
bool isForceY\n\
bool isForceZ\n\
bool isTorqueX\n\
bool isTorqueY\n\
bool isTorqueZ\n\
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

  static const char* value(const  ::ee_cart_imped_control::EECartImpedGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::ee_cart_imped_control::EECartImpedGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::ee_cart_imped_control::EECartImpedGoal_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::ee_cart_imped_control::EECartImpedGoal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.trajectory);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct EECartImpedGoal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ee_cart_imped_control::EECartImpedGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::ee_cart_imped_control::EECartImpedGoal_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "trajectory[]" << std::endl;
    for (size_t i = 0; i < v.trajectory.size(); ++i)
    {
      s << indent << "  trajectory[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ee_cart_imped_control::StiffPoint_<ContainerAllocator> >::stream(s, indent + "    ", v.trajectory[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // EE_CART_IMPED_CONTROL_MESSAGE_EECARTIMPEDGOAL_H
