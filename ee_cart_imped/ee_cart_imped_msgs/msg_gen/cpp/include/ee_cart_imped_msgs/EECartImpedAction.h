/* Auto-generated by genmsg_cpp for file /home/barragan/pr2_repo/ros_workspace/ee_cart_imped/ee_cart_imped_msgs/msg/EECartImpedAction.msg */
#ifndef EE_CART_IMPED_MSGS_MESSAGE_EECARTIMPEDACTION_H
#define EE_CART_IMPED_MSGS_MESSAGE_EECARTIMPEDACTION_H
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

#include "ee_cart_imped_msgs/EECartImpedActionGoal.h"
#include "ee_cart_imped_msgs/EECartImpedActionResult.h"
#include "ee_cart_imped_msgs/EECartImpedActionFeedback.h"

namespace ee_cart_imped_msgs
{
template <class ContainerAllocator>
struct EECartImpedAction_ {
  typedef EECartImpedAction_<ContainerAllocator> Type;

  EECartImpedAction_()
  : action_goal()
  , action_result()
  , action_feedback()
  {
  }

  EECartImpedAction_(const ContainerAllocator& _alloc)
  : action_goal(_alloc)
  , action_result(_alloc)
  , action_feedback(_alloc)
  {
  }

  typedef  ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator>  _action_goal_type;
   ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator>  action_goal;

  typedef  ::ee_cart_imped_msgs::EECartImpedActionResult_<ContainerAllocator>  _action_result_type;
   ::ee_cart_imped_msgs::EECartImpedActionResult_<ContainerAllocator>  action_result;

  typedef  ::ee_cart_imped_msgs::EECartImpedActionFeedback_<ContainerAllocator>  _action_feedback_type;
   ::ee_cart_imped_msgs::EECartImpedActionFeedback_<ContainerAllocator>  action_feedback;


  typedef boost::shared_ptr< ::ee_cart_imped_msgs::EECartImpedAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ee_cart_imped_msgs::EECartImpedAction_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct EECartImpedAction
typedef  ::ee_cart_imped_msgs::EECartImpedAction_<std::allocator<void> > EECartImpedAction;

typedef boost::shared_ptr< ::ee_cart_imped_msgs::EECartImpedAction> EECartImpedActionPtr;
typedef boost::shared_ptr< ::ee_cart_imped_msgs::EECartImpedAction const> EECartImpedActionConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::ee_cart_imped_msgs::EECartImpedAction_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::ee_cart_imped_msgs::EECartImpedAction_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace ee_cart_imped_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::ee_cart_imped_msgs::EECartImpedAction_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::ee_cart_imped_msgs::EECartImpedAction_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::ee_cart_imped_msgs::EECartImpedAction_<ContainerAllocator> > {
  static const char* value() 
  {
    return "45dcb843c9d6fcf22a742d509b64dac6";
  }

  static const char* value(const  ::ee_cart_imped_msgs::EECartImpedAction_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x45dcb843c9d6fcf2ULL;
  static const uint64_t static_value2 = 0x2a742d509b64dac6ULL;
};

template<class ContainerAllocator>
struct DataType< ::ee_cart_imped_msgs::EECartImpedAction_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ee_cart_imped_msgs/EECartImpedAction";
  }

  static const char* value(const  ::ee_cart_imped_msgs::EECartImpedAction_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::ee_cart_imped_msgs::EECartImpedAction_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
EECartImpedActionGoal action_goal\n\
EECartImpedActionResult action_result\n\
EECartImpedActionFeedback action_feedback\n\
\n\
================================================================================\n\
MSG: ee_cart_imped_msgs/EECartImpedActionGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
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
================================================================================\n\
MSG: ee_cart_imped_msgs/EECartImpedActionResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
EECartImpedResult result\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalStatus\n\
GoalID goal_id\n\
uint8 status\n\
uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n\
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n\
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n\
                            #   and has since completed its execution (Terminal State)\n\
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n\
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n\
                            #    to some failure (Terminal State)\n\
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n\
                            #    because the goal was unattainable or invalid (Terminal State)\n\
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n\
                            #    and has not yet completed execution\n\
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n\
                            #    but the action server has not yet confirmed that the goal is canceled\n\
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n\
                            #    and was successfully cancelled (Terminal State)\n\
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n\
                            #    sent over the wire by an action server\n\
\n\
#Allow for the user to associate a string with GoalStatus for debugging\n\
string text\n\
\n\
\n\
================================================================================\n\
MSG: ee_cart_imped_msgs/EECartImpedResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
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
MSG: ee_cart_imped_msgs/EECartImpedActionFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
EECartImpedFeedback feedback\n\
\n\
================================================================================\n\
MSG: ee_cart_imped_msgs/EECartImpedFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#feedback\n\
#current pose and squared error in force \n\
#and the running time of this goal so far\n\
Header header\n\
ee_cart_imped_msgs/StiffPoint[] goal\n\
ee_cart_imped_msgs/StiffPoint initial_point\n\
ee_cart_imped_msgs/StiffPoint desired\n\
ee_cart_imped_msgs/StiffPoint actual_pose\n\
geometry_msgs/Wrench actual_force\n\
float64 effort_sq_error\n\
float64[] requested_joint_efforts\n\
float64[] actual_joint_efforts\n\
duration running_time\n\
\n\
\n\
\n\
";
  }

  static const char* value(const  ::ee_cart_imped_msgs::EECartImpedAction_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::ee_cart_imped_msgs::EECartImpedAction_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.action_goal);
    stream.next(m.action_result);
    stream.next(m.action_feedback);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct EECartImpedAction_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ee_cart_imped_msgs::EECartImpedAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::ee_cart_imped_msgs::EECartImpedAction_<ContainerAllocator> & v) 
  {
    s << indent << "action_goal: ";
s << std::endl;
    Printer< ::ee_cart_imped_msgs::EECartImpedActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
s << std::endl;
    Printer< ::ee_cart_imped_msgs::EECartImpedActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
s << std::endl;
    Printer< ::ee_cart_imped_msgs::EECartImpedActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};


} // namespace message_operations
} // namespace ros

#endif // EE_CART_IMPED_MSGS_MESSAGE_EECARTIMPEDACTION_H

