/* Auto-generated by genmsg_cpp for file /home/barragan/pr2_repo/ros_workspace/slide_box/srv/robot_actuate_object.srv */
#ifndef SLIDE_BOX_SERVICE_ROBOT_ACTUATE_OBJECT_H
#define SLIDE_BOX_SERVICE_ROBOT_ACTUATE_OBJECT_H
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

#include "ros/service_traits.h"




namespace slide_box
{
template <class ContainerAllocator>
struct robot_actuate_objectRequest_ {
  typedef robot_actuate_objectRequest_<ContainerAllocator> Type;

  robot_actuate_objectRequest_()
  : action()
  {
  }

  robot_actuate_objectRequest_(const ContainerAllocator& _alloc)
  : action(_alloc)
  {
  }

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _action_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  action;


  typedef boost::shared_ptr< ::slide_box::robot_actuate_objectRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::slide_box::robot_actuate_objectRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct robot_actuate_objectRequest
typedef  ::slide_box::robot_actuate_objectRequest_<std::allocator<void> > robot_actuate_objectRequest;

typedef boost::shared_ptr< ::slide_box::robot_actuate_objectRequest> robot_actuate_objectRequestPtr;
typedef boost::shared_ptr< ::slide_box::robot_actuate_objectRequest const> robot_actuate_objectRequestConstPtr;


template <class ContainerAllocator>
struct robot_actuate_objectResponse_ {
  typedef robot_actuate_objectResponse_<ContainerAllocator> Type;

  robot_actuate_objectResponse_()
  : obs()
  {
  }

  robot_actuate_objectResponse_(const ContainerAllocator& _alloc)
  : obs(_alloc)
  {
  }

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _obs_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  obs;


  typedef boost::shared_ptr< ::slide_box::robot_actuate_objectResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::slide_box::robot_actuate_objectResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct robot_actuate_objectResponse
typedef  ::slide_box::robot_actuate_objectResponse_<std::allocator<void> > robot_actuate_objectResponse;

typedef boost::shared_ptr< ::slide_box::robot_actuate_objectResponse> robot_actuate_objectResponsePtr;
typedef boost::shared_ptr< ::slide_box::robot_actuate_objectResponse const> robot_actuate_objectResponseConstPtr;

struct robot_actuate_object
{

typedef robot_actuate_objectRequest Request;
typedef robot_actuate_objectResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct robot_actuate_object
} // namespace slide_box

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::slide_box::robot_actuate_objectRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::slide_box::robot_actuate_objectRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::slide_box::robot_actuate_objectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "79f44d272f2ebe04451185b0dea57684";
  }

  static const char* value(const  ::slide_box::robot_actuate_objectRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x79f44d272f2ebe04ULL;
  static const uint64_t static_value2 = 0x451185b0dea57684ULL;
};

template<class ContainerAllocator>
struct DataType< ::slide_box::robot_actuate_objectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "slide_box/robot_actuate_objectRequest";
  }

  static const char* value(const  ::slide_box::robot_actuate_objectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::slide_box::robot_actuate_objectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64[] action\n\
\n\
";
  }

  static const char* value(const  ::slide_box::robot_actuate_objectRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::slide_box::robot_actuate_objectResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::slide_box::robot_actuate_objectResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::slide_box::robot_actuate_objectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8dac64abe4f5eba5d19614ccef1fe66c";
  }

  static const char* value(const  ::slide_box::robot_actuate_objectResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8dac64abe4f5eba5ULL;
  static const uint64_t static_value2 = 0xd19614ccef1fe66cULL;
};

template<class ContainerAllocator>
struct DataType< ::slide_box::robot_actuate_objectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "slide_box/robot_actuate_objectResponse";
  }

  static const char* value(const  ::slide_box::robot_actuate_objectResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::slide_box::robot_actuate_objectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64[] obs\n\
\n\
\n\
";
  }

  static const char* value(const  ::slide_box::robot_actuate_objectResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::slide_box::robot_actuate_objectRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.action);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct robot_actuate_objectRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::slide_box::robot_actuate_objectResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.obs);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct robot_actuate_objectResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<slide_box::robot_actuate_object> {
  static const char* value() 
  {
    return "b7dc8304fadbcfaba89fb93716c57805";
  }

  static const char* value(const slide_box::robot_actuate_object&) { return value(); } 
};

template<>
struct DataType<slide_box::robot_actuate_object> {
  static const char* value() 
  {
    return "slide_box/robot_actuate_object";
  }

  static const char* value(const slide_box::robot_actuate_object&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<slide_box::robot_actuate_objectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b7dc8304fadbcfaba89fb93716c57805";
  }

  static const char* value(const slide_box::robot_actuate_objectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<slide_box::robot_actuate_objectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "slide_box/robot_actuate_object";
  }

  static const char* value(const slide_box::robot_actuate_objectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<slide_box::robot_actuate_objectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b7dc8304fadbcfaba89fb93716c57805";
  }

  static const char* value(const slide_box::robot_actuate_objectResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<slide_box::robot_actuate_objectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "slide_box/robot_actuate_object";
  }

  static const char* value(const slide_box::robot_actuate_objectResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // SLIDE_BOX_SERVICE_ROBOT_ACTUATE_OBJECT_H

