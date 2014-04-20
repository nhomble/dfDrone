/* Auto-generated by genmsg_cpp for file /home/turtlebot/groovy_workspace/sandbox/dfDrone/msg/DFDVelocity.msg */
#ifndef DFDRONE_MESSAGE_DFDVELOCITY_H
#define DFDRONE_MESSAGE_DFDVELOCITY_H
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

namespace dfDrone
{
template <class ContainerAllocator>
struct DFDVelocity_ {
  typedef DFDVelocity_<ContainerAllocator> Type;

  DFDVelocity_()
  : header()
  {
  }

  DFDVelocity_(const ContainerAllocator& _alloc)
  : header(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;


  typedef boost::shared_ptr< ::dfDrone::DFDVelocity_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dfDrone::DFDVelocity_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct DFDVelocity
typedef  ::dfDrone::DFDVelocity_<std::allocator<void> > DFDVelocity;

typedef boost::shared_ptr< ::dfDrone::DFDVelocity> DFDVelocityPtr;
typedef boost::shared_ptr< ::dfDrone::DFDVelocity const> DFDVelocityConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::dfDrone::DFDVelocity_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::dfDrone::DFDVelocity_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace dfDrone

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::dfDrone::DFDVelocity_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::dfDrone::DFDVelocity_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::dfDrone::DFDVelocity_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d7be0bb39af8fb9129d5a76e6b63a290";
  }

  static const char* value(const  ::dfDrone::DFDVelocity_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd7be0bb39af8fb91ULL;
  static const uint64_t static_value2 = 0x29d5a76e6b63a290ULL;
};

template<class ContainerAllocator>
struct DataType< ::dfDrone::DFDVelocity_<ContainerAllocator> > {
  static const char* value() 
  {
    return "dfDrone/DFDVelocity";
  }

  static const char* value(const  ::dfDrone::DFDVelocity_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::dfDrone::DFDVelocity_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
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
";
  }

  static const char* value(const  ::dfDrone::DFDVelocity_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::dfDrone::DFDVelocity_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::dfDrone::DFDVelocity_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::dfDrone::DFDVelocity_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct DFDVelocity_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dfDrone::DFDVelocity_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::dfDrone::DFDVelocity_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
  }
};


} // namespace message_operations
} // namespace ros

#endif // DFDRONE_MESSAGE_DFDVELOCITY_H
