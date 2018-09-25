// Generated by gencpp from file crazyflie_driver/Hover.msg
// DO NOT EDIT!


#ifndef CRAZYFLIE_DRIVER_MESSAGE_HOVER_H
#define CRAZYFLIE_DRIVER_MESSAGE_HOVER_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace crazyflie_driver
{
template <class ContainerAllocator>
struct Hover_
{
  typedef Hover_<ContainerAllocator> Type;

  Hover_()
    : header()
    , vx(0.0)
    , vy(0.0)
    , yawrate(0.0)
    , zDistance(0.0)  {
    }
  Hover_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , vx(0.0)
    , vy(0.0)
    , yawrate(0.0)
    , zDistance(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _vx_type;
  _vx_type vx;

   typedef float _vy_type;
  _vy_type vy;

   typedef float _yawrate_type;
  _yawrate_type yawrate;

   typedef float _zDistance_type;
  _zDistance_type zDistance;





  typedef boost::shared_ptr< ::crazyflie_driver::Hover_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::crazyflie_driver::Hover_<ContainerAllocator> const> ConstPtr;

}; // struct Hover_

typedef ::crazyflie_driver::Hover_<std::allocator<void> > Hover;

typedef boost::shared_ptr< ::crazyflie_driver::Hover > HoverPtr;
typedef boost::shared_ptr< ::crazyflie_driver::Hover const> HoverConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::crazyflie_driver::Hover_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::crazyflie_driver::Hover_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace crazyflie_driver

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'crazyflie_driver': ['/home/ezgondi/CrazyDrones/src/crazyflie_ros/crazyflie_driver/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_driver::Hover_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_driver::Hover_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_driver::Hover_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_driver::Hover_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_driver::Hover_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_driver::Hover_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::crazyflie_driver::Hover_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3d4f46e9fa4dcfdc0516eb081fef369e";
  }

  static const char* value(const ::crazyflie_driver::Hover_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3d4f46e9fa4dcfdcULL;
  static const uint64_t static_value2 = 0x0516eb081fef369eULL;
};

template<class ContainerAllocator>
struct DataType< ::crazyflie_driver::Hover_<ContainerAllocator> >
{
  static const char* value()
  {
    return "crazyflie_driver/Hover";
  }

  static const char* value(const ::crazyflie_driver::Hover_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::crazyflie_driver::Hover_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
float32 vx\n\
float32 vy\n\
float32 yawrate\n\
float32 zDistance\n\
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
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::crazyflie_driver::Hover_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::crazyflie_driver::Hover_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.vx);
      stream.next(m.vy);
      stream.next(m.yawrate);
      stream.next(m.zDistance);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Hover_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::crazyflie_driver::Hover_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::crazyflie_driver::Hover_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "vx: ";
    Printer<float>::stream(s, indent + "  ", v.vx);
    s << indent << "vy: ";
    Printer<float>::stream(s, indent + "  ", v.vy);
    s << indent << "yawrate: ";
    Printer<float>::stream(s, indent + "  ", v.yawrate);
    s << indent << "zDistance: ";
    Printer<float>::stream(s, indent + "  ", v.zDistance);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRAZYFLIE_DRIVER_MESSAGE_HOVER_H
