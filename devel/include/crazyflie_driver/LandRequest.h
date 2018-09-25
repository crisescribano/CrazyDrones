// Generated by gencpp from file crazyflie_driver/LandRequest.msg
// DO NOT EDIT!


#ifndef CRAZYFLIE_DRIVER_MESSAGE_LANDREQUEST_H
#define CRAZYFLIE_DRIVER_MESSAGE_LANDREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace crazyflie_driver
{
template <class ContainerAllocator>
struct LandRequest_
{
  typedef LandRequest_<ContainerAllocator> Type;

  LandRequest_()
    : groupMask(0)
    , height(0.0)
    , duration()  {
    }
  LandRequest_(const ContainerAllocator& _alloc)
    : groupMask(0)
    , height(0.0)
    , duration()  {
  (void)_alloc;
    }



   typedef uint8_t _groupMask_type;
  _groupMask_type groupMask;

   typedef float _height_type;
  _height_type height;

   typedef ros::Duration _duration_type;
  _duration_type duration;





  typedef boost::shared_ptr< ::crazyflie_driver::LandRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::crazyflie_driver::LandRequest_<ContainerAllocator> const> ConstPtr;

}; // struct LandRequest_

typedef ::crazyflie_driver::LandRequest_<std::allocator<void> > LandRequest;

typedef boost::shared_ptr< ::crazyflie_driver::LandRequest > LandRequestPtr;
typedef boost::shared_ptr< ::crazyflie_driver::LandRequest const> LandRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::crazyflie_driver::LandRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::crazyflie_driver::LandRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace crazyflie_driver

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'crazyflie_driver': ['/home/ezgondi/CrazyDrones/src/crazyflie_ros/crazyflie_driver/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_driver::LandRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_driver::LandRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_driver::LandRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_driver::LandRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_driver::LandRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_driver::LandRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::crazyflie_driver::LandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b665b6c83a196e4774268cc26329b159";
  }

  static const char* value(const ::crazyflie_driver::LandRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb665b6c83a196e47ULL;
  static const uint64_t static_value2 = 0x74268cc26329b159ULL;
};

template<class ContainerAllocator>
struct DataType< ::crazyflie_driver::LandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "crazyflie_driver/LandRequest";
  }

  static const char* value(const ::crazyflie_driver::LandRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::crazyflie_driver::LandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 groupMask\n\
float32 height\n\
duration duration\n\
";
  }

  static const char* value(const ::crazyflie_driver::LandRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::crazyflie_driver::LandRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.groupMask);
      stream.next(m.height);
      stream.next(m.duration);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LandRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::crazyflie_driver::LandRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::crazyflie_driver::LandRequest_<ContainerAllocator>& v)
  {
    s << indent << "groupMask: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.groupMask);
    s << indent << "height: ";
    Printer<float>::stream(s, indent + "  ", v.height);
    s << indent << "duration: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.duration);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRAZYFLIE_DRIVER_MESSAGE_LANDREQUEST_H
