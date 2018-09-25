// Generated by gencpp from file crazyflie_driver/sendPacketRequest.msg
// DO NOT EDIT!


#ifndef CRAZYFLIE_DRIVER_MESSAGE_SENDPACKETREQUEST_H
#define CRAZYFLIE_DRIVER_MESSAGE_SENDPACKETREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <crazyflie_driver/crtpPacket.h>

namespace crazyflie_driver
{
template <class ContainerAllocator>
struct sendPacketRequest_
{
  typedef sendPacketRequest_<ContainerAllocator> Type;

  sendPacketRequest_()
    : packet()  {
    }
  sendPacketRequest_(const ContainerAllocator& _alloc)
    : packet(_alloc)  {
  (void)_alloc;
    }



   typedef  ::crazyflie_driver::crtpPacket_<ContainerAllocator>  _packet_type;
  _packet_type packet;





  typedef boost::shared_ptr< ::crazyflie_driver::sendPacketRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::crazyflie_driver::sendPacketRequest_<ContainerAllocator> const> ConstPtr;

}; // struct sendPacketRequest_

typedef ::crazyflie_driver::sendPacketRequest_<std::allocator<void> > sendPacketRequest;

typedef boost::shared_ptr< ::crazyflie_driver::sendPacketRequest > sendPacketRequestPtr;
typedef boost::shared_ptr< ::crazyflie_driver::sendPacketRequest const> sendPacketRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::crazyflie_driver::sendPacketRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::crazyflie_driver::sendPacketRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::crazyflie_driver::sendPacketRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_driver::sendPacketRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_driver::sendPacketRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_driver::sendPacketRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_driver::sendPacketRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_driver::sendPacketRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::crazyflie_driver::sendPacketRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e3f946ce194c915d34d0f5ddedef0de7";
  }

  static const char* value(const ::crazyflie_driver::sendPacketRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe3f946ce194c915dULL;
  static const uint64_t static_value2 = 0x34d0f5ddedef0de7ULL;
};

template<class ContainerAllocator>
struct DataType< ::crazyflie_driver::sendPacketRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "crazyflie_driver/sendPacketRequest";
  }

  static const char* value(const ::crazyflie_driver::sendPacketRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::crazyflie_driver::sendPacketRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "crazyflie_driver/crtpPacket packet\n\
\n\
================================================================================\n\
MSG: crazyflie_driver/crtpPacket\n\
uint8 size\n\
uint8 header\n\
uint8[30] data\n\
";
  }

  static const char* value(const ::crazyflie_driver::sendPacketRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::crazyflie_driver::sendPacketRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.packet);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct sendPacketRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::crazyflie_driver::sendPacketRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::crazyflie_driver::sendPacketRequest_<ContainerAllocator>& v)
  {
    s << indent << "packet: ";
    s << std::endl;
    Printer< ::crazyflie_driver::crtpPacket_<ContainerAllocator> >::stream(s, indent + "  ", v.packet);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRAZYFLIE_DRIVER_MESSAGE_SENDPACKETREQUEST_H
