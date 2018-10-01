// Generated by gencpp from file qualisys/Markers.msg
// DO NOT EDIT!


#ifndef QUALISYS_MESSAGE_MARKERS_H
#define QUALISYS_MESSAGE_MARKERS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <qualisys/Marker.h>

namespace qualisys
{
template <class ContainerAllocator>
struct Markers_
{
  typedef Markers_<ContainerAllocator> Type;

  Markers_()
    : header()
    , markers()  {
    }
  Markers_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , markers(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::qualisys::Marker_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::qualisys::Marker_<ContainerAllocator> >::other >  _markers_type;
  _markers_type markers;





  typedef boost::shared_ptr< ::qualisys::Markers_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::qualisys::Markers_<ContainerAllocator> const> ConstPtr;

}; // struct Markers_

typedef ::qualisys::Markers_<std::allocator<void> > Markers;

typedef boost::shared_ptr< ::qualisys::Markers > MarkersPtr;
typedef boost::shared_ptr< ::qualisys::Markers const> MarkersConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::qualisys::Markers_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::qualisys::Markers_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace qualisys

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'qualisys': ['/home/cristinaescribano/CrazyDrones/src/qualisys/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::qualisys::Markers_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::qualisys::Markers_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qualisys::Markers_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qualisys::Markers_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qualisys::Markers_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qualisys::Markers_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::qualisys::Markers_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6ba87eb5111a9609174c0acf8542f67d";
  }

  static const char* value(const ::qualisys::Markers_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6ba87eb5111a9609ULL;
  static const uint64_t static_value2 = 0x174c0acf8542f67dULL;
};

template<class ContainerAllocator>
struct DataType< ::qualisys::Markers_<ContainerAllocator> >
{
  static const char* value()
  {
    return "qualisys/Markers";
  }

  static const char* value(const ::qualisys::Markers_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::qualisys::Markers_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
qualisys/Marker[] markers\n\
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
\n\
================================================================================\n\
MSG: qualisys/Marker\n\
string name\n\
string subject_name\n\
geometry_msgs/Point position\n\
bool occluded\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::qualisys::Markers_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::qualisys::Markers_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.markers);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Markers_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::qualisys::Markers_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::qualisys::Markers_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "markers[]" << std::endl;
    for (size_t i = 0; i < v.markers.size(); ++i)
    {
      s << indent << "  markers[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::qualisys::Marker_<ContainerAllocator> >::stream(s, indent + "    ", v.markers[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // QUALISYS_MESSAGE_MARKERS_H