// Generated by gencpp from file geometry_msgs/PolygonStamped.msg
// DO NOT EDIT!


#ifndef GEOMETRY_MSGS_MESSAGE_POLYGONSTAMPED_H
#define GEOMETRY_MSGS_MESSAGE_POLYGONSTAMPED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Polygon.h>

namespace geometry_msgs
{
template <class ContainerAllocator>
struct PolygonStamped_
{
  typedef PolygonStamped_<ContainerAllocator> Type;

  PolygonStamped_()
    : header()
    , polygon()  {
    }
  PolygonStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , polygon(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Polygon_<ContainerAllocator>  _polygon_type;
  _polygon_type polygon;




  typedef std::shared_ptr< ::geometry_msgs::PolygonStamped_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::geometry_msgs::PolygonStamped_<ContainerAllocator> const> ConstPtr;

}; // struct PolygonStamped_

typedef ::geometry_msgs::PolygonStamped_<std::allocator<void> > PolygonStamped;

typedef std::shared_ptr< ::geometry_msgs::PolygonStamped > PolygonStampedPtr;
typedef std::shared_ptr< ::geometry_msgs::PolygonStamped const> PolygonStampedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::geometry_msgs::PolygonStamped_<ContainerAllocator> & v)
{
rs2rosinternal::message_operations::Printer< ::geometry_msgs::PolygonStamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace geometry_msgs

namespace rs2rosinternal
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/tmp/binarydeb/ros-kinetic-geometry-msgs-1.12.5/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::geometry_msgs::PolygonStamped_<ContainerAllocator> >
  : std::false_type
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::geometry_msgs::PolygonStamped_<ContainerAllocator> const>
  : std::false_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::geometry_msgs::PolygonStamped_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct IsMessage< ::geometry_msgs::PolygonStamped_<ContainerAllocator> const>
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::geometry_msgs::PolygonStamped_<ContainerAllocator> >
  : std::true_type
  { };

template <class ContainerAllocator>
struct HasHeader< ::geometry_msgs::PolygonStamped_<ContainerAllocator> const>
  : std::true_type
  { };


template<class ContainerAllocator>
struct MD5Sum< ::geometry_msgs::PolygonStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c6be8f7dc3bee7fe9e8d296070f53340";
  }

  static const char* value(const ::geometry_msgs::PolygonStamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc6be8f7dc3bee7feULL;
  static const uint64_t static_value2 = 0x9e8d296070f53340ULL;
};

template<class ContainerAllocator>
struct DataType< ::geometry_msgs::PolygonStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/PolygonStamped";
  }

  static const char* value(const ::geometry_msgs::PolygonStamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::geometry_msgs::PolygonStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This represents a Polygon with reference coordinate frame and timestamp\n\
Header header\n\
Polygon polygon\n\
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
MSG: geometry_msgs/Polygon\n\
#A specification of a polygon where the first and last points are assumed to be connected\n\
Point32[] points\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z\n\
";
  }

  static const char* value(const ::geometry_msgs::PolygonStamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace rs2rosinternal

namespace rs2rosinternal
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::geometry_msgs::PolygonStamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.polygon);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PolygonStamped_

} // namespace serialization
} // namespace rs2rosinternal

namespace rs2rosinternal
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::geometry_msgs::PolygonStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::geometry_msgs::PolygonStamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "polygon: ";
    s << std::endl;
    Printer< ::geometry_msgs::Polygon_<ContainerAllocator> >::stream(s, indent + "  ", v.polygon);
  }
};

} // namespace message_operations
} // namespace rs2rosinternal

#endif // GEOMETRY_MSGS_MESSAGE_POLYGONSTAMPED_H
