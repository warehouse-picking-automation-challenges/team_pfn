// Generated by gencpp from file apc2016/CalibrationUpdateRequest.msg
// DO NOT EDIT!


#ifndef APC2016_MESSAGE_CALIBRATIONUPDATEREQUEST_H
#define APC2016_MESSAGE_CALIBRATIONUPDATEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>

namespace apc2016
{
template <class ContainerAllocator>
struct CalibrationUpdateRequest_
{
  typedef CalibrationUpdateRequest_<ContainerAllocator> Type;

  CalibrationUpdateRequest_()
    : upper_left()
    , upper_right()  {
    }
  CalibrationUpdateRequest_(const ContainerAllocator& _alloc)
    : upper_left(_alloc)
    , upper_right(_alloc)  {
    }



   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _upper_left_type;
  _upper_left_type upper_left;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _upper_right_type;
  _upper_right_type upper_right;




  typedef boost::shared_ptr< ::apc2016::CalibrationUpdateRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::apc2016::CalibrationUpdateRequest_<ContainerAllocator> const> ConstPtr;

}; // struct CalibrationUpdateRequest_

typedef ::apc2016::CalibrationUpdateRequest_<std::allocator<void> > CalibrationUpdateRequest;

typedef boost::shared_ptr< ::apc2016::CalibrationUpdateRequest > CalibrationUpdateRequestPtr;
typedef boost::shared_ptr< ::apc2016::CalibrationUpdateRequest const> CalibrationUpdateRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::apc2016::CalibrationUpdateRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::apc2016::CalibrationUpdateRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace apc2016

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'apc2016': ['/home/kamiya/data/apc/ros/src/apc2016/msg', '/home/kamiya/data/apc/ros/devel/share/apc2016/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::apc2016::CalibrationUpdateRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::apc2016::CalibrationUpdateRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::apc2016::CalibrationUpdateRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::apc2016::CalibrationUpdateRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::apc2016::CalibrationUpdateRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::apc2016::CalibrationUpdateRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::apc2016::CalibrationUpdateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cf7a797601ef4127a164d12651aa00b6";
  }

  static const char* value(const ::apc2016::CalibrationUpdateRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcf7a797601ef4127ULL;
  static const uint64_t static_value2 = 0xa164d12651aa00b6ULL;
};

template<class ContainerAllocator>
struct DataType< ::apc2016::CalibrationUpdateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "apc2016/CalibrationUpdateRequest";
  }

  static const char* value(const ::apc2016::CalibrationUpdateRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::apc2016::CalibrationUpdateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point upper_left\n\
geometry_msgs/Point upper_right\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::apc2016::CalibrationUpdateRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::apc2016::CalibrationUpdateRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.upper_left);
      stream.next(m.upper_right);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct CalibrationUpdateRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::apc2016::CalibrationUpdateRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::apc2016::CalibrationUpdateRequest_<ContainerAllocator>& v)
  {
    s << indent << "upper_left: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.upper_left);
    s << indent << "upper_right: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.upper_right);
  }
};

} // namespace message_operations
} // namespace ros

#endif // APC2016_MESSAGE_CALIBRATIONUPDATEREQUEST_H
