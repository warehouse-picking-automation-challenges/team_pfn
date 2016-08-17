// Generated by gencpp from file apc2016/GenericFeedback.msg
// DO NOT EDIT!


#ifndef APC2016_MESSAGE_GENERICFEEDBACK_H
#define APC2016_MESSAGE_GENERICFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace apc2016
{
template <class ContainerAllocator>
struct GenericFeedback_
{
  typedef GenericFeedback_<ContainerAllocator> Type;

  GenericFeedback_()
    : status()  {
    }
  GenericFeedback_(const ContainerAllocator& _alloc)
    : status(_alloc)  {
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _status_type;
  _status_type status;




  typedef boost::shared_ptr< ::apc2016::GenericFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::apc2016::GenericFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct GenericFeedback_

typedef ::apc2016::GenericFeedback_<std::allocator<void> > GenericFeedback;

typedef boost::shared_ptr< ::apc2016::GenericFeedback > GenericFeedbackPtr;
typedef boost::shared_ptr< ::apc2016::GenericFeedback const> GenericFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::apc2016::GenericFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::apc2016::GenericFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace apc2016

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'apc2016': ['/home/kamiya/data/apc/ros/src/apc2016/msg', '/home/kamiya/data/apc/ros/devel/share/apc2016/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::apc2016::GenericFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::apc2016::GenericFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::apc2016::GenericFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::apc2016::GenericFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::apc2016::GenericFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::apc2016::GenericFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::apc2016::GenericFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4fe5af303955c287688e7347e9b00278";
  }

  static const char* value(const ::apc2016::GenericFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4fe5af303955c287ULL;
  static const uint64_t static_value2 = 0x688e7347e9b00278ULL;
};

template<class ContainerAllocator>
struct DataType< ::apc2016::GenericFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "apc2016/GenericFeedback";
  }

  static const char* value(const ::apc2016::GenericFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::apc2016::GenericFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# Define a feedback message\n\
# status:\n\
# - for linear servo:\n\
#   space-concatenation of\n\
#   - \"0\" or \"1\" (boolean flag for \"is moving\")\n\
#   - decimal number (position in mm)\n\
#   e.g. \"1 12.03\"\n\
# - for rotary servos:\n\
#   space-concatenation of\n\
#   - \"0\" or \"1\" (boolean flag for \"first servo is moving\")\n\
#   - \"0\" or \"1\" (boolean flag for \"second servo is moving\")\n\
#   - decimal number (angle in deg for first servo)\n\
#   - decimal number (angle in deg for second servo)\n\
#   e.g. \"0 1 12.3 90.8\"\n\
string  status\n\
\n\
";
  }

  static const char* value(const ::apc2016::GenericFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::apc2016::GenericFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct GenericFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::apc2016::GenericFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::apc2016::GenericFeedback_<ContainerAllocator>& v)
  {
    s << indent << "status: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // APC2016_MESSAGE_GENERICFEEDBACK_H