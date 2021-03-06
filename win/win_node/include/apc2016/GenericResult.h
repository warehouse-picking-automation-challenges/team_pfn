// Generated by gencpp from file apc2016/GenericResult.msg
// DO NOT EDIT!


#ifndef APC2016_MESSAGE_GENERICRESULT_H
#define APC2016_MESSAGE_GENERICRESULT_H


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
struct GenericResult_
{
  typedef GenericResult_<ContainerAllocator> Type;

  GenericResult_()
    : result()
    , error()  {
    }
  GenericResult_(const ContainerAllocator& _alloc)
    : result(_alloc)
    , error(_alloc)  {
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _result_type;
  _result_type result;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _error_type;
  _error_type error;




  typedef boost::shared_ptr< ::apc2016::GenericResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::apc2016::GenericResult_<ContainerAllocator> const> ConstPtr;

}; // struct GenericResult_

typedef ::apc2016::GenericResult_<std::allocator<void> > GenericResult;

typedef boost::shared_ptr< ::apc2016::GenericResult > GenericResultPtr;
typedef boost::shared_ptr< ::apc2016::GenericResult const> GenericResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::apc2016::GenericResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::apc2016::GenericResult_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::apc2016::GenericResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::apc2016::GenericResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::apc2016::GenericResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::apc2016::GenericResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::apc2016::GenericResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::apc2016::GenericResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::apc2016::GenericResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "29d0896394f2279e1bd5505a7f4c4359";
  }

  static const char* value(const ::apc2016::GenericResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x29d0896394f2279eULL;
  static const uint64_t static_value2 = 0x1bd5505a7f4c4359ULL;
};

template<class ContainerAllocator>
struct DataType< ::apc2016::GenericResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "apc2016/GenericResult";
  }

  static const char* value(const ::apc2016::GenericResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::apc2016::GenericResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# Define the result\n\
# result:\n\
# - for linear servo:\n\
#   space-concatenation of\n\
#   - \"0\" or \"1\" (position index) OR \"-1\" or \"-2\" (for servo commands)\n\
#   - decimal number (position in mm)\n\
#   e.g. \"1 20.03\"\n\
# - for rotary servos:\n\
#   space-concatenation of\n\
#   - decimal number (angle in deg for first servo)\n\
#   - decimal number (angle in deg for second servo)\n\
#   e.g. \"20.3 90.8\"\n\
# error:\n\
#   contains an error message if a problem\n\
#   occured during move. if there is such\n\
#   a message, the values in the \"result\"\n\
#   string (if present) have no meaning\n\
string  result\n\
string  error\n\
";
  }

  static const char* value(const ::apc2016::GenericResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::apc2016::GenericResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.result);
      stream.next(m.error);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct GenericResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::apc2016::GenericResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::apc2016::GenericResult_<ContainerAllocator>& v)
  {
    s << indent << "result: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.result);
    s << indent << "error: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.error);
  }
};

} // namespace message_operations
} // namespace ros

#endif // APC2016_MESSAGE_GENERICRESULT_H
