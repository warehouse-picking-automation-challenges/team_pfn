// Generated by gencpp from file apc2016/ItemPoseEstRequest.msg
// DO NOT EDIT!


#ifndef APC2016_MESSAGE_ITEMPOSEESTREQUEST_H
#define APC2016_MESSAGE_ITEMPOSEESTREQUEST_H


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
struct ItemPoseEstRequest_
{
  typedef ItemPoseEstRequest_<ContainerAllocator> Type;

  ItemPoseEstRequest_()
    : item()  {
    }
  ItemPoseEstRequest_(const ContainerAllocator& _alloc)
    : item(_alloc)  {
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _item_type;
  _item_type item;




  typedef boost::shared_ptr< ::apc2016::ItemPoseEstRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::apc2016::ItemPoseEstRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ItemPoseEstRequest_

typedef ::apc2016::ItemPoseEstRequest_<std::allocator<void> > ItemPoseEstRequest;

typedef boost::shared_ptr< ::apc2016::ItemPoseEstRequest > ItemPoseEstRequestPtr;
typedef boost::shared_ptr< ::apc2016::ItemPoseEstRequest const> ItemPoseEstRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::apc2016::ItemPoseEstRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::apc2016::ItemPoseEstRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::apc2016::ItemPoseEstRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::apc2016::ItemPoseEstRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::apc2016::ItemPoseEstRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::apc2016::ItemPoseEstRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::apc2016::ItemPoseEstRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::apc2016::ItemPoseEstRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::apc2016::ItemPoseEstRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2d14a38979abbdbe6981f6dc67e36fef";
  }

  static const char* value(const ::apc2016::ItemPoseEstRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2d14a38979abbdbeULL;
  static const uint64_t static_value2 = 0x6981f6dc67e36fefULL;
};

template<class ContainerAllocator>
struct DataType< ::apc2016::ItemPoseEstRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "apc2016/ItemPoseEstRequest";
  }

  static const char* value(const ::apc2016::ItemPoseEstRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::apc2016::ItemPoseEstRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string item\n\
";
  }

  static const char* value(const ::apc2016::ItemPoseEstRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::apc2016::ItemPoseEstRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.item);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct ItemPoseEstRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::apc2016::ItemPoseEstRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::apc2016::ItemPoseEstRequest_<ContainerAllocator>& v)
  {
    s << indent << "item: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.item);
  }
};

} // namespace message_operations
} // namespace ros

#endif // APC2016_MESSAGE_ITEMPOSEESTREQUEST_H
