// Generated by gencpp from file apc2016/XYZImage.msg
// DO NOT EDIT!


#ifndef APC2016_MESSAGE_XYZIMAGE_H
#define APC2016_MESSAGE_XYZIMAGE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>

namespace apc2016
{
template <class ContainerAllocator>
struct XYZImage_
{
  typedef XYZImage_<ContainerAllocator> Type;

  XYZImage_()
    : x()
    , y()
    , z()  {
    }
  XYZImage_(const ContainerAllocator& _alloc)
    : x(_alloc)
    , y(_alloc)
    , z(_alloc)  {
    }



   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _x_type;
  _x_type x;

   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _y_type;
  _y_type y;

   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _z_type;
  _z_type z;




  typedef boost::shared_ptr< ::apc2016::XYZImage_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::apc2016::XYZImage_<ContainerAllocator> const> ConstPtr;

}; // struct XYZImage_

typedef ::apc2016::XYZImage_<std::allocator<void> > XYZImage;

typedef boost::shared_ptr< ::apc2016::XYZImage > XYZImagePtr;
typedef boost::shared_ptr< ::apc2016::XYZImage const> XYZImageConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::apc2016::XYZImage_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::apc2016::XYZImage_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::apc2016::XYZImage_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::apc2016::XYZImage_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::apc2016::XYZImage_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::apc2016::XYZImage_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::apc2016::XYZImage_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::apc2016::XYZImage_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::apc2016::XYZImage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "47f16ab30fc71d03566789ee4004bc8a";
  }

  static const char* value(const ::apc2016::XYZImage_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x47f16ab30fc71d03ULL;
  static const uint64_t static_value2 = 0x566789ee4004bc8aULL;
};

template<class ContainerAllocator>
struct DataType< ::apc2016::XYZImage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "apc2016/XYZImage";
  }

  static const char* value(const ::apc2016::XYZImage_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::apc2016::XYZImage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sensor_msgs/Image x\n\
sensor_msgs/Image y\n\
sensor_msgs/Image z\n\
\n\
\n\
================================================================================\n\
MSG: sensor_msgs/Image\n\
# This message contains an uncompressed image\n\
# (0, 0) is at top-left corner of image\n\
#\n\
\n\
Header header        # Header timestamp should be acquisition time of image\n\
                     # Header frame_id should be optical frame of camera\n\
                     # origin of frame should be optical center of cameara\n\
                     # +x should point to the right in the image\n\
                     # +y should point down in the image\n\
                     # +z should point into to plane of the image\n\
                     # If the frame_id here and the frame_id of the CameraInfo\n\
                     # message associated with the image conflict\n\
                     # the behavior is undefined\n\
\n\
uint32 height         # image height, that is, number of rows\n\
uint32 width          # image width, that is, number of columns\n\
\n\
# The legal values for encoding are in file src/image_encodings.cpp\n\
# If you want to standardize a new string format, join\n\
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n\
\n\
string encoding       # Encoding of pixels -- channel meaning, ordering, size\n\
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n\
\n\
uint8 is_bigendian    # is this data bigendian?\n\
uint32 step           # Full row length in bytes\n\
uint8[] data          # actual matrix data, size is (step * rows)\n\
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

  static const char* value(const ::apc2016::XYZImage_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::apc2016::XYZImage_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct XYZImage_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::apc2016::XYZImage_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::apc2016::XYZImage_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // APC2016_MESSAGE_XYZIMAGE_H
