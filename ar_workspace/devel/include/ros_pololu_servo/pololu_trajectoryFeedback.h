// Generated by gencpp from file ros_pololu_servo/pololu_trajectoryFeedback.msg
// DO NOT EDIT!


#ifndef ROS_POLOLU_SERVO_MESSAGE_POLOLU_TRAJECTORYFEEDBACK_H
#define ROS_POLOLU_SERVO_MESSAGE_POLOLU_TRAJECTORYFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ros_pololu_servo
{
template <class ContainerAllocator>
struct pololu_trajectoryFeedback_
{
  typedef pololu_trajectoryFeedback_<ContainerAllocator> Type;

  pololu_trajectoryFeedback_()
    : percent_complete(0.0)  {
    }
  pololu_trajectoryFeedback_(const ContainerAllocator& _alloc)
    : percent_complete(0.0)  {
  (void)_alloc;
    }



   typedef float _percent_complete_type;
  _percent_complete_type percent_complete;





  typedef boost::shared_ptr< ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct pololu_trajectoryFeedback_

typedef ::ros_pololu_servo::pololu_trajectoryFeedback_<std::allocator<void> > pololu_trajectoryFeedback;

typedef boost::shared_ptr< ::ros_pololu_servo::pololu_trajectoryFeedback > pololu_trajectoryFeedbackPtr;
typedef boost::shared_ptr< ::ros_pololu_servo::pololu_trajectoryFeedback const> pololu_trajectoryFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ros_pololu_servo

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'trajectory_msgs': ['/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'ros_pololu_servo': ['/home/odroid/ar_workspace/src/ros_pololu_servo/msg', '/home/odroid/ar_workspace/devel/share/ros_pololu_servo/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d342375c60a5a58d3bc32664070a1368";
  }

  static const char* value(const ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd342375c60a5a58dULL;
  static const uint64_t static_value2 = 0x3bc32664070a1368ULL;
};

template<class ContainerAllocator>
struct DataType< ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ros_pololu_servo/pololu_trajectoryFeedback";
  }

  static const char* value(const ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# Define a feedback message\n\
float32 percent_complete\n\
";
  }

  static const char* value(const ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.percent_complete);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct pololu_trajectoryFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ros_pololu_servo::pololu_trajectoryFeedback_<ContainerAllocator>& v)
  {
    s << indent << "percent_complete: ";
    Printer<float>::stream(s, indent + "  ", v.percent_complete);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROS_POLOLU_SERVO_MESSAGE_POLOLU_TRAJECTORYFEEDBACK_H
