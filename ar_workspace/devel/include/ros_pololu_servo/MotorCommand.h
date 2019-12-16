// Generated by gencpp from file ros_pololu_servo/MotorCommand.msg
// DO NOT EDIT!


#ifndef ROS_POLOLU_SERVO_MESSAGE_MOTORCOMMAND_H
#define ROS_POLOLU_SERVO_MESSAGE_MOTORCOMMAND_H


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
struct MotorCommand_
{
  typedef MotorCommand_<ContainerAllocator> Type;

  MotorCommand_()
    : joint_name()
    , position(0.0)
    , speed(0.0)
    , acceleration(0.0)  {
    }
  MotorCommand_(const ContainerAllocator& _alloc)
    : joint_name(_alloc)
    , position(0.0)
    , speed(0.0)
    , acceleration(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _joint_name_type;
  _joint_name_type joint_name;

   typedef double _position_type;
  _position_type position;

   typedef float _speed_type;
  _speed_type speed;

   typedef float _acceleration_type;
  _acceleration_type acceleration;





  typedef boost::shared_ptr< ::ros_pololu_servo::MotorCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ros_pololu_servo::MotorCommand_<ContainerAllocator> const> ConstPtr;

}; // struct MotorCommand_

typedef ::ros_pololu_servo::MotorCommand_<std::allocator<void> > MotorCommand;

typedef boost::shared_ptr< ::ros_pololu_servo::MotorCommand > MotorCommandPtr;
typedef boost::shared_ptr< ::ros_pololu_servo::MotorCommand const> MotorCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ros_pololu_servo::MotorCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ros_pololu_servo::MotorCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ros_pololu_servo

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'trajectory_msgs': ['/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'ros_pololu_servo': ['/home/odroid/ar_workspace/src/ros_pololu_servo/msg', '/home/odroid/ar_workspace/devel/share/ros_pololu_servo/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ros_pololu_servo::MotorCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ros_pololu_servo::MotorCommand_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_pololu_servo::MotorCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_pololu_servo::MotorCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_pololu_servo::MotorCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_pololu_servo::MotorCommand_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ros_pololu_servo::MotorCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c357daac337dac3f7e4bb73a055e6e8c";
  }

  static const char* value(const ::ros_pololu_servo::MotorCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc357daac337dac3fULL;
  static const uint64_t static_value2 = 0x7e4bb73a055e6e8cULL;
};

template<class ContainerAllocator>
struct DataType< ::ros_pololu_servo::MotorCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ros_pololu_servo/MotorCommand";
  }

  static const char* value(const ::ros_pololu_servo::MotorCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ros_pololu_servo::MotorCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string joint_name       # Name of the joint (specified in the yaml file), or motor_id for default calibration\n\
float64 position        # Position to move to in radians\n\
float32 speed           # Speed to move at (0.0 - 1.0)\n\
float32 acceleration    # Acceleration to move at (0.0 - 1.0)\n\
";
  }

  static const char* value(const ::ros_pololu_servo::MotorCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ros_pololu_servo::MotorCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.joint_name);
      stream.next(m.position);
      stream.next(m.speed);
      stream.next(m.acceleration);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MotorCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ros_pololu_servo::MotorCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ros_pololu_servo::MotorCommand_<ContainerAllocator>& v)
  {
    s << indent << "joint_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.joint_name);
    s << indent << "position: ";
    Printer<double>::stream(s, indent + "  ", v.position);
    s << indent << "speed: ";
    Printer<float>::stream(s, indent + "  ", v.speed);
    s << indent << "acceleration: ";
    Printer<float>::stream(s, indent + "  ", v.acceleration);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROS_POLOLU_SERVO_MESSAGE_MOTORCOMMAND_H
