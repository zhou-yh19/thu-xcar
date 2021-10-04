// Generated by gencpp from file vesc_msgs/VescCtrl.msg
// DO NOT EDIT!


#ifndef VESC_MSGS_MESSAGE_VESCCTRL_H
#define VESC_MSGS_MESSAGE_VESCCTRL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace vesc_msgs
{
template <class ContainerAllocator>
struct VescCtrl_
{
  typedef VescCtrl_<ContainerAllocator> Type;

  VescCtrl_()
    : mode(0)
    , duty_cycle(0.0)
    , current(0.0)
    , brake(0.0)
    , speed(0.0)
    , position(0.0)
    , servo(0.0)  {
    }
  VescCtrl_(const ContainerAllocator& _alloc)
    : mode(0)
    , duty_cycle(0.0)
    , current(0.0)
    , brake(0.0)
    , speed(0.0)
    , position(0.0)
    , servo(0.0)  {
  (void)_alloc;
    }



   typedef int64_t _mode_type;
  _mode_type mode;

   typedef double _duty_cycle_type;
  _duty_cycle_type duty_cycle;

   typedef double _current_type;
  _current_type current;

   typedef double _brake_type;
  _brake_type brake;

   typedef double _speed_type;
  _speed_type speed;

   typedef double _position_type;
  _position_type position;

   typedef double _servo_type;
  _servo_type servo;





  typedef boost::shared_ptr< ::vesc_msgs::VescCtrl_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vesc_msgs::VescCtrl_<ContainerAllocator> const> ConstPtr;

}; // struct VescCtrl_

typedef ::vesc_msgs::VescCtrl_<std::allocator<void> > VescCtrl;

typedef boost::shared_ptr< ::vesc_msgs::VescCtrl > VescCtrlPtr;
typedef boost::shared_ptr< ::vesc_msgs::VescCtrl const> VescCtrlConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vesc_msgs::VescCtrl_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vesc_msgs::VescCtrl_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace vesc_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'vesc_msgs': ['/home/nvidia/thu-Xcar/src/xcar-hardware/vesc/vesc_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::vesc_msgs::VescCtrl_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vesc_msgs::VescCtrl_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vesc_msgs::VescCtrl_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vesc_msgs::VescCtrl_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vesc_msgs::VescCtrl_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vesc_msgs::VescCtrl_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vesc_msgs::VescCtrl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "034ba36ad2a5637b06f87a196fb8b3a2";
  }

  static const char* value(const ::vesc_msgs::VescCtrl_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x034ba36ad2a5637bULL;
  static const uint64_t static_value2 = 0x06f87a196fb8b3a2ULL;
};

template<class ContainerAllocator>
struct DataType< ::vesc_msgs::VescCtrl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vesc_msgs/VescCtrl";
  }

  static const char* value(const ::vesc_msgs::VescCtrl_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vesc_msgs::VescCtrl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#commanded VESC duty cycle, current, brake current, speed, motor position, servo position\n\
\n\
int64 mode  #control mode 1-brake mode,2-current mode,3-speed mode,4-duty_cycle mode,5-position mode\n\
float64 duty_cycle #vesc control mode\n\
float64 current    #vesc control mode\n\
float64 brake\n\
float64 speed      #vesc control mode\n\
float64 position\n\
float64 servo\n\
";
  }

  static const char* value(const ::vesc_msgs::VescCtrl_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vesc_msgs::VescCtrl_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mode);
      stream.next(m.duty_cycle);
      stream.next(m.current);
      stream.next(m.brake);
      stream.next(m.speed);
      stream.next(m.position);
      stream.next(m.servo);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct VescCtrl_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vesc_msgs::VescCtrl_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vesc_msgs::VescCtrl_<ContainerAllocator>& v)
  {
    s << indent << "mode: ";
    Printer<int64_t>::stream(s, indent + "  ", v.mode);
    s << indent << "duty_cycle: ";
    Printer<double>::stream(s, indent + "  ", v.duty_cycle);
    s << indent << "current: ";
    Printer<double>::stream(s, indent + "  ", v.current);
    s << indent << "brake: ";
    Printer<double>::stream(s, indent + "  ", v.brake);
    s << indent << "speed: ";
    Printer<double>::stream(s, indent + "  ", v.speed);
    s << indent << "position: ";
    Printer<double>::stream(s, indent + "  ", v.position);
    s << indent << "servo: ";
    Printer<double>::stream(s, indent + "  ", v.servo);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VESC_MSGS_MESSAGE_VESCCTRL_H
