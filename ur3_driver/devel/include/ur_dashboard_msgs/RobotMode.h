// Generated by gencpp from file ur_dashboard_msgs/RobotMode.msg
// DO NOT EDIT!


#ifndef UR_DASHBOARD_MSGS_MESSAGE_ROBOTMODE_H
#define UR_DASHBOARD_MSGS_MESSAGE_ROBOTMODE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ur_dashboard_msgs
{
template <class ContainerAllocator>
struct RobotMode_
{
  typedef RobotMode_<ContainerAllocator> Type;

  RobotMode_()
    : mode(0)  {
    }
  RobotMode_(const ContainerAllocator& _alloc)
    : mode(0)  {
  (void)_alloc;
    }



   typedef int8_t _mode_type;
  _mode_type mode;



  enum {
    NO_CONTROLLER = -1,
    DISCONNECTED = 0,
    CONFIRM_SAFETY = 1,
    BOOTING = 2,
    POWER_OFF = 3,
    POWER_ON = 4,
    IDLE = 5,
    BACKDRIVE = 6,
    RUNNING = 7,
    UPDATING_FIRMWARE = 8,
  };


  typedef boost::shared_ptr< ::ur_dashboard_msgs::RobotMode_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ur_dashboard_msgs::RobotMode_<ContainerAllocator> const> ConstPtr;

}; // struct RobotMode_

typedef ::ur_dashboard_msgs::RobotMode_<std::allocator<void> > RobotMode;

typedef boost::shared_ptr< ::ur_dashboard_msgs::RobotMode > RobotModePtr;
typedef boost::shared_ptr< ::ur_dashboard_msgs::RobotMode const> RobotModeConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ur_dashboard_msgs::RobotMode_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ur_dashboard_msgs::RobotMode_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ur_dashboard_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'ur_dashboard_msgs': ['/home/demo/ur3_driver/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs/msg', '/home/demo/ur3_driver/devel/share/ur_dashboard_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ur_dashboard_msgs::RobotMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur_dashboard_msgs::RobotMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_dashboard_msgs::RobotMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_dashboard_msgs::RobotMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_dashboard_msgs::RobotMode_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_dashboard_msgs::RobotMode_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ur_dashboard_msgs::RobotMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "73b72d58742c4889c47118411b03a3b4";
  }

  static const char* value(const ::ur_dashboard_msgs::RobotMode_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x73b72d58742c4889ULL;
  static const uint64_t static_value2 = 0xc47118411b03a3b4ULL;
};

template<class ContainerAllocator>
struct DataType< ::ur_dashboard_msgs::RobotMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ur_dashboard_msgs/RobotMode";
  }

  static const char* value(const ::ur_dashboard_msgs::RobotMode_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ur_dashboard_msgs::RobotMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 NO_CONTROLLER=-1\n\
int8 DISCONNECTED=0\n\
int8 CONFIRM_SAFETY=1\n\
int8 BOOTING=2\n\
int8 POWER_OFF=3\n\
int8 POWER_ON=4\n\
int8 IDLE=5\n\
int8 BACKDRIVE=6\n\
int8 RUNNING=7\n\
int8 UPDATING_FIRMWARE=8\n\
\n\
int8 mode\n\
\n\
";
  }

  static const char* value(const ::ur_dashboard_msgs::RobotMode_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ur_dashboard_msgs::RobotMode_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RobotMode_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ur_dashboard_msgs::RobotMode_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ur_dashboard_msgs::RobotMode_<ContainerAllocator>& v)
  {
    s << indent << "mode: ";
    Printer<int8_t>::stream(s, indent + "  ", v.mode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UR_DASHBOARD_MSGS_MESSAGE_ROBOTMODE_H
