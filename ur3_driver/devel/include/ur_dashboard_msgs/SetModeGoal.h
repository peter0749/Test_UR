// Generated by gencpp from file ur_dashboard_msgs/SetModeGoal.msg
// DO NOT EDIT!


#ifndef UR_DASHBOARD_MSGS_MESSAGE_SETMODEGOAL_H
#define UR_DASHBOARD_MSGS_MESSAGE_SETMODEGOAL_H


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
struct SetModeGoal_
{
  typedef SetModeGoal_<ContainerAllocator> Type;

  SetModeGoal_()
    : target_robot_mode(0)
    , stop_program(false)
    , play_program(false)  {
    }
  SetModeGoal_(const ContainerAllocator& _alloc)
    : target_robot_mode(0)
    , stop_program(false)
    , play_program(false)  {
  (void)_alloc;
    }



   typedef int8_t _target_robot_mode_type;
  _target_robot_mode_type target_robot_mode;

   typedef uint8_t _stop_program_type;
  _stop_program_type stop_program;

   typedef uint8_t _play_program_type;
  _play_program_type play_program;





  typedef boost::shared_ptr< ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator> const> ConstPtr;

}; // struct SetModeGoal_

typedef ::ur_dashboard_msgs::SetModeGoal_<std::allocator<void> > SetModeGoal;

typedef boost::shared_ptr< ::ur_dashboard_msgs::SetModeGoal > SetModeGoalPtr;
typedef boost::shared_ptr< ::ur_dashboard_msgs::SetModeGoal const> SetModeGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6832df07338535cc06b3835f89ba9555";
  }

  static const char* value(const ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6832df07338535ccULL;
  static const uint64_t static_value2 = 0x06b3835f89ba9555ULL;
};

template<class ContainerAllocator>
struct DataType< ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ur_dashboard_msgs/SetModeGoal";
  }

  static const char* value(const ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# This action is for setting the robot into a desired mode (e.g. RUNNING) and safety mode into a\n\
# non-critical state (e.g. NORMAL or REDUCED), for example after a safety incident happened.\n\
\n\
# goal\n\
int8 target_robot_mode\n\
\n\
# Stop program execution before restoring the target mode. Can be used together with 'play_program'.\n\
bool stop_program\n\
\n\
# Play the currently loaded program after target mode is reached.#\n\
# NOTE: Requesting mode RUNNING in combination with this will make the robot continue the motion it\n\
# was doing before. This might probably lead into the same problem (protective stop, EM-Stop due to\n\
# faulty motion, etc.) If you want to be safe, set the 'stop_program' flag below and manually play\n\
# the program after robot state is returned to normal.\n\
# This flag will only be used when requesting mode RUNNING\n\
bool play_program\n\
\n\
";
  }

  static const char* value(const ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.target_robot_mode);
      stream.next(m.stop_program);
      stream.next(m.play_program);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetModeGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ur_dashboard_msgs::SetModeGoal_<ContainerAllocator>& v)
  {
    s << indent << "target_robot_mode: ";
    Printer<int8_t>::stream(s, indent + "  ", v.target_robot_mode);
    s << indent << "stop_program: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.stop_program);
    s << indent << "play_program: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.play_program);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UR_DASHBOARD_MSGS_MESSAGE_SETMODEGOAL_H
