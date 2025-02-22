// Generated by gencpp from file ur_dashboard_msgs/GetProgramStateResponse.msg
// DO NOT EDIT!


#ifndef UR_DASHBOARD_MSGS_MESSAGE_GETPROGRAMSTATERESPONSE_H
#define UR_DASHBOARD_MSGS_MESSAGE_GETPROGRAMSTATERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <ur_dashboard_msgs/ProgramState.h>

namespace ur_dashboard_msgs
{
template <class ContainerAllocator>
struct GetProgramStateResponse_
{
  typedef GetProgramStateResponse_<ContainerAllocator> Type;

  GetProgramStateResponse_()
    : state()
    , program_name()
    , answer()
    , success(false)  {
    }
  GetProgramStateResponse_(const ContainerAllocator& _alloc)
    : state(_alloc)
    , program_name(_alloc)
    , answer(_alloc)
    , success(false)  {
  (void)_alloc;
    }



   typedef  ::ur_dashboard_msgs::ProgramState_<ContainerAllocator>  _state_type;
  _state_type state;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _program_name_type;
  _program_name_type program_name;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _answer_type;
  _answer_type answer;

   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetProgramStateResponse_

typedef ::ur_dashboard_msgs::GetProgramStateResponse_<std::allocator<void> > GetProgramStateResponse;

typedef boost::shared_ptr< ::ur_dashboard_msgs::GetProgramStateResponse > GetProgramStateResponsePtr;
typedef boost::shared_ptr< ::ur_dashboard_msgs::GetProgramStateResponse const> GetProgramStateResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ur_dashboard_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'ur_dashboard_msgs': ['/home/demo/ur3_driver/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs/msg', '/home/demo/ur3_driver/devel/share/ur_dashboard_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "522d35a6f2ebd9702cb1e33489e1aa96";
  }

  static const char* value(const ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x522d35a6f2ebd970ULL;
  static const uint64_t static_value2 = 0x2cb1e33489e1aa96ULL;
};

template<class ContainerAllocator>
struct DataType< ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ur_dashboard_msgs/GetProgramStateResponse";
  }

  static const char* value(const ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ProgramState state\n\
string program_name\n\
string answer\n\
bool success\n\
\n\
\n\
================================================================================\n\
MSG: ur_dashboard_msgs/ProgramState\n\
string STOPPED=STOPPED\n\
string PLAYING=PLAYING\n\
string PAUSED=PAUSED\n\
\n\
string state\n\
\n\
";
  }

  static const char* value(const ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.state);
      stream.next(m.program_name);
      stream.next(m.answer);
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetProgramStateResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ur_dashboard_msgs::GetProgramStateResponse_<ContainerAllocator>& v)
  {
    s << indent << "state: ";
    s << std::endl;
    Printer< ::ur_dashboard_msgs::ProgramState_<ContainerAllocator> >::stream(s, indent + "  ", v.state);
    s << indent << "program_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.program_name);
    s << indent << "answer: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.answer);
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UR_DASHBOARD_MSGS_MESSAGE_GETPROGRAMSTATERESPONSE_H
