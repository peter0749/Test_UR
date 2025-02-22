// Generated by gencpp from file ur_dashboard_msgs/IsProgramSavedResponse.msg
// DO NOT EDIT!


#ifndef UR_DASHBOARD_MSGS_MESSAGE_ISPROGRAMSAVEDRESPONSE_H
#define UR_DASHBOARD_MSGS_MESSAGE_ISPROGRAMSAVEDRESPONSE_H


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
struct IsProgramSavedResponse_
{
  typedef IsProgramSavedResponse_<ContainerAllocator> Type;

  IsProgramSavedResponse_()
    : answer()
    , program_name()
    , program_saved(false)
    , success(false)  {
    }
  IsProgramSavedResponse_(const ContainerAllocator& _alloc)
    : answer(_alloc)
    , program_name(_alloc)
    , program_saved(false)
    , success(false)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _answer_type;
  _answer_type answer;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _program_name_type;
  _program_name_type program_name;

   typedef uint8_t _program_saved_type;
  _program_saved_type program_saved;

   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator> const> ConstPtr;

}; // struct IsProgramSavedResponse_

typedef ::ur_dashboard_msgs::IsProgramSavedResponse_<std::allocator<void> > IsProgramSavedResponse;

typedef boost::shared_ptr< ::ur_dashboard_msgs::IsProgramSavedResponse > IsProgramSavedResponsePtr;
typedef boost::shared_ptr< ::ur_dashboard_msgs::IsProgramSavedResponse const> IsProgramSavedResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e922e4e4f5e4157c23417860c8b2336a";
  }

  static const char* value(const ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe922e4e4f5e4157cULL;
  static const uint64_t static_value2 = 0x23417860c8b2336aULL;
};

template<class ContainerAllocator>
struct DataType< ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ur_dashboard_msgs/IsProgramSavedResponse";
  }

  static const char* value(const ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string answer\n\
string program_name\n\
bool program_saved\n\
bool success\n\
\n\
";
  }

  static const char* value(const ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.answer);
      stream.next(m.program_name);
      stream.next(m.program_saved);
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct IsProgramSavedResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ur_dashboard_msgs::IsProgramSavedResponse_<ContainerAllocator>& v)
  {
    s << indent << "answer: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.answer);
    s << indent << "program_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.program_name);
    s << indent << "program_saved: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.program_saved);
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UR_DASHBOARD_MSGS_MESSAGE_ISPROGRAMSAVEDRESPONSE_H
