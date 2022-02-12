// Generated by gencpp from file ur_dashboard_msgs/PopupRequest.msg
// DO NOT EDIT!


#ifndef UR_DASHBOARD_MSGS_MESSAGE_POPUPREQUEST_H
#define UR_DASHBOARD_MSGS_MESSAGE_POPUPREQUEST_H


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
struct PopupRequest_
{
  typedef PopupRequest_<ContainerAllocator> Type;

  PopupRequest_()
    : message()  {
    }
  PopupRequest_(const ContainerAllocator& _alloc)
    : message(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _message_type;
  _message_type message;





  typedef boost::shared_ptr< ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator> const> ConstPtr;

}; // struct PopupRequest_

typedef ::ur_dashboard_msgs::PopupRequest_<std::allocator<void> > PopupRequest;

typedef boost::shared_ptr< ::ur_dashboard_msgs::PopupRequest > PopupRequestPtr;
typedef boost::shared_ptr< ::ur_dashboard_msgs::PopupRequest const> PopupRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5f003d6bcc824cbd51361d66d8e4f76c";
  }

  static const char* value(const ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5f003d6bcc824cbdULL;
  static const uint64_t static_value2 = 0x51361d66d8e4f76cULL;
};

template<class ContainerAllocator>
struct DataType< ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ur_dashboard_msgs/PopupRequest";
  }

  static const char* value(const ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string message\n\
";
  }

  static const char* value(const ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.message);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PopupRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ur_dashboard_msgs::PopupRequest_<ContainerAllocator>& v)
  {
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.message);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UR_DASHBOARD_MSGS_MESSAGE_POPUPREQUEST_H
