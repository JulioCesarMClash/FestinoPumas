// Generated by gencpp from file robotino_msgs/ResetOdometryResponse.msg
// DO NOT EDIT!


#ifndef ROBOTINO_MSGS_MESSAGE_RESETODOMETRYRESPONSE_H
#define ROBOTINO_MSGS_MESSAGE_RESETODOMETRYRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robotino_msgs
{
template <class ContainerAllocator>
struct ResetOdometryResponse_
{
  typedef ResetOdometryResponse_<ContainerAllocator> Type;

  ResetOdometryResponse_()
    {
    }
  ResetOdometryResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ResetOdometryResponse_

typedef ::robotino_msgs::ResetOdometryResponse_<std::allocator<void> > ResetOdometryResponse;

typedef boost::shared_ptr< ::robotino_msgs::ResetOdometryResponse > ResetOdometryResponsePtr;
typedef boost::shared_ptr< ::robotino_msgs::ResetOdometryResponse const> ResetOdometryResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace robotino_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotino_msgs/ResetOdometryResponse";
  }

  static const char* value(const ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ResetOdometryResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::robotino_msgs::ResetOdometryResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTINO_MSGS_MESSAGE_RESETODOMETRYRESPONSE_H
