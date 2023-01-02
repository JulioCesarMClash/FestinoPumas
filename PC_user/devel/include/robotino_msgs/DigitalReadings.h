// Generated by gencpp from file robotino_msgs/DigitalReadings.msg
// DO NOT EDIT!


#ifndef ROBOTINO_MSGS_MESSAGE_DIGITALREADINGS_H
#define ROBOTINO_MSGS_MESSAGE_DIGITALREADINGS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robotino_msgs
{
template <class ContainerAllocator>
struct DigitalReadings_
{
  typedef DigitalReadings_<ContainerAllocator> Type;

  DigitalReadings_()
    : stamp()
    , values()  {
    }
  DigitalReadings_(const ContainerAllocator& _alloc)
    : stamp()
    , values(_alloc)  {
  (void)_alloc;
    }



   typedef ros::Time _stamp_type;
  _stamp_type stamp;

   typedef std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> _values_type;
  _values_type values;





  typedef boost::shared_ptr< ::robotino_msgs::DigitalReadings_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotino_msgs::DigitalReadings_<ContainerAllocator> const> ConstPtr;

}; // struct DigitalReadings_

typedef ::robotino_msgs::DigitalReadings_<std::allocator<void> > DigitalReadings;

typedef boost::shared_ptr< ::robotino_msgs::DigitalReadings > DigitalReadingsPtr;
typedef boost::shared_ptr< ::robotino_msgs::DigitalReadings const> DigitalReadingsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotino_msgs::DigitalReadings_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotino_msgs::DigitalReadings_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robotino_msgs::DigitalReadings_<ContainerAllocator1> & lhs, const ::robotino_msgs::DigitalReadings_<ContainerAllocator2> & rhs)
{
  return lhs.stamp == rhs.stamp &&
    lhs.values == rhs.values;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robotino_msgs::DigitalReadings_<ContainerAllocator1> & lhs, const ::robotino_msgs::DigitalReadings_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robotino_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::robotino_msgs::DigitalReadings_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotino_msgs::DigitalReadings_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotino_msgs::DigitalReadings_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotino_msgs::DigitalReadings_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotino_msgs::DigitalReadings_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotino_msgs::DigitalReadings_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotino_msgs::DigitalReadings_<ContainerAllocator> >
{
  static const char* value()
  {
    return "21240637a82d18c261b7e2f567659e7e";
  }

  static const char* value(const ::robotino_msgs::DigitalReadings_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x21240637a82d18c2ULL;
  static const uint64_t static_value2 = 0x61b7e2f567659e7eULL;
};

template<class ContainerAllocator>
struct DataType< ::robotino_msgs::DigitalReadings_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotino_msgs/DigitalReadings";
  }

  static const char* value(const ::robotino_msgs::DigitalReadings_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotino_msgs::DigitalReadings_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time stamp\n"
"bool[] values\n"
;
  }

  static const char* value(const ::robotino_msgs::DigitalReadings_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotino_msgs::DigitalReadings_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.stamp);
      stream.next(m.values);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DigitalReadings_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotino_msgs::DigitalReadings_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotino_msgs::DigitalReadings_<ContainerAllocator>& v)
  {
    s << indent << "stamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.stamp);
    s << indent << "values[]" << std::endl;
    for (size_t i = 0; i < v.values.size(); ++i)
    {
      s << indent << "  values[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.values[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTINO_MSGS_MESSAGE_DIGITALREADINGS_H
