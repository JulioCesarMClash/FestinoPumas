// Generated by gencpp from file robotino_msgs/MotorReadings.msg
// DO NOT EDIT!


#ifndef ROBOTINO_MSGS_MESSAGE_MOTORREADINGS_H
#define ROBOTINO_MSGS_MESSAGE_MOTORREADINGS_H


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
struct MotorReadings_
{
  typedef MotorReadings_<ContainerAllocator> Type;

  MotorReadings_()
    : stamp()
    , velocities()
    , positions()
    , currents()  {
    }
  MotorReadings_(const ContainerAllocator& _alloc)
    : stamp()
    , velocities(_alloc)
    , positions(_alloc)
    , currents(_alloc)  {
  (void)_alloc;
    }



   typedef ros::Time _stamp_type;
  _stamp_type stamp;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _velocities_type;
  _velocities_type velocities;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _positions_type;
  _positions_type positions;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _currents_type;
  _currents_type currents;





  typedef boost::shared_ptr< ::robotino_msgs::MotorReadings_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotino_msgs::MotorReadings_<ContainerAllocator> const> ConstPtr;

}; // struct MotorReadings_

typedef ::robotino_msgs::MotorReadings_<std::allocator<void> > MotorReadings;

typedef boost::shared_ptr< ::robotino_msgs::MotorReadings > MotorReadingsPtr;
typedef boost::shared_ptr< ::robotino_msgs::MotorReadings const> MotorReadingsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotino_msgs::MotorReadings_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotino_msgs::MotorReadings_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robotino_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'robotino_msgs': ['/home/robotino/FestinoPumas/Robotino/src/Hardware/robotino_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robotino_msgs::MotorReadings_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotino_msgs::MotorReadings_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotino_msgs::MotorReadings_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotino_msgs::MotorReadings_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotino_msgs::MotorReadings_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotino_msgs::MotorReadings_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotino_msgs::MotorReadings_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3974e9bd8305667fc0637697b49a8e1f";
  }

  static const char* value(const ::robotino_msgs::MotorReadings_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3974e9bd8305667fULL;
  static const uint64_t static_value2 = 0xc0637697b49a8e1fULL;
};

template<class ContainerAllocator>
struct DataType< ::robotino_msgs::MotorReadings_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotino_msgs/MotorReadings";
  }

  static const char* value(const ::robotino_msgs::MotorReadings_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotino_msgs::MotorReadings_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time stamp\n\
float32[] velocities 	# in rpm\n\
int32[] positions\n\
float32[] currents		# in A\n\
";
  }

  static const char* value(const ::robotino_msgs::MotorReadings_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotino_msgs::MotorReadings_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.stamp);
      stream.next(m.velocities);
      stream.next(m.positions);
      stream.next(m.currents);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MotorReadings_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotino_msgs::MotorReadings_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotino_msgs::MotorReadings_<ContainerAllocator>& v)
  {
    s << indent << "stamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.stamp);
    s << indent << "velocities[]" << std::endl;
    for (size_t i = 0; i < v.velocities.size(); ++i)
    {
      s << indent << "  velocities[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.velocities[i]);
    }
    s << indent << "positions[]" << std::endl;
    for (size_t i = 0; i < v.positions.size(); ++i)
    {
      s << indent << "  positions[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.positions[i]);
    }
    s << indent << "currents[]" << std::endl;
    for (size_t i = 0; i < v.currents.size(); ++i)
    {
      s << indent << "  currents[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.currents[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTINO_MSGS_MESSAGE_MOTORREADINGS_H
