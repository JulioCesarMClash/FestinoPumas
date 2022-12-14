// Generated by gencpp from file robotino_msgs/SetEncoderPosition.msg
// DO NOT EDIT!


#ifndef ROBOTINO_MSGS_MESSAGE_SETENCODERPOSITION_H
#define ROBOTINO_MSGS_MESSAGE_SETENCODERPOSITION_H

#include <ros/service_traits.h>


#include <robotino_msgs/SetEncoderPositionRequest.h>
#include <robotino_msgs/SetEncoderPositionResponse.h>


namespace robotino_msgs
{

struct SetEncoderPosition
{

typedef SetEncoderPositionRequest Request;
typedef SetEncoderPositionResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetEncoderPosition
} // namespace robotino_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::robotino_msgs::SetEncoderPosition > {
  static const char* value()
  {
    return "e17f4b08b3e2cdbbf2b0d78e4f62e5d6";
  }

  static const char* value(const ::robotino_msgs::SetEncoderPosition&) { return value(); }
};

template<>
struct DataType< ::robotino_msgs::SetEncoderPosition > {
  static const char* value()
  {
    return "robotino_msgs/SetEncoderPosition";
  }

  static const char* value(const ::robotino_msgs::SetEncoderPosition&) { return value(); }
};


// service_traits::MD5Sum< ::robotino_msgs::SetEncoderPositionRequest> should match
// service_traits::MD5Sum< ::robotino_msgs::SetEncoderPosition >
template<>
struct MD5Sum< ::robotino_msgs::SetEncoderPositionRequest>
{
  static const char* value()
  {
    return MD5Sum< ::robotino_msgs::SetEncoderPosition >::value();
  }
  static const char* value(const ::robotino_msgs::SetEncoderPositionRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::robotino_msgs::SetEncoderPositionRequest> should match
// service_traits::DataType< ::robotino_msgs::SetEncoderPosition >
template<>
struct DataType< ::robotino_msgs::SetEncoderPositionRequest>
{
  static const char* value()
  {
    return DataType< ::robotino_msgs::SetEncoderPosition >::value();
  }
  static const char* value(const ::robotino_msgs::SetEncoderPositionRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::robotino_msgs::SetEncoderPositionResponse> should match
// service_traits::MD5Sum< ::robotino_msgs::SetEncoderPosition >
template<>
struct MD5Sum< ::robotino_msgs::SetEncoderPositionResponse>
{
  static const char* value()
  {
    return MD5Sum< ::robotino_msgs::SetEncoderPosition >::value();
  }
  static const char* value(const ::robotino_msgs::SetEncoderPositionResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::robotino_msgs::SetEncoderPositionResponse> should match
// service_traits::DataType< ::robotino_msgs::SetEncoderPosition >
template<>
struct DataType< ::robotino_msgs::SetEncoderPositionResponse>
{
  static const char* value()
  {
    return DataType< ::robotino_msgs::SetEncoderPosition >::value();
  }
  static const char* value(const ::robotino_msgs::SetEncoderPositionResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROBOTINO_MSGS_MESSAGE_SETENCODERPOSITION_H
