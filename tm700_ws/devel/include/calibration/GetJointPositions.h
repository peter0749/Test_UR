// Generated by gencpp from file calibration/GetJointPositions.msg
// DO NOT EDIT!


#ifndef CALIBRATION_MESSAGE_GETJOINTPOSITIONS_H
#define CALIBRATION_MESSAGE_GETJOINTPOSITIONS_H

#include <ros/service_traits.h>


#include <calibration/GetJointPositionsRequest.h>
#include <calibration/GetJointPositionsResponse.h>


namespace calibration
{

struct GetJointPositions
{

typedef GetJointPositionsRequest Request;
typedef GetJointPositionsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetJointPositions
} // namespace calibration


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::calibration::GetJointPositions > {
  static const char* value()
  {
    return "a286ff40b196573b9ebf3999a2f8d438";
  }

  static const char* value(const ::calibration::GetJointPositions&) { return value(); }
};

template<>
struct DataType< ::calibration::GetJointPositions > {
  static const char* value()
  {
    return "calibration/GetJointPositions";
  }

  static const char* value(const ::calibration::GetJointPositions&) { return value(); }
};


// service_traits::MD5Sum< ::calibration::GetJointPositionsRequest> should match 
// service_traits::MD5Sum< ::calibration::GetJointPositions > 
template<>
struct MD5Sum< ::calibration::GetJointPositionsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::calibration::GetJointPositions >::value();
  }
  static const char* value(const ::calibration::GetJointPositionsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::calibration::GetJointPositionsRequest> should match 
// service_traits::DataType< ::calibration::GetJointPositions > 
template<>
struct DataType< ::calibration::GetJointPositionsRequest>
{
  static const char* value()
  {
    return DataType< ::calibration::GetJointPositions >::value();
  }
  static const char* value(const ::calibration::GetJointPositionsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::calibration::GetJointPositionsResponse> should match 
// service_traits::MD5Sum< ::calibration::GetJointPositions > 
template<>
struct MD5Sum< ::calibration::GetJointPositionsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::calibration::GetJointPositions >::value();
  }
  static const char* value(const ::calibration::GetJointPositionsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::calibration::GetJointPositionsResponse> should match 
// service_traits::DataType< ::calibration::GetJointPositions > 
template<>
struct DataType< ::calibration::GetJointPositionsResponse>
{
  static const char* value()
  {
    return DataType< ::calibration::GetJointPositions >::value();
  }
  static const char* value(const ::calibration::GetJointPositionsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CALIBRATION_MESSAGE_GETJOINTPOSITIONS_H
