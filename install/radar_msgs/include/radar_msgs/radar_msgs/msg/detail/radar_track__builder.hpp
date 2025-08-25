// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from radar_msgs:msg/RadarTrack.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSGS__MSG__DETAIL__RADAR_TRACK__BUILDER_HPP_
#define RADAR_MSGS__MSG__DETAIL__RADAR_TRACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "radar_msgs/msg/detail/radar_track__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace radar_msgs
{

namespace msg
{

namespace builder
{

class Init_RadarTrack_vy
{
public:
  explicit Init_RadarTrack_vy(::radar_msgs::msg::RadarTrack & msg)
  : msg_(msg)
  {}
  ::radar_msgs::msg::RadarTrack vy(::radar_msgs::msg::RadarTrack::_vy_type arg)
  {
    msg_.vy = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msgs::msg::RadarTrack msg_;
};

class Init_RadarTrack_vx
{
public:
  explicit Init_RadarTrack_vx(::radar_msgs::msg::RadarTrack & msg)
  : msg_(msg)
  {}
  Init_RadarTrack_vy vx(::radar_msgs::msg::RadarTrack::_vx_type arg)
  {
    msg_.vx = std::move(arg);
    return Init_RadarTrack_vy(msg_);
  }

private:
  ::radar_msgs::msg::RadarTrack msg_;
};

class Init_RadarTrack_y_distance
{
public:
  explicit Init_RadarTrack_y_distance(::radar_msgs::msg::RadarTrack & msg)
  : msg_(msg)
  {}
  Init_RadarTrack_vx y_distance(::radar_msgs::msg::RadarTrack::_y_distance_type arg)
  {
    msg_.y_distance = std::move(arg);
    return Init_RadarTrack_vx(msg_);
  }

private:
  ::radar_msgs::msg::RadarTrack msg_;
};

class Init_RadarTrack_x_distance
{
public:
  explicit Init_RadarTrack_x_distance(::radar_msgs::msg::RadarTrack & msg)
  : msg_(msg)
  {}
  Init_RadarTrack_y_distance x_distance(::radar_msgs::msg::RadarTrack::_x_distance_type arg)
  {
    msg_.x_distance = std::move(arg);
    return Init_RadarTrack_y_distance(msg_);
  }

private:
  ::radar_msgs::msg::RadarTrack msg_;
};

class Init_RadarTrack_tracking_id
{
public:
  Init_RadarTrack_tracking_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RadarTrack_x_distance tracking_id(::radar_msgs::msg::RadarTrack::_tracking_id_type arg)
  {
    msg_.tracking_id = std::move(arg);
    return Init_RadarTrack_x_distance(msg_);
  }

private:
  ::radar_msgs::msg::RadarTrack msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msgs::msg::RadarTrack>()
{
  return radar_msgs::msg::builder::Init_RadarTrack_tracking_id();
}

}  // namespace radar_msgs

#endif  // RADAR_MSGS__MSG__DETAIL__RADAR_TRACK__BUILDER_HPP_
