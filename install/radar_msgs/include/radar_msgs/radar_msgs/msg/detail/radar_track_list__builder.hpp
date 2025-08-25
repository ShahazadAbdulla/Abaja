// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from radar_msgs:msg/RadarTrackList.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSGS__MSG__DETAIL__RADAR_TRACK_LIST__BUILDER_HPP_
#define RADAR_MSGS__MSG__DETAIL__RADAR_TRACK_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "radar_msgs/msg/detail/radar_track_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace radar_msgs
{

namespace msg
{

namespace builder
{

class Init_RadarTrackList_objects
{
public:
  Init_RadarTrackList_objects()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::radar_msgs::msg::RadarTrackList objects(::radar_msgs::msg::RadarTrackList::_objects_type arg)
  {
    msg_.objects = std::move(arg);
    return std::move(msg_);
  }

private:
  ::radar_msgs::msg::RadarTrackList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::radar_msgs::msg::RadarTrackList>()
{
  return radar_msgs::msg::builder::Init_RadarTrackList_objects();
}

}  // namespace radar_msgs

#endif  // RADAR_MSGS__MSG__DETAIL__RADAR_TRACK_LIST__BUILDER_HPP_
