// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inertial_msgs:msg/Pose.idl
// generated code does not contain a copyright notice

#ifndef INERTIAL_MSGS__MSG__DETAIL__POSE__BUILDER_HPP_
#define INERTIAL_MSGS__MSG__DETAIL__POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inertial_msgs/msg/detail/pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inertial_msgs
{

namespace msg
{

namespace builder
{

class Init_Pose_linear_acceleration
{
public:
  explicit Init_Pose_linear_acceleration(::inertial_msgs::msg::Pose & msg)
  : msg_(msg)
  {}
  ::inertial_msgs::msg::Pose linear_acceleration(::inertial_msgs::msg::Pose::_linear_acceleration_type arg)
  {
    msg_.linear_acceleration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_msgs::msg::Pose msg_;
};

class Init_Pose_angular_velocity
{
public:
  explicit Init_Pose_angular_velocity(::inertial_msgs::msg::Pose & msg)
  : msg_(msg)
  {}
  Init_Pose_linear_acceleration angular_velocity(::inertial_msgs::msg::Pose::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_Pose_linear_acceleration(msg_);
  }

private:
  ::inertial_msgs::msg::Pose msg_;
};

class Init_Pose_orientation
{
public:
  explicit Init_Pose_orientation(::inertial_msgs::msg::Pose & msg)
  : msg_(msg)
  {}
  Init_Pose_angular_velocity orientation(::inertial_msgs::msg::Pose::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_Pose_angular_velocity(msg_);
  }

private:
  ::inertial_msgs::msg::Pose msg_;
};

class Init_Pose_velocity
{
public:
  explicit Init_Pose_velocity(::inertial_msgs::msg::Pose & msg)
  : msg_(msg)
  {}
  Init_Pose_orientation velocity(::inertial_msgs::msg::Pose::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_Pose_orientation(msg_);
  }

private:
  ::inertial_msgs::msg::Pose msg_;
};

class Init_Pose_position
{
public:
  Init_Pose_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pose_velocity position(::inertial_msgs::msg::Pose::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Pose_velocity(msg_);
  }

private:
  ::inertial_msgs::msg::Pose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_msgs::msg::Pose>()
{
  return inertial_msgs::msg::builder::Init_Pose_position();
}

}  // namespace inertial_msgs

#endif  // INERTIAL_MSGS__MSG__DETAIL__POSE__BUILDER_HPP_
