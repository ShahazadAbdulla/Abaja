// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from feedback:msg/Velocity.idl
// generated code does not contain a copyright notice

#ifndef FEEDBACK__MSG__DETAIL__VELOCITY__BUILDER_HPP_
#define FEEDBACK__MSG__DETAIL__VELOCITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "feedback/msg/detail/velocity__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace feedback
{

namespace msg
{

namespace builder
{

class Init_Velocity_wheelrpm_rr
{
public:
  explicit Init_Velocity_wheelrpm_rr(::feedback::msg::Velocity & msg)
  : msg_(msg)
  {}
  ::feedback::msg::Velocity wheelrpm_rr(::feedback::msg::Velocity::_wheelrpm_rr_type arg)
  {
    msg_.wheelrpm_rr = std::move(arg);
    return std::move(msg_);
  }

private:
  ::feedback::msg::Velocity msg_;
};

class Init_Velocity_wheelrpm_rl
{
public:
  explicit Init_Velocity_wheelrpm_rl(::feedback::msg::Velocity & msg)
  : msg_(msg)
  {}
  Init_Velocity_wheelrpm_rr wheelrpm_rl(::feedback::msg::Velocity::_wheelrpm_rl_type arg)
  {
    msg_.wheelrpm_rl = std::move(arg);
    return Init_Velocity_wheelrpm_rr(msg_);
  }

private:
  ::feedback::msg::Velocity msg_;
};

class Init_Velocity_wheelrpm_fr
{
public:
  explicit Init_Velocity_wheelrpm_fr(::feedback::msg::Velocity & msg)
  : msg_(msg)
  {}
  Init_Velocity_wheelrpm_rl wheelrpm_fr(::feedback::msg::Velocity::_wheelrpm_fr_type arg)
  {
    msg_.wheelrpm_fr = std::move(arg);
    return Init_Velocity_wheelrpm_rl(msg_);
  }

private:
  ::feedback::msg::Velocity msg_;
};

class Init_Velocity_wheelrpm_fl
{
public:
  explicit Init_Velocity_wheelrpm_fl(::feedback::msg::Velocity & msg)
  : msg_(msg)
  {}
  Init_Velocity_wheelrpm_fr wheelrpm_fl(::feedback::msg::Velocity::_wheelrpm_fl_type arg)
  {
    msg_.wheelrpm_fl = std::move(arg);
    return Init_Velocity_wheelrpm_fr(msg_);
  }

private:
  ::feedback::msg::Velocity msg_;
};

class Init_Velocity_vehicle_velocity
{
public:
  Init_Velocity_vehicle_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Velocity_wheelrpm_fl vehicle_velocity(::feedback::msg::Velocity::_vehicle_velocity_type arg)
  {
    msg_.vehicle_velocity = std::move(arg);
    return Init_Velocity_wheelrpm_fl(msg_);
  }

private:
  ::feedback::msg::Velocity msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::feedback::msg::Velocity>()
{
  return feedback::msg::builder::Init_Velocity_vehicle_velocity();
}

}  // namespace feedback

#endif  // FEEDBACK__MSG__DETAIL__VELOCITY__BUILDER_HPP_
