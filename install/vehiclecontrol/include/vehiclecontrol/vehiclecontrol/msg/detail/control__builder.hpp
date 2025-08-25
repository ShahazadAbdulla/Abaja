// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vehiclecontrol:msg/Control.idl
// generated code does not contain a copyright notice

#ifndef VEHICLECONTROL__MSG__DETAIL__CONTROL__BUILDER_HPP_
#define VEHICLECONTROL__MSG__DETAIL__CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vehiclecontrol/msg/detail/control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vehiclecontrol
{

namespace msg
{

namespace builder
{

class Init_Control_latswitch
{
public:
  explicit Init_Control_latswitch(::vehiclecontrol::msg::Control & msg)
  : msg_(msg)
  {}
  ::vehiclecontrol::msg::Control latswitch(::vehiclecontrol::msg::Control::_latswitch_type arg)
  {
    msg_.latswitch = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vehiclecontrol::msg::Control msg_;
};

class Init_Control_longswitch
{
public:
  explicit Init_Control_longswitch(::vehiclecontrol::msg::Control & msg)
  : msg_(msg)
  {}
  Init_Control_latswitch longswitch(::vehiclecontrol::msg::Control::_longswitch_type arg)
  {
    msg_.longswitch = std::move(arg);
    return Init_Control_latswitch(msg_);
  }

private:
  ::vehiclecontrol::msg::Control msg_;
};

class Init_Control_brake
{
public:
  explicit Init_Control_brake(::vehiclecontrol::msg::Control & msg)
  : msg_(msg)
  {}
  Init_Control_longswitch brake(::vehiclecontrol::msg::Control::_brake_type arg)
  {
    msg_.brake = std::move(arg);
    return Init_Control_longswitch(msg_);
  }

private:
  ::vehiclecontrol::msg::Control msg_;
};

class Init_Control_throttle
{
public:
  explicit Init_Control_throttle(::vehiclecontrol::msg::Control & msg)
  : msg_(msg)
  {}
  Init_Control_brake throttle(::vehiclecontrol::msg::Control::_throttle_type arg)
  {
    msg_.throttle = std::move(arg);
    return Init_Control_brake(msg_);
  }

private:
  ::vehiclecontrol::msg::Control msg_;
};

class Init_Control_steering
{
public:
  Init_Control_steering()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Control_throttle steering(::vehiclecontrol::msg::Control::_steering_type arg)
  {
    msg_.steering = std::move(arg);
    return Init_Control_throttle(msg_);
  }

private:
  ::vehiclecontrol::msg::Control msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vehiclecontrol::msg::Control>()
{
  return vehiclecontrol::msg::builder::Init_Control_steering();
}

}  // namespace vehiclecontrol

#endif  // VEHICLECONTROL__MSG__DETAIL__CONTROL__BUILDER_HPP_
