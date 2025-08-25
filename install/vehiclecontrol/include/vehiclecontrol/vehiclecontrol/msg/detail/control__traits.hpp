// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vehiclecontrol:msg/Control.idl
// generated code does not contain a copyright notice

#ifndef VEHICLECONTROL__MSG__DETAIL__CONTROL__TRAITS_HPP_
#define VEHICLECONTROL__MSG__DETAIL__CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vehiclecontrol/msg/detail/control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vehiclecontrol
{

namespace msg
{

inline void to_flow_style_yaml(
  const Control & msg,
  std::ostream & out)
{
  out << "{";
  // member: steering
  {
    out << "steering: ";
    rosidl_generator_traits::value_to_yaml(msg.steering, out);
    out << ", ";
  }

  // member: throttle
  {
    out << "throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle, out);
    out << ", ";
  }

  // member: brake
  {
    out << "brake: ";
    rosidl_generator_traits::value_to_yaml(msg.brake, out);
    out << ", ";
  }

  // member: longswitch
  {
    out << "longswitch: ";
    rosidl_generator_traits::value_to_yaml(msg.longswitch, out);
    out << ", ";
  }

  // member: latswitch
  {
    out << "latswitch: ";
    rosidl_generator_traits::value_to_yaml(msg.latswitch, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Control & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: steering
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "steering: ";
    rosidl_generator_traits::value_to_yaml(msg.steering, out);
    out << "\n";
  }

  // member: throttle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle, out);
    out << "\n";
  }

  // member: brake
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "brake: ";
    rosidl_generator_traits::value_to_yaml(msg.brake, out);
    out << "\n";
  }

  // member: longswitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longswitch: ";
    rosidl_generator_traits::value_to_yaml(msg.longswitch, out);
    out << "\n";
  }

  // member: latswitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latswitch: ";
    rosidl_generator_traits::value_to_yaml(msg.latswitch, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Control & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace vehiclecontrol

namespace rosidl_generator_traits
{

[[deprecated("use vehiclecontrol::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vehiclecontrol::msg::Control & msg,
  std::ostream & out, size_t indentation = 0)
{
  vehiclecontrol::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vehiclecontrol::msg::to_yaml() instead")]]
inline std::string to_yaml(const vehiclecontrol::msg::Control & msg)
{
  return vehiclecontrol::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vehiclecontrol::msg::Control>()
{
  return "vehiclecontrol::msg::Control";
}

template<>
inline const char * name<vehiclecontrol::msg::Control>()
{
  return "vehiclecontrol/msg/Control";
}

template<>
struct has_fixed_size<vehiclecontrol::msg::Control>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vehiclecontrol::msg::Control>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vehiclecontrol::msg::Control>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VEHICLECONTROL__MSG__DETAIL__CONTROL__TRAITS_HPP_
