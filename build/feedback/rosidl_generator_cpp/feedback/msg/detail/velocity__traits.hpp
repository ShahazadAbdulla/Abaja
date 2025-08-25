// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from feedback:msg/Velocity.idl
// generated code does not contain a copyright notice

#ifndef FEEDBACK__MSG__DETAIL__VELOCITY__TRAITS_HPP_
#define FEEDBACK__MSG__DETAIL__VELOCITY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "feedback/msg/detail/velocity__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace feedback
{

namespace msg
{

inline void to_flow_style_yaml(
  const Velocity & msg,
  std::ostream & out)
{
  out << "{";
  // member: vehicle_velocity
  {
    out << "vehicle_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_velocity, out);
    out << ", ";
  }

  // member: wheelrpm_fl
  {
    out << "wheelrpm_fl: ";
    rosidl_generator_traits::value_to_yaml(msg.wheelrpm_fl, out);
    out << ", ";
  }

  // member: wheelrpm_fr
  {
    out << "wheelrpm_fr: ";
    rosidl_generator_traits::value_to_yaml(msg.wheelrpm_fr, out);
    out << ", ";
  }

  // member: wheelrpm_rl
  {
    out << "wheelrpm_rl: ";
    rosidl_generator_traits::value_to_yaml(msg.wheelrpm_rl, out);
    out << ", ";
  }

  // member: wheelrpm_rr
  {
    out << "wheelrpm_rr: ";
    rosidl_generator_traits::value_to_yaml(msg.wheelrpm_rr, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Velocity & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: vehicle_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vehicle_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_velocity, out);
    out << "\n";
  }

  // member: wheelrpm_fl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wheelrpm_fl: ";
    rosidl_generator_traits::value_to_yaml(msg.wheelrpm_fl, out);
    out << "\n";
  }

  // member: wheelrpm_fr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wheelrpm_fr: ";
    rosidl_generator_traits::value_to_yaml(msg.wheelrpm_fr, out);
    out << "\n";
  }

  // member: wheelrpm_rl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wheelrpm_rl: ";
    rosidl_generator_traits::value_to_yaml(msg.wheelrpm_rl, out);
    out << "\n";
  }

  // member: wheelrpm_rr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wheelrpm_rr: ";
    rosidl_generator_traits::value_to_yaml(msg.wheelrpm_rr, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Velocity & msg, bool use_flow_style = false)
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

}  // namespace feedback

namespace rosidl_generator_traits
{

[[deprecated("use feedback::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const feedback::msg::Velocity & msg,
  std::ostream & out, size_t indentation = 0)
{
  feedback::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use feedback::msg::to_yaml() instead")]]
inline std::string to_yaml(const feedback::msg::Velocity & msg)
{
  return feedback::msg::to_yaml(msg);
}

template<>
inline const char * data_type<feedback::msg::Velocity>()
{
  return "feedback::msg::Velocity";
}

template<>
inline const char * name<feedback::msg::Velocity>()
{
  return "feedback/msg/Velocity";
}

template<>
struct has_fixed_size<feedback::msg::Velocity>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<feedback::msg::Velocity>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<feedback::msg::Velocity>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // FEEDBACK__MSG__DETAIL__VELOCITY__TRAITS_HPP_
