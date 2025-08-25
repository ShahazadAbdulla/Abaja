// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from radar_msgs:msg/RadarTrack.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSGS__MSG__DETAIL__RADAR_TRACK__TRAITS_HPP_
#define RADAR_MSGS__MSG__DETAIL__RADAR_TRACK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "radar_msgs/msg/detail/radar_track__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace radar_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RadarTrack & msg,
  std::ostream & out)
{
  out << "{";
  // member: tracking_id
  {
    out << "tracking_id: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking_id, out);
    out << ", ";
  }

  // member: x_distance
  {
    out << "x_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.x_distance, out);
    out << ", ";
  }

  // member: y_distance
  {
    out << "y_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.y_distance, out);
    out << ", ";
  }

  // member: vx
  {
    out << "vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vx, out);
    out << ", ";
  }

  // member: vy
  {
    out << "vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vy, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RadarTrack & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: tracking_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tracking_id: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking_id, out);
    out << "\n";
  }

  // member: x_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.x_distance, out);
    out << "\n";
  }

  // member: y_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.y_distance, out);
    out << "\n";
  }

  // member: vx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vx, out);
    out << "\n";
  }

  // member: vy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vy, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RadarTrack & msg, bool use_flow_style = false)
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

}  // namespace radar_msgs

namespace rosidl_generator_traits
{

[[deprecated("use radar_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const radar_msgs::msg::RadarTrack & msg,
  std::ostream & out, size_t indentation = 0)
{
  radar_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use radar_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const radar_msgs::msg::RadarTrack & msg)
{
  return radar_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<radar_msgs::msg::RadarTrack>()
{
  return "radar_msgs::msg::RadarTrack";
}

template<>
inline const char * name<radar_msgs::msg::RadarTrack>()
{
  return "radar_msgs/msg/RadarTrack";
}

template<>
struct has_fixed_size<radar_msgs::msg::RadarTrack>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<radar_msgs::msg::RadarTrack>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<radar_msgs::msg::RadarTrack>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RADAR_MSGS__MSG__DETAIL__RADAR_TRACK__TRAITS_HPP_
