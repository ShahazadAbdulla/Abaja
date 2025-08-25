// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from feedback:msg/Velocity.idl
// generated code does not contain a copyright notice
#include "feedback/msg/detail/velocity__rosidl_typesupport_fastrtps_cpp.hpp"
#include "feedback/msg/detail/velocity__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace feedback
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_feedback
cdr_serialize(
  const feedback::msg::Velocity & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: vehicle_velocity
  cdr << ros_message.vehicle_velocity;
  // Member: wheelrpm_fl
  cdr << ros_message.wheelrpm_fl;
  // Member: wheelrpm_fr
  cdr << ros_message.wheelrpm_fr;
  // Member: wheelrpm_rl
  cdr << ros_message.wheelrpm_rl;
  // Member: wheelrpm_rr
  cdr << ros_message.wheelrpm_rr;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_feedback
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  feedback::msg::Velocity & ros_message)
{
  // Member: vehicle_velocity
  cdr >> ros_message.vehicle_velocity;

  // Member: wheelrpm_fl
  cdr >> ros_message.wheelrpm_fl;

  // Member: wheelrpm_fr
  cdr >> ros_message.wheelrpm_fr;

  // Member: wheelrpm_rl
  cdr >> ros_message.wheelrpm_rl;

  // Member: wheelrpm_rr
  cdr >> ros_message.wheelrpm_rr;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_feedback
get_serialized_size(
  const feedback::msg::Velocity & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: vehicle_velocity
  {
    size_t item_size = sizeof(ros_message.vehicle_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: wheelrpm_fl
  {
    size_t item_size = sizeof(ros_message.wheelrpm_fl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: wheelrpm_fr
  {
    size_t item_size = sizeof(ros_message.wheelrpm_fr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: wheelrpm_rl
  {
    size_t item_size = sizeof(ros_message.wheelrpm_rl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: wheelrpm_rr
  {
    size_t item_size = sizeof(ros_message.wheelrpm_rr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_feedback
max_serialized_size_Velocity(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: vehicle_velocity
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: wheelrpm_fl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: wheelrpm_fr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: wheelrpm_rl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: wheelrpm_rr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = feedback::msg::Velocity;
    is_plain =
      (
      offsetof(DataType, wheelrpm_rr) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Velocity__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const feedback::msg::Velocity *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Velocity__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<feedback::msg::Velocity *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Velocity__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const feedback::msg::Velocity *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Velocity__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Velocity(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Velocity__callbacks = {
  "feedback::msg",
  "Velocity",
  _Velocity__cdr_serialize,
  _Velocity__cdr_deserialize,
  _Velocity__get_serialized_size,
  _Velocity__max_serialized_size
};

static rosidl_message_type_support_t _Velocity__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Velocity__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace feedback

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_feedback
const rosidl_message_type_support_t *
get_message_type_support_handle<feedback::msg::Velocity>()
{
  return &feedback::msg::typesupport_fastrtps_cpp::_Velocity__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, feedback, msg, Velocity)() {
  return &feedback::msg::typesupport_fastrtps_cpp::_Velocity__handle;
}

#ifdef __cplusplus
}
#endif
