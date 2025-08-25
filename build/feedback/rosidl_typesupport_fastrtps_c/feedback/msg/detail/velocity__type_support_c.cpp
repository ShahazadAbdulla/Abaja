// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from feedback:msg/Velocity.idl
// generated code does not contain a copyright notice
#include "feedback/msg/detail/velocity__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "feedback/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "feedback/msg/detail/velocity__struct.h"
#include "feedback/msg/detail/velocity__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _Velocity__ros_msg_type = feedback__msg__Velocity;

static bool _Velocity__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Velocity__ros_msg_type * ros_message = static_cast<const _Velocity__ros_msg_type *>(untyped_ros_message);
  // Field name: vehicle_velocity
  {
    cdr << ros_message->vehicle_velocity;
  }

  // Field name: wheelrpm_fl
  {
    cdr << ros_message->wheelrpm_fl;
  }

  // Field name: wheelrpm_fr
  {
    cdr << ros_message->wheelrpm_fr;
  }

  // Field name: wheelrpm_rl
  {
    cdr << ros_message->wheelrpm_rl;
  }

  // Field name: wheelrpm_rr
  {
    cdr << ros_message->wheelrpm_rr;
  }

  return true;
}

static bool _Velocity__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Velocity__ros_msg_type * ros_message = static_cast<_Velocity__ros_msg_type *>(untyped_ros_message);
  // Field name: vehicle_velocity
  {
    cdr >> ros_message->vehicle_velocity;
  }

  // Field name: wheelrpm_fl
  {
    cdr >> ros_message->wheelrpm_fl;
  }

  // Field name: wheelrpm_fr
  {
    cdr >> ros_message->wheelrpm_fr;
  }

  // Field name: wheelrpm_rl
  {
    cdr >> ros_message->wheelrpm_rl;
  }

  // Field name: wheelrpm_rr
  {
    cdr >> ros_message->wheelrpm_rr;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_feedback
size_t get_serialized_size_feedback__msg__Velocity(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Velocity__ros_msg_type * ros_message = static_cast<const _Velocity__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name vehicle_velocity
  {
    size_t item_size = sizeof(ros_message->vehicle_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name wheelrpm_fl
  {
    size_t item_size = sizeof(ros_message->wheelrpm_fl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name wheelrpm_fr
  {
    size_t item_size = sizeof(ros_message->wheelrpm_fr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name wheelrpm_rl
  {
    size_t item_size = sizeof(ros_message->wheelrpm_rl);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name wheelrpm_rr
  {
    size_t item_size = sizeof(ros_message->wheelrpm_rr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Velocity__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_feedback__msg__Velocity(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_feedback
size_t max_serialized_size_feedback__msg__Velocity(
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

  // member: vehicle_velocity
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: wheelrpm_fl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: wheelrpm_fr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: wheelrpm_rl
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: wheelrpm_rr
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
    using DataType = feedback__msg__Velocity;
    is_plain =
      (
      offsetof(DataType, wheelrpm_rr) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Velocity__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_feedback__msg__Velocity(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Velocity = {
  "feedback::msg",
  "Velocity",
  _Velocity__cdr_serialize,
  _Velocity__cdr_deserialize,
  _Velocity__get_serialized_size,
  _Velocity__max_serialized_size
};

static rosidl_message_type_support_t _Velocity__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Velocity,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, feedback, msg, Velocity)() {
  return &_Velocity__type_support;
}

#if defined(__cplusplus)
}
#endif
