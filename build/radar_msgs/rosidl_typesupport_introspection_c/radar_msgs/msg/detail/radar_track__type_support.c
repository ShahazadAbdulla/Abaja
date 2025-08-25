// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from radar_msgs:msg/RadarTrack.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "radar_msgs/msg/detail/radar_track__rosidl_typesupport_introspection_c.h"
#include "radar_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "radar_msgs/msg/detail/radar_track__functions.h"
#include "radar_msgs/msg/detail/radar_track__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void radar_msgs__msg__RadarTrack__rosidl_typesupport_introspection_c__RadarTrack_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  radar_msgs__msg__RadarTrack__init(message_memory);
}

void radar_msgs__msg__RadarTrack__rosidl_typesupport_introspection_c__RadarTrack_fini_function(void * message_memory)
{
  radar_msgs__msg__RadarTrack__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember radar_msgs__msg__RadarTrack__rosidl_typesupport_introspection_c__RadarTrack_message_member_array[5] = {
  {
    "tracking_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msgs__msg__RadarTrack, tracking_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x_distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msgs__msg__RadarTrack, x_distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y_distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msgs__msg__RadarTrack, y_distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msgs__msg__RadarTrack, vx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msgs__msg__RadarTrack, vy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers radar_msgs__msg__RadarTrack__rosidl_typesupport_introspection_c__RadarTrack_message_members = {
  "radar_msgs__msg",  // message namespace
  "RadarTrack",  // message name
  5,  // number of fields
  sizeof(radar_msgs__msg__RadarTrack),
  radar_msgs__msg__RadarTrack__rosidl_typesupport_introspection_c__RadarTrack_message_member_array,  // message members
  radar_msgs__msg__RadarTrack__rosidl_typesupport_introspection_c__RadarTrack_init_function,  // function to initialize message memory (memory has to be allocated)
  radar_msgs__msg__RadarTrack__rosidl_typesupport_introspection_c__RadarTrack_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t radar_msgs__msg__RadarTrack__rosidl_typesupport_introspection_c__RadarTrack_message_type_support_handle = {
  0,
  &radar_msgs__msg__RadarTrack__rosidl_typesupport_introspection_c__RadarTrack_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_radar_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, radar_msgs, msg, RadarTrack)() {
  if (!radar_msgs__msg__RadarTrack__rosidl_typesupport_introspection_c__RadarTrack_message_type_support_handle.typesupport_identifier) {
    radar_msgs__msg__RadarTrack__rosidl_typesupport_introspection_c__RadarTrack_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &radar_msgs__msg__RadarTrack__rosidl_typesupport_introspection_c__RadarTrack_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
