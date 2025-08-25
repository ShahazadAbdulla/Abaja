// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from radar_msgs:msg/RadarTrack.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSGS__MSG__DETAIL__RADAR_TRACK__STRUCT_H_
#define RADAR_MSGS__MSG__DETAIL__RADAR_TRACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RadarTrack in the package radar_msgs.
typedef struct radar_msgs__msg__RadarTrack
{
  int32_t tracking_id;
  float x_distance;
  float y_distance;
  float vx;
  float vy;
} radar_msgs__msg__RadarTrack;

// Struct for a sequence of radar_msgs__msg__RadarTrack.
typedef struct radar_msgs__msg__RadarTrack__Sequence
{
  radar_msgs__msg__RadarTrack * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msgs__msg__RadarTrack__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RADAR_MSGS__MSG__DETAIL__RADAR_TRACK__STRUCT_H_
