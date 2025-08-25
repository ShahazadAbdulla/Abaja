// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from radar_msgs:msg/RadarTrackList.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSGS__MSG__DETAIL__RADAR_TRACK_LIST__STRUCT_H_
#define RADAR_MSGS__MSG__DETAIL__RADAR_TRACK_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'objects'
#include "radar_msgs/msg/detail/radar_track__struct.h"

/// Struct defined in msg/RadarTrackList in the package radar_msgs.
typedef struct radar_msgs__msg__RadarTrackList
{
  radar_msgs__msg__RadarTrack__Sequence objects;
} radar_msgs__msg__RadarTrackList;

// Struct for a sequence of radar_msgs__msg__RadarTrackList.
typedef struct radar_msgs__msg__RadarTrackList__Sequence
{
  radar_msgs__msg__RadarTrackList * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} radar_msgs__msg__RadarTrackList__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RADAR_MSGS__MSG__DETAIL__RADAR_TRACK_LIST__STRUCT_H_
