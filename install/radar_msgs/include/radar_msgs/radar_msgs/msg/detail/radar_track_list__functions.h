// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from radar_msgs:msg/RadarTrackList.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSGS__MSG__DETAIL__RADAR_TRACK_LIST__FUNCTIONS_H_
#define RADAR_MSGS__MSG__DETAIL__RADAR_TRACK_LIST__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "radar_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "radar_msgs/msg/detail/radar_track_list__struct.h"

/// Initialize msg/RadarTrackList message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * radar_msgs__msg__RadarTrackList
 * )) before or use
 * radar_msgs__msg__RadarTrackList__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msgs
bool
radar_msgs__msg__RadarTrackList__init(radar_msgs__msg__RadarTrackList * msg);

/// Finalize msg/RadarTrackList message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msgs
void
radar_msgs__msg__RadarTrackList__fini(radar_msgs__msg__RadarTrackList * msg);

/// Create msg/RadarTrackList message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * radar_msgs__msg__RadarTrackList__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msgs
radar_msgs__msg__RadarTrackList *
radar_msgs__msg__RadarTrackList__create();

/// Destroy msg/RadarTrackList message.
/**
 * It calls
 * radar_msgs__msg__RadarTrackList__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msgs
void
radar_msgs__msg__RadarTrackList__destroy(radar_msgs__msg__RadarTrackList * msg);

/// Check for msg/RadarTrackList message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msgs
bool
radar_msgs__msg__RadarTrackList__are_equal(const radar_msgs__msg__RadarTrackList * lhs, const radar_msgs__msg__RadarTrackList * rhs);

/// Copy a msg/RadarTrackList message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msgs
bool
radar_msgs__msg__RadarTrackList__copy(
  const radar_msgs__msg__RadarTrackList * input,
  radar_msgs__msg__RadarTrackList * output);

/// Initialize array of msg/RadarTrackList messages.
/**
 * It allocates the memory for the number of elements and calls
 * radar_msgs__msg__RadarTrackList__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msgs
bool
radar_msgs__msg__RadarTrackList__Sequence__init(radar_msgs__msg__RadarTrackList__Sequence * array, size_t size);

/// Finalize array of msg/RadarTrackList messages.
/**
 * It calls
 * radar_msgs__msg__RadarTrackList__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msgs
void
radar_msgs__msg__RadarTrackList__Sequence__fini(radar_msgs__msg__RadarTrackList__Sequence * array);

/// Create array of msg/RadarTrackList messages.
/**
 * It allocates the memory for the array and calls
 * radar_msgs__msg__RadarTrackList__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msgs
radar_msgs__msg__RadarTrackList__Sequence *
radar_msgs__msg__RadarTrackList__Sequence__create(size_t size);

/// Destroy array of msg/RadarTrackList messages.
/**
 * It calls
 * radar_msgs__msg__RadarTrackList__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msgs
void
radar_msgs__msg__RadarTrackList__Sequence__destroy(radar_msgs__msg__RadarTrackList__Sequence * array);

/// Check for msg/RadarTrackList message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msgs
bool
radar_msgs__msg__RadarTrackList__Sequence__are_equal(const radar_msgs__msg__RadarTrackList__Sequence * lhs, const radar_msgs__msg__RadarTrackList__Sequence * rhs);

/// Copy an array of msg/RadarTrackList messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_radar_msgs
bool
radar_msgs__msg__RadarTrackList__Sequence__copy(
  const radar_msgs__msg__RadarTrackList__Sequence * input,
  radar_msgs__msg__RadarTrackList__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // RADAR_MSGS__MSG__DETAIL__RADAR_TRACK_LIST__FUNCTIONS_H_
