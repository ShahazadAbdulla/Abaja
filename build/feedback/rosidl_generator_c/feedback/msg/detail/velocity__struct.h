// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from feedback:msg/Velocity.idl
// generated code does not contain a copyright notice

#ifndef FEEDBACK__MSG__DETAIL__VELOCITY__STRUCT_H_
#define FEEDBACK__MSG__DETAIL__VELOCITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Velocity in the package feedback.
typedef struct feedback__msg__Velocity
{
  /// Vhcl.v
  float vehicle_velocity;
  /// Car.WheelSpd_FL
  float wheelrpm_fl;
  /// Car.WheelSpd_FR
  float wheelrpm_fr;
  /// Car.WheelSpd_RL
  float wheelrpm_rl;
  /// Car.WheelSpd_RR
  float wheelrpm_rr;
} feedback__msg__Velocity;

// Struct for a sequence of feedback__msg__Velocity.
typedef struct feedback__msg__Velocity__Sequence
{
  feedback__msg__Velocity * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} feedback__msg__Velocity__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FEEDBACK__MSG__DETAIL__VELOCITY__STRUCT_H_
