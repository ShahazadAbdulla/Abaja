// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inertial_msgs:msg/Pose.idl
// generated code does not contain a copyright notice

#ifndef INERTIAL_MSGS__MSG__DETAIL__POSE__STRUCT_H_
#define INERTIAL_MSGS__MSG__DETAIL__POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity'
// Member 'orientation'
// Member 'angular_velocity'
// Member 'linear_acceleration'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Pose in the package inertial_msgs.
/**
  * Pose.msg
  *
  * This message contains inertial sensor data (pose, velocity, orientation, etc.)
  *
  * Position in X, Y, Z from InertialSensor[Nb].Pos_0[0..2]
 */
typedef struct inertial_msgs__msg__Pose
{
  geometry_msgs__msg__Point position;
  /// Velocity from InertialSensor[Nb].Vel_0[0..1] and Vehicle.v (for the Z component)
  geometry_msgs__msg__Vector3 velocity;
  /// Orientation in X, Y, Z from Vhcl.Pitch, Vhcl.Roll, Vhcl.Yaw
  geometry_msgs__msg__Vector3 orientation;
  /// Angular velocity (for example, using Vhcl.PitchVel, Vhcl.RollVel, Vhcl.YawRate or InertialSensor[Nb].Omega_0[0..2])
  geometry_msgs__msg__Vector3 angular_velocity;
  /// Linear acceleration in X, Y, Z
  geometry_msgs__msg__Vector3 linear_acceleration;
} inertial_msgs__msg__Pose;

// Struct for a sequence of inertial_msgs__msg__Pose.
typedef struct inertial_msgs__msg__Pose__Sequence
{
  inertial_msgs__msg__Pose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inertial_msgs__msg__Pose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INERTIAL_MSGS__MSG__DETAIL__POSE__STRUCT_H_
