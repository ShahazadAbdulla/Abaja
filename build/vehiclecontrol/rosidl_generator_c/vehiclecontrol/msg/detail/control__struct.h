// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vehiclecontrol:msg/Control.idl
// generated code does not contain a copyright notice

#ifndef VEHICLECONTROL__MSG__DETAIL__CONTROL__STRUCT_H_
#define VEHICLECONTROL__MSG__DETAIL__CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Control in the package vehiclecontrol.
typedef struct vehiclecontrol__msg__Control
{
  float steering;
  float throttle;
  float brake;
  int8_t longswitch;
  int8_t latswitch;
} vehiclecontrol__msg__Control;

// Struct for a sequence of vehiclecontrol__msg__Control.
typedef struct vehiclecontrol__msg__Control__Sequence
{
  vehiclecontrol__msg__Control * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vehiclecontrol__msg__Control__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VEHICLECONTROL__MSG__DETAIL__CONTROL__STRUCT_H_
