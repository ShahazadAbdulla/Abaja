// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from radar_msgs:msg/RadarTrack.idl
// generated code does not contain a copyright notice
#include "radar_msgs/msg/detail/radar_track__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
radar_msgs__msg__RadarTrack__init(radar_msgs__msg__RadarTrack * msg)
{
  if (!msg) {
    return false;
  }
  // tracking_id
  // x_distance
  // y_distance
  // vx
  // vy
  return true;
}

void
radar_msgs__msg__RadarTrack__fini(radar_msgs__msg__RadarTrack * msg)
{
  if (!msg) {
    return;
  }
  // tracking_id
  // x_distance
  // y_distance
  // vx
  // vy
}

bool
radar_msgs__msg__RadarTrack__are_equal(const radar_msgs__msg__RadarTrack * lhs, const radar_msgs__msg__RadarTrack * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // tracking_id
  if (lhs->tracking_id != rhs->tracking_id) {
    return false;
  }
  // x_distance
  if (lhs->x_distance != rhs->x_distance) {
    return false;
  }
  // y_distance
  if (lhs->y_distance != rhs->y_distance) {
    return false;
  }
  // vx
  if (lhs->vx != rhs->vx) {
    return false;
  }
  // vy
  if (lhs->vy != rhs->vy) {
    return false;
  }
  return true;
}

bool
radar_msgs__msg__RadarTrack__copy(
  const radar_msgs__msg__RadarTrack * input,
  radar_msgs__msg__RadarTrack * output)
{
  if (!input || !output) {
    return false;
  }
  // tracking_id
  output->tracking_id = input->tracking_id;
  // x_distance
  output->x_distance = input->x_distance;
  // y_distance
  output->y_distance = input->y_distance;
  // vx
  output->vx = input->vx;
  // vy
  output->vy = input->vy;
  return true;
}

radar_msgs__msg__RadarTrack *
radar_msgs__msg__RadarTrack__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msgs__msg__RadarTrack * msg = (radar_msgs__msg__RadarTrack *)allocator.allocate(sizeof(radar_msgs__msg__RadarTrack), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(radar_msgs__msg__RadarTrack));
  bool success = radar_msgs__msg__RadarTrack__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
radar_msgs__msg__RadarTrack__destroy(radar_msgs__msg__RadarTrack * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    radar_msgs__msg__RadarTrack__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
radar_msgs__msg__RadarTrack__Sequence__init(radar_msgs__msg__RadarTrack__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msgs__msg__RadarTrack * data = NULL;

  if (size) {
    data = (radar_msgs__msg__RadarTrack *)allocator.zero_allocate(size, sizeof(radar_msgs__msg__RadarTrack), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = radar_msgs__msg__RadarTrack__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        radar_msgs__msg__RadarTrack__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
radar_msgs__msg__RadarTrack__Sequence__fini(radar_msgs__msg__RadarTrack__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      radar_msgs__msg__RadarTrack__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

radar_msgs__msg__RadarTrack__Sequence *
radar_msgs__msg__RadarTrack__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  radar_msgs__msg__RadarTrack__Sequence * array = (radar_msgs__msg__RadarTrack__Sequence *)allocator.allocate(sizeof(radar_msgs__msg__RadarTrack__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = radar_msgs__msg__RadarTrack__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
radar_msgs__msg__RadarTrack__Sequence__destroy(radar_msgs__msg__RadarTrack__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    radar_msgs__msg__RadarTrack__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
radar_msgs__msg__RadarTrack__Sequence__are_equal(const radar_msgs__msg__RadarTrack__Sequence * lhs, const radar_msgs__msg__RadarTrack__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!radar_msgs__msg__RadarTrack__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
radar_msgs__msg__RadarTrack__Sequence__copy(
  const radar_msgs__msg__RadarTrack__Sequence * input,
  radar_msgs__msg__RadarTrack__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(radar_msgs__msg__RadarTrack);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    radar_msgs__msg__RadarTrack * data =
      (radar_msgs__msg__RadarTrack *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!radar_msgs__msg__RadarTrack__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          radar_msgs__msg__RadarTrack__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!radar_msgs__msg__RadarTrack__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
