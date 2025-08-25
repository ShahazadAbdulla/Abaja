// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from feedback:msg/Velocity.idl
// generated code does not contain a copyright notice
#include "feedback/msg/detail/velocity__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
feedback__msg__Velocity__init(feedback__msg__Velocity * msg)
{
  if (!msg) {
    return false;
  }
  // vehicle_velocity
  // wheelrpm_fl
  // wheelrpm_fr
  // wheelrpm_rl
  // wheelrpm_rr
  return true;
}

void
feedback__msg__Velocity__fini(feedback__msg__Velocity * msg)
{
  if (!msg) {
    return;
  }
  // vehicle_velocity
  // wheelrpm_fl
  // wheelrpm_fr
  // wheelrpm_rl
  // wheelrpm_rr
}

bool
feedback__msg__Velocity__are_equal(const feedback__msg__Velocity * lhs, const feedback__msg__Velocity * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // vehicle_velocity
  if (lhs->vehicle_velocity != rhs->vehicle_velocity) {
    return false;
  }
  // wheelrpm_fl
  if (lhs->wheelrpm_fl != rhs->wheelrpm_fl) {
    return false;
  }
  // wheelrpm_fr
  if (lhs->wheelrpm_fr != rhs->wheelrpm_fr) {
    return false;
  }
  // wheelrpm_rl
  if (lhs->wheelrpm_rl != rhs->wheelrpm_rl) {
    return false;
  }
  // wheelrpm_rr
  if (lhs->wheelrpm_rr != rhs->wheelrpm_rr) {
    return false;
  }
  return true;
}

bool
feedback__msg__Velocity__copy(
  const feedback__msg__Velocity * input,
  feedback__msg__Velocity * output)
{
  if (!input || !output) {
    return false;
  }
  // vehicle_velocity
  output->vehicle_velocity = input->vehicle_velocity;
  // wheelrpm_fl
  output->wheelrpm_fl = input->wheelrpm_fl;
  // wheelrpm_fr
  output->wheelrpm_fr = input->wheelrpm_fr;
  // wheelrpm_rl
  output->wheelrpm_rl = input->wheelrpm_rl;
  // wheelrpm_rr
  output->wheelrpm_rr = input->wheelrpm_rr;
  return true;
}

feedback__msg__Velocity *
feedback__msg__Velocity__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  feedback__msg__Velocity * msg = (feedback__msg__Velocity *)allocator.allocate(sizeof(feedback__msg__Velocity), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(feedback__msg__Velocity));
  bool success = feedback__msg__Velocity__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
feedback__msg__Velocity__destroy(feedback__msg__Velocity * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    feedback__msg__Velocity__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
feedback__msg__Velocity__Sequence__init(feedback__msg__Velocity__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  feedback__msg__Velocity * data = NULL;

  if (size) {
    data = (feedback__msg__Velocity *)allocator.zero_allocate(size, sizeof(feedback__msg__Velocity), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = feedback__msg__Velocity__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        feedback__msg__Velocity__fini(&data[i - 1]);
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
feedback__msg__Velocity__Sequence__fini(feedback__msg__Velocity__Sequence * array)
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
      feedback__msg__Velocity__fini(&array->data[i]);
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

feedback__msg__Velocity__Sequence *
feedback__msg__Velocity__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  feedback__msg__Velocity__Sequence * array = (feedback__msg__Velocity__Sequence *)allocator.allocate(sizeof(feedback__msg__Velocity__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = feedback__msg__Velocity__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
feedback__msg__Velocity__Sequence__destroy(feedback__msg__Velocity__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    feedback__msg__Velocity__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
feedback__msg__Velocity__Sequence__are_equal(const feedback__msg__Velocity__Sequence * lhs, const feedback__msg__Velocity__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!feedback__msg__Velocity__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
feedback__msg__Velocity__Sequence__copy(
  const feedback__msg__Velocity__Sequence * input,
  feedback__msg__Velocity__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(feedback__msg__Velocity);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    feedback__msg__Velocity * data =
      (feedback__msg__Velocity *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!feedback__msg__Velocity__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          feedback__msg__Velocity__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!feedback__msg__Velocity__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
