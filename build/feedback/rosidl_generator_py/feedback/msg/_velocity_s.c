// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from feedback:msg/Velocity.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "feedback/msg/detail/velocity__struct.h"
#include "feedback/msg/detail/velocity__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool feedback__msg__velocity__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[32];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("feedback.msg._velocity.Velocity", full_classname_dest, 31) == 0);
  }
  feedback__msg__Velocity * ros_message = _ros_message;
  {  // vehicle_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "vehicle_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vehicle_velocity = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // wheelrpm_fl
    PyObject * field = PyObject_GetAttrString(_pymsg, "wheelrpm_fl");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->wheelrpm_fl = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // wheelrpm_fr
    PyObject * field = PyObject_GetAttrString(_pymsg, "wheelrpm_fr");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->wheelrpm_fr = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // wheelrpm_rl
    PyObject * field = PyObject_GetAttrString(_pymsg, "wheelrpm_rl");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->wheelrpm_rl = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // wheelrpm_rr
    PyObject * field = PyObject_GetAttrString(_pymsg, "wheelrpm_rr");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->wheelrpm_rr = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * feedback__msg__velocity__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Velocity */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("feedback.msg._velocity");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Velocity");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  feedback__msg__Velocity * ros_message = (feedback__msg__Velocity *)raw_ros_message;
  {  // vehicle_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vehicle_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vehicle_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // wheelrpm_fl
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->wheelrpm_fl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "wheelrpm_fl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // wheelrpm_fr
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->wheelrpm_fr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "wheelrpm_fr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // wheelrpm_rl
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->wheelrpm_rl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "wheelrpm_rl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // wheelrpm_rr
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->wheelrpm_rr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "wheelrpm_rr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
