// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from radar_msgs:msg/RadarTrack.idl
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
#include "radar_msgs/msg/detail/radar_track__struct.h"
#include "radar_msgs/msg/detail/radar_track__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool radar_msgs__msg__radar_track__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[39];
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
    assert(strncmp("radar_msgs.msg._radar_track.RadarTrack", full_classname_dest, 38) == 0);
  }
  radar_msgs__msg__RadarTrack * ros_message = _ros_message;
  {  // tracking_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "tracking_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->tracking_id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // x_distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x_distance = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y_distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y_distance = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vx
    PyObject * field = PyObject_GetAttrString(_pymsg, "vx");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vx = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vy
    PyObject * field = PyObject_GetAttrString(_pymsg, "vy");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vy = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * radar_msgs__msg__radar_track__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RadarTrack */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("radar_msgs.msg._radar_track");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RadarTrack");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  radar_msgs__msg__RadarTrack * ros_message = (radar_msgs__msg__RadarTrack *)raw_ros_message;
  {  // tracking_id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->tracking_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tracking_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x_distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x_distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y_distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vx
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vy
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
