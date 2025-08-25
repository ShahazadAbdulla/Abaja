// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from radar_msgs:msg/RadarTrackList.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "radar_msgs/msg/detail/radar_track_list__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace radar_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void RadarTrackList_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) radar_msgs::msg::RadarTrackList(_init);
}

void RadarTrackList_fini_function(void * message_memory)
{
  auto typed_message = static_cast<radar_msgs::msg::RadarTrackList *>(message_memory);
  typed_message->~RadarTrackList();
}

size_t size_function__RadarTrackList__objects(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<radar_msgs::msg::RadarTrack> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RadarTrackList__objects(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<radar_msgs::msg::RadarTrack> *>(untyped_member);
  return &member[index];
}

void * get_function__RadarTrackList__objects(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<radar_msgs::msg::RadarTrack> *>(untyped_member);
  return &member[index];
}

void fetch_function__RadarTrackList__objects(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const radar_msgs::msg::RadarTrack *>(
    get_const_function__RadarTrackList__objects(untyped_member, index));
  auto & value = *reinterpret_cast<radar_msgs::msg::RadarTrack *>(untyped_value);
  value = item;
}

void assign_function__RadarTrackList__objects(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<radar_msgs::msg::RadarTrack *>(
    get_function__RadarTrackList__objects(untyped_member, index));
  const auto & value = *reinterpret_cast<const radar_msgs::msg::RadarTrack *>(untyped_value);
  item = value;
}

void resize_function__RadarTrackList__objects(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<radar_msgs::msg::RadarTrack> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RadarTrackList_message_member_array[1] = {
  {
    "objects",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<radar_msgs::msg::RadarTrack>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(radar_msgs::msg::RadarTrackList, objects),  // bytes offset in struct
    nullptr,  // default value
    size_function__RadarTrackList__objects,  // size() function pointer
    get_const_function__RadarTrackList__objects,  // get_const(index) function pointer
    get_function__RadarTrackList__objects,  // get(index) function pointer
    fetch_function__RadarTrackList__objects,  // fetch(index, &value) function pointer
    assign_function__RadarTrackList__objects,  // assign(index, value) function pointer
    resize_function__RadarTrackList__objects  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RadarTrackList_message_members = {
  "radar_msgs::msg",  // message namespace
  "RadarTrackList",  // message name
  1,  // number of fields
  sizeof(radar_msgs::msg::RadarTrackList),
  RadarTrackList_message_member_array,  // message members
  RadarTrackList_init_function,  // function to initialize message memory (memory has to be allocated)
  RadarTrackList_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RadarTrackList_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RadarTrackList_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace radar_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<radar_msgs::msg::RadarTrackList>()
{
  return &::radar_msgs::msg::rosidl_typesupport_introspection_cpp::RadarTrackList_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, radar_msgs, msg, RadarTrackList)() {
  return &::radar_msgs::msg::rosidl_typesupport_introspection_cpp::RadarTrackList_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
