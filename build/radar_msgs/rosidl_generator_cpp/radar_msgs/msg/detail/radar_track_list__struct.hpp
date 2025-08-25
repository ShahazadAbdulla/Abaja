// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from radar_msgs:msg/RadarTrackList.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSGS__MSG__DETAIL__RADAR_TRACK_LIST__STRUCT_HPP_
#define RADAR_MSGS__MSG__DETAIL__RADAR_TRACK_LIST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'objects'
#include "radar_msgs/msg/detail/radar_track__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__radar_msgs__msg__RadarTrackList __attribute__((deprecated))
#else
# define DEPRECATED__radar_msgs__msg__RadarTrackList __declspec(deprecated)
#endif

namespace radar_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RadarTrackList_
{
  using Type = RadarTrackList_<ContainerAllocator>;

  explicit RadarTrackList_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit RadarTrackList_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _objects_type =
    std::vector<radar_msgs::msg::RadarTrack_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<radar_msgs::msg::RadarTrack_<ContainerAllocator>>>;
  _objects_type objects;

  // setters for named parameter idiom
  Type & set__objects(
    const std::vector<radar_msgs::msg::RadarTrack_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<radar_msgs::msg::RadarTrack_<ContainerAllocator>>> & _arg)
  {
    this->objects = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_msgs::msg::RadarTrackList_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msgs::msg::RadarTrackList_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msgs::msg::RadarTrackList_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msgs::msg::RadarTrackList_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msgs::msg::RadarTrackList_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msgs::msg::RadarTrackList_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msgs::msg::RadarTrackList_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msgs::msg::RadarTrackList_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msgs::msg::RadarTrackList_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msgs::msg::RadarTrackList_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msgs__msg__RadarTrackList
    std::shared_ptr<radar_msgs::msg::RadarTrackList_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msgs__msg__RadarTrackList
    std::shared_ptr<radar_msgs::msg::RadarTrackList_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RadarTrackList_ & other) const
  {
    if (this->objects != other.objects) {
      return false;
    }
    return true;
  }
  bool operator!=(const RadarTrackList_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RadarTrackList_

// alias to use template instance with default allocator
using RadarTrackList =
  radar_msgs::msg::RadarTrackList_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace radar_msgs

#endif  // RADAR_MSGS__MSG__DETAIL__RADAR_TRACK_LIST__STRUCT_HPP_
