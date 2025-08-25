// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from radar_msgs:msg/RadarTrack.idl
// generated code does not contain a copyright notice

#ifndef RADAR_MSGS__MSG__DETAIL__RADAR_TRACK__STRUCT_HPP_
#define RADAR_MSGS__MSG__DETAIL__RADAR_TRACK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__radar_msgs__msg__RadarTrack __attribute__((deprecated))
#else
# define DEPRECATED__radar_msgs__msg__RadarTrack __declspec(deprecated)
#endif

namespace radar_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RadarTrack_
{
  using Type = RadarTrack_<ContainerAllocator>;

  explicit RadarTrack_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tracking_id = 0l;
      this->x_distance = 0.0f;
      this->y_distance = 0.0f;
      this->vx = 0.0f;
      this->vy = 0.0f;
    }
  }

  explicit RadarTrack_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tracking_id = 0l;
      this->x_distance = 0.0f;
      this->y_distance = 0.0f;
      this->vx = 0.0f;
      this->vy = 0.0f;
    }
  }

  // field types and members
  using _tracking_id_type =
    int32_t;
  _tracking_id_type tracking_id;
  using _x_distance_type =
    float;
  _x_distance_type x_distance;
  using _y_distance_type =
    float;
  _y_distance_type y_distance;
  using _vx_type =
    float;
  _vx_type vx;
  using _vy_type =
    float;
  _vy_type vy;

  // setters for named parameter idiom
  Type & set__tracking_id(
    const int32_t & _arg)
  {
    this->tracking_id = _arg;
    return *this;
  }
  Type & set__x_distance(
    const float & _arg)
  {
    this->x_distance = _arg;
    return *this;
  }
  Type & set__y_distance(
    const float & _arg)
  {
    this->y_distance = _arg;
    return *this;
  }
  Type & set__vx(
    const float & _arg)
  {
    this->vx = _arg;
    return *this;
  }
  Type & set__vy(
    const float & _arg)
  {
    this->vy = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    radar_msgs::msg::RadarTrack_<ContainerAllocator> *;
  using ConstRawPtr =
    const radar_msgs::msg::RadarTrack_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<radar_msgs::msg::RadarTrack_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<radar_msgs::msg::RadarTrack_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      radar_msgs::msg::RadarTrack_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<radar_msgs::msg::RadarTrack_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      radar_msgs::msg::RadarTrack_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<radar_msgs::msg::RadarTrack_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<radar_msgs::msg::RadarTrack_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<radar_msgs::msg::RadarTrack_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__radar_msgs__msg__RadarTrack
    std::shared_ptr<radar_msgs::msg::RadarTrack_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__radar_msgs__msg__RadarTrack
    std::shared_ptr<radar_msgs::msg::RadarTrack_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RadarTrack_ & other) const
  {
    if (this->tracking_id != other.tracking_id) {
      return false;
    }
    if (this->x_distance != other.x_distance) {
      return false;
    }
    if (this->y_distance != other.y_distance) {
      return false;
    }
    if (this->vx != other.vx) {
      return false;
    }
    if (this->vy != other.vy) {
      return false;
    }
    return true;
  }
  bool operator!=(const RadarTrack_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RadarTrack_

// alias to use template instance with default allocator
using RadarTrack =
  radar_msgs::msg::RadarTrack_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace radar_msgs

#endif  // RADAR_MSGS__MSG__DETAIL__RADAR_TRACK__STRUCT_HPP_
