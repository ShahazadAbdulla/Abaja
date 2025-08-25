// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vehiclecontrol:msg/Control.idl
// generated code does not contain a copyright notice

#ifndef VEHICLECONTROL__MSG__DETAIL__CONTROL__STRUCT_HPP_
#define VEHICLECONTROL__MSG__DETAIL__CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vehiclecontrol__msg__Control __attribute__((deprecated))
#else
# define DEPRECATED__vehiclecontrol__msg__Control __declspec(deprecated)
#endif

namespace vehiclecontrol
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Control_
{
  using Type = Control_<ContainerAllocator>;

  explicit Control_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->steering = 0.0f;
      this->throttle = 0.0f;
      this->brake = 0.0f;
      this->longswitch = 0;
      this->latswitch = 0;
    }
  }

  explicit Control_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->steering = 0.0f;
      this->throttle = 0.0f;
      this->brake = 0.0f;
      this->longswitch = 0;
      this->latswitch = 0;
    }
  }

  // field types and members
  using _steering_type =
    float;
  _steering_type steering;
  using _throttle_type =
    float;
  _throttle_type throttle;
  using _brake_type =
    float;
  _brake_type brake;
  using _longswitch_type =
    int8_t;
  _longswitch_type longswitch;
  using _latswitch_type =
    int8_t;
  _latswitch_type latswitch;

  // setters for named parameter idiom
  Type & set__steering(
    const float & _arg)
  {
    this->steering = _arg;
    return *this;
  }
  Type & set__throttle(
    const float & _arg)
  {
    this->throttle = _arg;
    return *this;
  }
  Type & set__brake(
    const float & _arg)
  {
    this->brake = _arg;
    return *this;
  }
  Type & set__longswitch(
    const int8_t & _arg)
  {
    this->longswitch = _arg;
    return *this;
  }
  Type & set__latswitch(
    const int8_t & _arg)
  {
    this->latswitch = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vehiclecontrol::msg::Control_<ContainerAllocator> *;
  using ConstRawPtr =
    const vehiclecontrol::msg::Control_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vehiclecontrol::msg::Control_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vehiclecontrol::msg::Control_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vehiclecontrol::msg::Control_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vehiclecontrol::msg::Control_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vehiclecontrol::msg::Control_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vehiclecontrol::msg::Control_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vehiclecontrol::msg::Control_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vehiclecontrol::msg::Control_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vehiclecontrol__msg__Control
    std::shared_ptr<vehiclecontrol::msg::Control_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vehiclecontrol__msg__Control
    std::shared_ptr<vehiclecontrol::msg::Control_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Control_ & other) const
  {
    if (this->steering != other.steering) {
      return false;
    }
    if (this->throttle != other.throttle) {
      return false;
    }
    if (this->brake != other.brake) {
      return false;
    }
    if (this->longswitch != other.longswitch) {
      return false;
    }
    if (this->latswitch != other.latswitch) {
      return false;
    }
    return true;
  }
  bool operator!=(const Control_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Control_

// alias to use template instance with default allocator
using Control =
  vehiclecontrol::msg::Control_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vehiclecontrol

#endif  // VEHICLECONTROL__MSG__DETAIL__CONTROL__STRUCT_HPP_
