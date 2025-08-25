// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from feedback:msg/Velocity.idl
// generated code does not contain a copyright notice

#ifndef FEEDBACK__MSG__DETAIL__VELOCITY__STRUCT_HPP_
#define FEEDBACK__MSG__DETAIL__VELOCITY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__feedback__msg__Velocity __attribute__((deprecated))
#else
# define DEPRECATED__feedback__msg__Velocity __declspec(deprecated)
#endif

namespace feedback
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Velocity_
{
  using Type = Velocity_<ContainerAllocator>;

  explicit Velocity_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vehicle_velocity = 0.0f;
      this->wheelrpm_fl = 0.0f;
      this->wheelrpm_fr = 0.0f;
      this->wheelrpm_rl = 0.0f;
      this->wheelrpm_rr = 0.0f;
    }
  }

  explicit Velocity_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vehicle_velocity = 0.0f;
      this->wheelrpm_fl = 0.0f;
      this->wheelrpm_fr = 0.0f;
      this->wheelrpm_rl = 0.0f;
      this->wheelrpm_rr = 0.0f;
    }
  }

  // field types and members
  using _vehicle_velocity_type =
    float;
  _vehicle_velocity_type vehicle_velocity;
  using _wheelrpm_fl_type =
    float;
  _wheelrpm_fl_type wheelrpm_fl;
  using _wheelrpm_fr_type =
    float;
  _wheelrpm_fr_type wheelrpm_fr;
  using _wheelrpm_rl_type =
    float;
  _wheelrpm_rl_type wheelrpm_rl;
  using _wheelrpm_rr_type =
    float;
  _wheelrpm_rr_type wheelrpm_rr;

  // setters for named parameter idiom
  Type & set__vehicle_velocity(
    const float & _arg)
  {
    this->vehicle_velocity = _arg;
    return *this;
  }
  Type & set__wheelrpm_fl(
    const float & _arg)
  {
    this->wheelrpm_fl = _arg;
    return *this;
  }
  Type & set__wheelrpm_fr(
    const float & _arg)
  {
    this->wheelrpm_fr = _arg;
    return *this;
  }
  Type & set__wheelrpm_rl(
    const float & _arg)
  {
    this->wheelrpm_rl = _arg;
    return *this;
  }
  Type & set__wheelrpm_rr(
    const float & _arg)
  {
    this->wheelrpm_rr = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    feedback::msg::Velocity_<ContainerAllocator> *;
  using ConstRawPtr =
    const feedback::msg::Velocity_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<feedback::msg::Velocity_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<feedback::msg::Velocity_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      feedback::msg::Velocity_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<feedback::msg::Velocity_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      feedback::msg::Velocity_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<feedback::msg::Velocity_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<feedback::msg::Velocity_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<feedback::msg::Velocity_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__feedback__msg__Velocity
    std::shared_ptr<feedback::msg::Velocity_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__feedback__msg__Velocity
    std::shared_ptr<feedback::msg::Velocity_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Velocity_ & other) const
  {
    if (this->vehicle_velocity != other.vehicle_velocity) {
      return false;
    }
    if (this->wheelrpm_fl != other.wheelrpm_fl) {
      return false;
    }
    if (this->wheelrpm_fr != other.wheelrpm_fr) {
      return false;
    }
    if (this->wheelrpm_rl != other.wheelrpm_rl) {
      return false;
    }
    if (this->wheelrpm_rr != other.wheelrpm_rr) {
      return false;
    }
    return true;
  }
  bool operator!=(const Velocity_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Velocity_

// alias to use template instance with default allocator
using Velocity =
  feedback::msg::Velocity_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace feedback

#endif  // FEEDBACK__MSG__DETAIL__VELOCITY__STRUCT_HPP_
