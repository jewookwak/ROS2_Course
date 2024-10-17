// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_interfaces_tutorial:msg/Spheres.idl
// generated code does not contain a copyright notice

#ifndef ROS2_INTERFACES_TUTORIAL__MSG__DETAIL__SPHERES__STRUCT_HPP_
#define ROS2_INTERFACES_TUTORIAL__MSG__DETAIL__SPHERES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'spheres'
#include "ros2_interfaces_tutorial/msg/detail/sphere__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros2_interfaces_tutorial__msg__Spheres __attribute__((deprecated))
#else
# define DEPRECATED__ros2_interfaces_tutorial__msg__Spheres __declspec(deprecated)
#endif

namespace ros2_interfaces_tutorial
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Spheres_
{
  using Type = Spheres_<ContainerAllocator>;

  explicit Spheres_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Spheres_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _spheres_type =
    std::vector<ros2_interfaces_tutorial::msg::Sphere_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros2_interfaces_tutorial::msg::Sphere_<ContainerAllocator>>>;
  _spheres_type spheres;

  // setters for named parameter idiom
  Type & set__spheres(
    const std::vector<ros2_interfaces_tutorial::msg::Sphere_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros2_interfaces_tutorial::msg::Sphere_<ContainerAllocator>>> & _arg)
  {
    this->spheres = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_interfaces_tutorial::msg::Spheres_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_interfaces_tutorial::msg::Spheres_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_interfaces_tutorial::msg::Spheres_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_interfaces_tutorial::msg::Spheres_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_interfaces_tutorial::msg::Spheres_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_interfaces_tutorial::msg::Spheres_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_interfaces_tutorial::msg::Spheres_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_interfaces_tutorial::msg::Spheres_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_interfaces_tutorial::msg::Spheres_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_interfaces_tutorial::msg::Spheres_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_interfaces_tutorial__msg__Spheres
    std::shared_ptr<ros2_interfaces_tutorial::msg::Spheres_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_interfaces_tutorial__msg__Spheres
    std::shared_ptr<ros2_interfaces_tutorial::msg::Spheres_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Spheres_ & other) const
  {
    if (this->spheres != other.spheres) {
      return false;
    }
    return true;
  }
  bool operator!=(const Spheres_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Spheres_

// alias to use template instance with default allocator
using Spheres =
  ros2_interfaces_tutorial::msg::Spheres_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros2_interfaces_tutorial

#endif  // ROS2_INTERFACES_TUTORIAL__MSG__DETAIL__SPHERES__STRUCT_HPP_
