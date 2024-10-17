// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_interfaces_tutorial:msg/Sphere.idl
// generated code does not contain a copyright notice

#ifndef ROS2_INTERFACES_TUTORIAL__MSG__DETAIL__SPHERE__BUILDER_HPP_
#define ROS2_INTERFACES_TUTORIAL__MSG__DETAIL__SPHERE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_interfaces_tutorial/msg/detail/sphere__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_interfaces_tutorial
{

namespace msg
{

namespace builder
{

class Init_Sphere_radius
{
public:
  explicit Init_Sphere_radius(::ros2_interfaces_tutorial::msg::Sphere & msg)
  : msg_(msg)
  {}
  ::ros2_interfaces_tutorial::msg::Sphere radius(::ros2_interfaces_tutorial::msg::Sphere::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_interfaces_tutorial::msg::Sphere msg_;
};

class Init_Sphere_center
{
public:
  Init_Sphere_center()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Sphere_radius center(::ros2_interfaces_tutorial::msg::Sphere::_center_type arg)
  {
    msg_.center = std::move(arg);
    return Init_Sphere_radius(msg_);
  }

private:
  ::ros2_interfaces_tutorial::msg::Sphere msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_interfaces_tutorial::msg::Sphere>()
{
  return ros2_interfaces_tutorial::msg::builder::Init_Sphere_center();
}

}  // namespace ros2_interfaces_tutorial

#endif  // ROS2_INTERFACES_TUTORIAL__MSG__DETAIL__SPHERE__BUILDER_HPP_
