// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tutorial_interface:msg/Sphere.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INTERFACE__MSG__DETAIL__SPHERE__BUILDER_HPP_
#define TUTORIAL_INTERFACE__MSG__DETAIL__SPHERE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tutorial_interface/msg/detail/sphere__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tutorial_interface
{

namespace msg
{

namespace builder
{

class Init_Sphere_radius
{
public:
  explicit Init_Sphere_radius(::tutorial_interface::msg::Sphere & msg)
  : msg_(msg)
  {}
  ::tutorial_interface::msg::Sphere radius(::tutorial_interface::msg::Sphere::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tutorial_interface::msg::Sphere msg_;
};

class Init_Sphere_center
{
public:
  Init_Sphere_center()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Sphere_radius center(::tutorial_interface::msg::Sphere::_center_type arg)
  {
    msg_.center = std::move(arg);
    return Init_Sphere_radius(msg_);
  }

private:
  ::tutorial_interface::msg::Sphere msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tutorial_interface::msg::Sphere>()
{
  return tutorial_interface::msg::builder::Init_Sphere_center();
}

}  // namespace tutorial_interface

#endif  // TUTORIAL_INTERFACE__MSG__DETAIL__SPHERE__BUILDER_HPP_
