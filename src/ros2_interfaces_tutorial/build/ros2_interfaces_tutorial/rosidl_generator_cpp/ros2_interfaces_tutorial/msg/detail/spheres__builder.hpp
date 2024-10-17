// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_interfaces_tutorial:msg/Spheres.idl
// generated code does not contain a copyright notice

#ifndef ROS2_INTERFACES_TUTORIAL__MSG__DETAIL__SPHERES__BUILDER_HPP_
#define ROS2_INTERFACES_TUTORIAL__MSG__DETAIL__SPHERES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_interfaces_tutorial/msg/detail/spheres__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_interfaces_tutorial
{

namespace msg
{

namespace builder
{

class Init_Spheres_spheres
{
public:
  Init_Spheres_spheres()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ros2_interfaces_tutorial::msg::Spheres spheres(::ros2_interfaces_tutorial::msg::Spheres::_spheres_type arg)
  {
    msg_.spheres = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_interfaces_tutorial::msg::Spheres msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_interfaces_tutorial::msg::Spheres>()
{
  return ros2_interfaces_tutorial::msg::builder::Init_Spheres_spheres();
}

}  // namespace ros2_interfaces_tutorial

#endif  // ROS2_INTERFACES_TUTORIAL__MSG__DETAIL__SPHERES__BUILDER_HPP_
