// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_interfaces_tutorial:msg/Spheres.idl
// generated code does not contain a copyright notice

#ifndef ROS2_INTERFACES_TUTORIAL__MSG__DETAIL__SPHERES__TRAITS_HPP_
#define ROS2_INTERFACES_TUTORIAL__MSG__DETAIL__SPHERES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_interfaces_tutorial/msg/detail/spheres__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'spheres'
#include "ros2_interfaces_tutorial/msg/detail/sphere__traits.hpp"

namespace ros2_interfaces_tutorial
{

namespace msg
{

inline void to_flow_style_yaml(
  const Spheres & msg,
  std::ostream & out)
{
  out << "{";
  // member: spheres
  {
    if (msg.spheres.size() == 0) {
      out << "spheres: []";
    } else {
      out << "spheres: [";
      size_t pending_items = msg.spheres.size();
      for (auto item : msg.spheres) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Spheres & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: spheres
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.spheres.size() == 0) {
      out << "spheres: []\n";
    } else {
      out << "spheres:\n";
      for (auto item : msg.spheres) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Spheres & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ros2_interfaces_tutorial

namespace rosidl_generator_traits
{

[[deprecated("use ros2_interfaces_tutorial::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2_interfaces_tutorial::msg::Spheres & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_interfaces_tutorial::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_interfaces_tutorial::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros2_interfaces_tutorial::msg::Spheres & msg)
{
  return ros2_interfaces_tutorial::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_interfaces_tutorial::msg::Spheres>()
{
  return "ros2_interfaces_tutorial::msg::Spheres";
}

template<>
inline const char * name<ros2_interfaces_tutorial::msg::Spheres>()
{
  return "ros2_interfaces_tutorial/msg/Spheres";
}

template<>
struct has_fixed_size<ros2_interfaces_tutorial::msg::Spheres>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2_interfaces_tutorial::msg::Spheres>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2_interfaces_tutorial::msg::Spheres>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2_INTERFACES_TUTORIAL__MSG__DETAIL__SPHERES__TRAITS_HPP_
