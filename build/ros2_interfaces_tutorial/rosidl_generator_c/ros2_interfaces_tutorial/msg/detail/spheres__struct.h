// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_interfaces_tutorial:msg/Spheres.idl
// generated code does not contain a copyright notice

#ifndef ROS2_INTERFACES_TUTORIAL__MSG__DETAIL__SPHERES__STRUCT_H_
#define ROS2_INTERFACES_TUTORIAL__MSG__DETAIL__SPHERES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'spheres'
#include "ros2_interfaces_tutorial/msg/detail/sphere__struct.h"

/// Struct defined in msg/Spheres in the package ros2_interfaces_tutorial.
typedef struct ros2_interfaces_tutorial__msg__Spheres
{
  ros2_interfaces_tutorial__msg__Sphere__Sequence spheres;
} ros2_interfaces_tutorial__msg__Spheres;

// Struct for a sequence of ros2_interfaces_tutorial__msg__Spheres.
typedef struct ros2_interfaces_tutorial__msg__Spheres__Sequence
{
  ros2_interfaces_tutorial__msg__Spheres * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_interfaces_tutorial__msg__Spheres__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_INTERFACES_TUTORIAL__MSG__DETAIL__SPHERES__STRUCT_H_
