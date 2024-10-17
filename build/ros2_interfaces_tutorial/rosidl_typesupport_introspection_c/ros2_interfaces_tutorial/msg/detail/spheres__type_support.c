// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros2_interfaces_tutorial:msg/Spheres.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros2_interfaces_tutorial/msg/detail/spheres__rosidl_typesupport_introspection_c.h"
#include "ros2_interfaces_tutorial/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros2_interfaces_tutorial/msg/detail/spheres__functions.h"
#include "ros2_interfaces_tutorial/msg/detail/spheres__struct.h"


// Include directives for member types
// Member `spheres`
#include "ros2_interfaces_tutorial/msg/sphere.h"
// Member `spheres`
#include "ros2_interfaces_tutorial/msg/detail/sphere__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__Spheres_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2_interfaces_tutorial__msg__Spheres__init(message_memory);
}

void ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__Spheres_fini_function(void * message_memory)
{
  ros2_interfaces_tutorial__msg__Spheres__fini(message_memory);
}

size_t ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__size_function__Spheres__spheres(
  const void * untyped_member)
{
  const ros2_interfaces_tutorial__msg__Sphere__Sequence * member =
    (const ros2_interfaces_tutorial__msg__Sphere__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__get_const_function__Spheres__spheres(
  const void * untyped_member, size_t index)
{
  const ros2_interfaces_tutorial__msg__Sphere__Sequence * member =
    (const ros2_interfaces_tutorial__msg__Sphere__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__get_function__Spheres__spheres(
  void * untyped_member, size_t index)
{
  ros2_interfaces_tutorial__msg__Sphere__Sequence * member =
    (ros2_interfaces_tutorial__msg__Sphere__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__fetch_function__Spheres__spheres(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ros2_interfaces_tutorial__msg__Sphere * item =
    ((const ros2_interfaces_tutorial__msg__Sphere *)
    ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__get_const_function__Spheres__spheres(untyped_member, index));
  ros2_interfaces_tutorial__msg__Sphere * value =
    (ros2_interfaces_tutorial__msg__Sphere *)(untyped_value);
  *value = *item;
}

void ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__assign_function__Spheres__spheres(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ros2_interfaces_tutorial__msg__Sphere * item =
    ((ros2_interfaces_tutorial__msg__Sphere *)
    ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__get_function__Spheres__spheres(untyped_member, index));
  const ros2_interfaces_tutorial__msg__Sphere * value =
    (const ros2_interfaces_tutorial__msg__Sphere *)(untyped_value);
  *item = *value;
}

bool ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__resize_function__Spheres__spheres(
  void * untyped_member, size_t size)
{
  ros2_interfaces_tutorial__msg__Sphere__Sequence * member =
    (ros2_interfaces_tutorial__msg__Sphere__Sequence *)(untyped_member);
  ros2_interfaces_tutorial__msg__Sphere__Sequence__fini(member);
  return ros2_interfaces_tutorial__msg__Sphere__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__Spheres_message_member_array[1] = {
  {
    "spheres",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_interfaces_tutorial__msg__Spheres, spheres),  // bytes offset in struct
    NULL,  // default value
    ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__size_function__Spheres__spheres,  // size() function pointer
    ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__get_const_function__Spheres__spheres,  // get_const(index) function pointer
    ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__get_function__Spheres__spheres,  // get(index) function pointer
    ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__fetch_function__Spheres__spheres,  // fetch(index, &value) function pointer
    ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__assign_function__Spheres__spheres,  // assign(index, value) function pointer
    ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__resize_function__Spheres__spheres  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__Spheres_message_members = {
  "ros2_interfaces_tutorial__msg",  // message namespace
  "Spheres",  // message name
  1,  // number of fields
  sizeof(ros2_interfaces_tutorial__msg__Spheres),
  ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__Spheres_message_member_array,  // message members
  ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__Spheres_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__Spheres_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__Spheres_message_type_support_handle = {
  0,
  &ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__Spheres_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2_interfaces_tutorial
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_interfaces_tutorial, msg, Spheres)() {
  ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__Spheres_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_interfaces_tutorial, msg, Sphere)();
  if (!ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__Spheres_message_type_support_handle.typesupport_identifier) {
    ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__Spheres_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2_interfaces_tutorial__msg__Spheres__rosidl_typesupport_introspection_c__Spheres_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
