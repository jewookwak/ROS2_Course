# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pic4rl_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pic4rl_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pic4rl_FOUND FALSE)
  elseif(NOT pic4rl_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pic4rl_FOUND FALSE)
  endif()
  return()
endif()
set(_pic4rl_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pic4rl_FIND_QUIETLY)
  message(STATUS "Found pic4rl: 0.0.0 (${pic4rl_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pic4rl' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${pic4rl_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pic4rl_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pic4rl_DIR}/${_extra}")
endforeach()
