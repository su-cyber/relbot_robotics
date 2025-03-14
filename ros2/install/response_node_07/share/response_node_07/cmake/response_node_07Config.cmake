# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_response_node_07_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED response_node_07_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(response_node_07_FOUND FALSE)
  elseif(NOT response_node_07_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(response_node_07_FOUND FALSE)
  endif()
  return()
endif()
set(_response_node_07_CONFIG_INCLUDED TRUE)

# output package information
if(NOT response_node_07_FIND_QUIETLY)
  message(STATUS "Found response_node_07: 0.1.0 (${response_node_07_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'response_node_07' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${response_node_07_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(response_node_07_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${response_node_07_DIR}/${_extra}")
endforeach()
