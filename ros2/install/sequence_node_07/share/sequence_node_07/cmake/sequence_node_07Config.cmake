# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_sequence_node_07_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED sequence_node_07_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(sequence_node_07_FOUND FALSE)
  elseif(NOT sequence_node_07_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(sequence_node_07_FOUND FALSE)
  endif()
  return()
endif()
set(_sequence_node_07_CONFIG_INCLUDED TRUE)

# output package information
if(NOT sequence_node_07_FIND_QUIETLY)
  message(STATUS "Found sequence_node_07: 0.1.0 (${sequence_node_07_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'sequence_node_07' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${sequence_node_07_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(sequence_node_07_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${sequence_node_07_DIR}/${_extra}")
endforeach()
