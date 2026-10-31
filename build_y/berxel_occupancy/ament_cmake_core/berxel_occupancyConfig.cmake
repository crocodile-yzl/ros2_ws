# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_berxel_occupancy_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED berxel_occupancy_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(berxel_occupancy_FOUND FALSE)
  elseif(NOT berxel_occupancy_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(berxel_occupancy_FOUND FALSE)
  endif()
  return()
endif()
set(_berxel_occupancy_CONFIG_INCLUDED TRUE)

# output package information
if(NOT berxel_occupancy_FIND_QUIETLY)
  message(STATUS "Found berxel_occupancy: 0.1.0 (${berxel_occupancy_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'berxel_occupancy' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${berxel_occupancy_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(berxel_occupancy_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${berxel_occupancy_DIR}/${_extra}")
endforeach()
