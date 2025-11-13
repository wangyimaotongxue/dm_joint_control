# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_turret_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED turret_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(turret_FOUND FALSE)
  elseif(NOT turret_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(turret_FOUND FALSE)
  endif()
  return()
endif()
set(_turret_CONFIG_INCLUDED TRUE)

# output package information
if(NOT turret_FIND_QUIETLY)
  message(STATUS "Found turret: 0.0.0 (${turret_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'turret' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${turret_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(turret_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${turret_DIR}/${_extra}")
endforeach()
