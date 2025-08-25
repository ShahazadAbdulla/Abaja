# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_aeb_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED aeb_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(aeb_FOUND FALSE)
  elseif(NOT aeb_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(aeb_FOUND FALSE)
  endif()
  return()
endif()
set(_aeb_CONFIG_INCLUDED TRUE)

# output package information
if(NOT aeb_FIND_QUIETLY)
  message(STATUS "Found aeb: 0.0.0 (${aeb_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'aeb' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${aeb_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(aeb_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${aeb_DIR}/${_extra}")
endforeach()
