# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_modbus_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED modbus_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(modbus_FOUND FALSE)
  elseif(NOT modbus_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(modbus_FOUND FALSE)
  endif()
  return()
endif()
set(_modbus_CONFIG_INCLUDED TRUE)

# output package information
if(NOT modbus_FIND_QUIETLY)
  message(STATUS "Found modbus: 0.0.0 (${modbus_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'modbus' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${modbus_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(modbus_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake")
foreach(_extra ${_extras})
  include("${modbus_DIR}/${_extra}")
endforeach()
