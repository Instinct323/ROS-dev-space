# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_composition_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED composition_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(composition_FOUND FALSE)
  elseif(NOT composition_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(composition_FOUND FALSE)
  endif()
  return()
endif()
set(_composition_CONFIG_INCLUDED TRUE)

# output package information
if(NOT composition_FIND_QUIETLY)
  message(STATUS "Found composition: 0.9.4 (${composition_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'composition' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${composition_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(composition_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${composition_DIR}/${_extra}")
endforeach()
