# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_qt_gui_cpp_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED qt_gui_cpp_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(qt_gui_cpp_FOUND FALSE)
  elseif(NOT qt_gui_cpp_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(qt_gui_cpp_FOUND FALSE)
  endif()
  return()
endif()
set(_qt_gui_cpp_CONFIG_INCLUDED TRUE)

# output package information
if(NOT qt_gui_cpp_FIND_QUIETLY)
  message(STATUS "Found qt_gui_cpp: 1.1.3 (${qt_gui_cpp_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'qt_gui_cpp' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${qt_gui_cpp_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(qt_gui_cpp_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "qt_gui_cpp-extras.cmake;ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake")
foreach(_extra ${_extras})
  include("${qt_gui_cpp_DIR}/${_extra}")
endforeach()
