#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "octomath" for configuration "None"
set_property(TARGET octomath APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(octomath PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/liboctomath.so.1.9.8"
  IMPORTED_SONAME_NONE "liboctomath.so.1.9"
  )

list(APPEND _IMPORT_CHECK_TARGETS octomath )
list(APPEND _IMPORT_CHECK_FILES_FOR_octomath "${_IMPORT_PREFIX}/lib/liboctomath.so.1.9.8" )

# Import target "octomath-static" for configuration "None"
set_property(TARGET octomath-static APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(octomath-static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NONE "CXX"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/liboctomath.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS octomath-static )
list(APPEND _IMPORT_CHECK_FILES_FOR_octomath-static "${_IMPORT_PREFIX}/lib/liboctomath.a" )

# Import target "octomap" for configuration "None"
set_property(TARGET octomap APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(octomap PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/liboctomap.so.1.9.8"
  IMPORTED_SONAME_NONE "liboctomap.so.1.9"
  )

list(APPEND _IMPORT_CHECK_TARGETS octomap )
list(APPEND _IMPORT_CHECK_FILES_FOR_octomap "${_IMPORT_PREFIX}/lib/liboctomap.so.1.9.8" )

# Import target "octomap-static" for configuration "None"
set_property(TARGET octomap-static APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(octomap-static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NONE "CXX"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/liboctomap.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS octomap-static )
list(APPEND _IMPORT_CHECK_FILES_FOR_octomap-static "${_IMPORT_PREFIX}/lib/liboctomap.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
