#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "eigenpy::eigenpy" for configuration "None"
set_property(TARGET eigenpy::eigenpy APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(eigenpy::eigenpy PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libeigenpy.so"
  IMPORTED_SONAME_NONE "libeigenpy.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS eigenpy::eigenpy )
list(APPEND _IMPORT_CHECK_FILES_FOR_eigenpy::eigenpy "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libeigenpy.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
