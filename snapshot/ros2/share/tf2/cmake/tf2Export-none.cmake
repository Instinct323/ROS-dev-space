#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "tf2::tf2" for configuration "None"
set_property(TARGET tf2::tf2 APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(tf2::tf2 PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/libtf2.so"
  IMPORTED_SONAME_NONE "libtf2.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS tf2::tf2 )
list(APPEND _IMPORT_CHECK_FILES_FOR_tf2::tf2 "${_IMPORT_PREFIX}/lib/libtf2.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
