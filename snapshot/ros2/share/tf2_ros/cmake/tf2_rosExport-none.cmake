#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "tf2_ros::tf2_ros" for configuration "None"
set_property(TARGET tf2_ros::tf2_ros APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(tf2_ros::tf2_ros PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/libtf2_ros.so"
  IMPORTED_SONAME_NONE "libtf2_ros.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS tf2_ros::tf2_ros )
list(APPEND _IMPORT_CHECK_FILES_FOR_tf2_ros::tf2_ros "${_IMPORT_PREFIX}/lib/libtf2_ros.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
