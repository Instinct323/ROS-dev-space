#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rmw_dds_common::rmw_dds_common_library" for configuration "None"
set_property(TARGET rmw_dds_common::rmw_dds_common_library APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(rmw_dds_common::rmw_dds_common_library PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/librmw_dds_common.so"
  IMPORTED_SONAME_NONE "librmw_dds_common.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rmw_dds_common::rmw_dds_common_library )
list(APPEND _IMPORT_CHECK_FILES_FOR_rmw_dds_common::rmw_dds_common_library "${_IMPORT_PREFIX}/lib/librmw_dds_common.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
