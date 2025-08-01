## mjbogusz: find_package() and if() disabled until ament can handle complex paths resolving to /usr/include
## always use locally built yaml-cpp for now
# find_package(yaml-cpp QUIET)

# if(NOT yaml-cpp_FOUND)
  # add the local Modules directory to the modules path
  if(WIN32)
    set(yaml-cpp_DIR "${yaml_cpp_vendor_DIR}/../../../opt/yaml_cpp_vendor/CMake")
  else()
    set(yaml-cpp_DIR "${yaml_cpp_vendor_DIR}/../../../opt/yaml_cpp_vendor/lib/cmake/yaml-cpp")
  endif()
  message(STATUS "Setting yaml-cpp_DIR to: '${yaml-cpp_DIR}'")
# endif()

find_package(yaml-cpp CONFIG REQUIRED QUIET)

set(yaml_cpp_vendor_LIBRARIES ${YAML_CPP_LIBRARIES})
set(yaml_cpp_vendor_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})

list(APPEND yaml_cpp_vendor_TARGETS yaml-cpp)
if(WIN32)
  set_target_properties(${YAML_CPP_LIBRARIES} PROPERTIES INTERFACE_COMPILE_DEFINITIONS YAML_CPP_DLL)
endif()
