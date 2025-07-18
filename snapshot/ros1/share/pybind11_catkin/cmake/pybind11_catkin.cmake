cmake_minimum_required(VERSION 3.4)

# As of July 31, 2020 pybind11 upgraded to CMake version 3.8 (effectively dropping 16.04).
# To be able to configure, we need to set CMP0069 (interprocedural optimization) to NEW
# cmake_policy(SET CMP0069 NEW)

# Configure pybind11 using the cmake file provided by the upstream package.
# This finds python includes and libs and defines pybind11_add_module()
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")
set(PYBIND11_PYTHON_VERSION ${PYTHON_VERSION_STRING})
include(pybind11Config)
include(pybind11Common)
include(pybind11Tools)

# set variables used by pybind11_add_module()
if(TRUE)
  set(pybind11_catkin_INCLUDE_DIRS "${pybind11_catkin_INCLUDE_DIRS}/pybind11_catkin")
endif()
set(PYBIND11_INCLUDE_DIR "${pybind11_catkin_INCLUDE_DIRS}")
list(APPEND pybind11_catkin_INCLUDE_DIRS "${PYTHON_INCLUDE_DIRS}")

macro(pybind_add_module target_name other)
    pybind11_add_module(${ARGV})
    target_link_libraries(${target_name} PRIVATE ${PYTHON_LIBRARIES} ${catkin_LIBRARIES})
    set_target_properties(${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})
endmacro(pybind_add_module)
