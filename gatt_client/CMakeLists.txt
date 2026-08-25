# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# Use the LVGL checkout shipped with the companion TFT project when it is
# available locally.  The manifest remains as the normal fallback for a clean
# checkout where the component manager is enabled.
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/components/lvgl/CMakeLists.txt")
    list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/components/lvgl")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(gatt_client)
