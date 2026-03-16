# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

execute_process(
    COMMAND python ${CMAKE_SOURCE_DIR}/managed_components/external-managed-fix/fix.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Wheelboard-SDM26-ESP32)
