cmake_minimum_required(VERSION 2.4)

include(FindPkgConfig)
pkg_check_modules(OpenCV opencv)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBRARIES})
