#!/bin/bash

# find sip for python2.7 first
export PATH=/usr/local/Cellar/python@2/2.7.17/Frameworks/Python.framework/Versions/2.7/bin:$PATH

# rename sip.h for python3
mv /usr/local/include/sip.h /usr/local/include/sip.h.bak

# remove opecv3
rm -rf src/opencv3
# brew install opencv@3
# brew unlink opencv
# brew link --force opencv@3

# edit rospack
# https://github.com/ros/rospack/pull/80/files

# add the following line to src/qt_gui_core/qt_gui_cpp/CMakeLists.txt
# set(Boost_LIBRARIES /usr/local/lib/libboost_filesystem.dylib /usr/local/lib/libboost_system.dylib)

# remove "signals" from the following CMakeLists.txt as boost_signals doesn't exist since boost 1.68
# src/geometry/tf/CMakeLists.txt
# src/geometry2/tf2/CMakeLists.txt
# src/ros_comm/message_filters/CMakeLists.txt
# src/ros_comm/roscpp/CMakeLists.txt
# src/rviz/CMakeLists.txt

# replace logWarn and logError with CONSOLE_BRIDGE_logWarn and CONSOLE_BRIDGE_logError respectively in src/geometry2/tf2/src/buffer_core.cpp
# see https://github.com/ros-gbp/geometry2-release/issues/2

# add the following line to src/geometric_shapes/CMakeLists.txt
# set(ASSIMP_LIBRARIES /usr/local/lib/libassimp.5.dylib)

# edit src/cmake_modules/cmake/Modules/FindUUID.cmake as follows
# <   find_path(UUID_INCLUDE_DIRS uuid/uuid.h)
# ---
# >   find_path(UUID_INCLUDE_DIRS ossp/uuid.h)

# edit src/actionlib/CMakeLists.txt as follows
# < include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
# ---
# > include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# edit actionlib
# https://github.com/ros/actionlib/pull/105/files

# edit src/vision_opencv/cv_bridge/CMakeLists.txt as follows
# <   find_package(Boost REQUIRED python)
# ---
# >   set(Boost_LIBRARIES /usr/local/lib/libboost_python27.dylib)
# better solution for boost >= 1.67 (not tested yet) 
# <   find_package(Boost REQUIRED python)
# ---
# >   find_package(Boost REQUIRED python27)
# 

# edit src/rviz/src/rviz/CMakeLists.txt as follows
# <   yaml-cpp
# ---
# >   /usr/local/lib/libyaml-cpp.dylib

./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False

# restore sip.h
mv /usr/local/include/sip.h.bak /usr/local/include/sip.h
