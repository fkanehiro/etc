cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl)

find_package(PCL 1.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cluster_extraction cluster_extraction.cpp)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})

add_executable (planar_segmentation planar_segmentation.cpp)
target_link_libraries (planar_segmentation ${PCL_LIBRARIES})

add_executable (region_growing_segmentation region_growing_segmentation.cpp)
target_link_libraries (region_growing_segmentation ${PCL_LIBRARIES})

add_executable (statistical_removal statistical_removal.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})

add_executable (plane_extraction plane_extraction.cpp)
target_link_libraries (plane_extraction ${PCL_LIBRARIES})

add_executable (moving_least_squares moving_least_squares.cpp)
target_link_libraries (moving_least_squares ${PCL_LIBRARIES})

add_executable (normals normals.cpp)
target_link_libraries (normals ${PCL_LIBRARIES})
