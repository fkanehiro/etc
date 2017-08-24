cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl)

find_package(PCL 1.1 REQUIRED)
if (NOT "${PCL_LIBRARIES}" STREQUAL "")
  list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
endif()

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

add_executable (append_pcd append_pcd.cpp)
target_link_libraries (append_pcd ${PCL_LIBRARIES})

add_executable (filter_pcd filter_pcd.cpp)
target_link_libraries (filter_pcd ${PCL_LIBRARIES})

add_executable (sor sor.cpp)
target_link_libraries (sor ${PCL_LIBRARIES})

add_executable (openni_viewer_simple openni_viewer_simple.cpp)
target_link_libraries (openni_viewer_simple ${PCL_LIBRARIES})

add_executable (openni_grabber openni_grabber.cpp)
target_link_libraries (openni_grabber ${PCL_LIBRARIES})

add_executable (openni2_viewer openni2_viewer.cpp)
target_link_libraries (openni2_viewer ${PCL_LIBRARIES})

add_executable (extract_image extract_image.cpp)
target_link_libraries (extract_image ${PCL_LIBRARIES})

add_executable (extract_region extract_region.cpp)
target_link_libraries (extract_region ${PCL_LIBRARIES})

add_executable (voxel_grid_filter voxel_grid_filter.cpp)
target_link_libraries (voxel_grid_filter ${PCL_LIBRARIES})

add_executable (approximate_voxel_grid_filter approximate_voxel_grid_filter.cpp)
target_link_libraries (approximate_voxel_grid_filter ${PCL_LIBRARIES})
