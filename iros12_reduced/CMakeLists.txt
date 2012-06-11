cmake_minimum_required(VERSION 2.4)

include(FindPkgConfig)
pkg_check_modules(OPENHRP openhrp3.1)
include_directories(${OPENHRP_INCLUDE_DIRS})
link_directories(${OPENHRP_LIBRARY_DIRS})

add_executable(findgoal3 findgoal3.cpp SphereTree.cpp CdShape.cpp DistUtil.cpp problem.cpp CdShapeUtil.cpp SphereTreeUtil.cpp CustomCD.cpp)
target_link_libraries(findgoal3 ${OPENHRP_LIBRARIES} )

add_executable(findpath3 findpath3.cpp problem.cpp SphereTree.cpp CdShape.cpp DistUtil.cpp problem.cpp CdShapeUtil.cpp SphereTreeUtil.cpp CustomCD.cpp)
target_link_libraries(findpath3 ${OPENHRP_LIBRARIES} )



