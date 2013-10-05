cmake_minimum_required(VERSION 2.4)

include(FindPkgConfig)
pkg_check_modules(HMC2 hmc2)
include_directories(${HMC2_INCLUDE_DIRS})
link_directories(${HMC2_LIBRARY_DIRS})

pkg_check_modules(OCTOMAP octomap) 
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})

add_executable(findgoal findgoal.cpp problem.cpp)
target_link_libraries(findgoal ${HMC2_LIBRARIES} )

add_executable(findpath findpath.cpp problem.cpp)
target_link_libraries(findpath ${HMC2_LIBRARIES} )

add_executable(genogmap genogmap.cpp)
target_link_libraries(genogmap ${HMC2_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(sttest sttest.cpp SphereTree.cpp DistUtil.cpp)
target_link_libraries(sttest ${HMC2_LIBRARIES})

add_executable(cdtest cdtest.cpp SphereTree.cpp CdShape.cpp DistUtil.cpp problem.cpp CdShapeUtil.cpp SphereTreeUtil.cpp)
target_link_libraries(cdtest ${HMC2_LIBRARIES} )

add_executable(findgoal2 findgoal2.cpp SphereTree.cpp CdShape.cpp DistUtil.cpp problem.cpp CdShapeUtil.cpp SphereTreeUtil.cpp CustomCD.cpp)
target_link_libraries(findgoal2 ${HMC2_LIBRARIES} )

add_executable(findgoal3 findgoal3.cpp SphereTree.cpp CdShape.cpp DistUtil.cpp problem.cpp CdShapeUtil.cpp SphereTreeUtil.cpp CustomCD.cpp RobotUtil.cpp)
target_link_libraries(findgoal3 ${HMC2_LIBRARIES} )

add_executable(iktest iktest.cpp problem.cpp RobotUtil.cpp)
target_link_libraries(iktest ${HMC2_LIBRARIES} )

add_executable(cfgtest cfgtest.cpp problem.cpp RobotUtil.cpp)
target_link_libraries(cfgtest ${HMC2_LIBRARIES} )

add_executable(test iktest.cpp problem.cpp RobotUtil.cpp)
target_link_libraries(iktest ${HMC2_LIBRARIES} )

add_executable(findpath2 findpath2.cpp problem.cpp SphereTree.cpp CdShape.cpp DistUtil.cpp problem.cpp CdShapeUtil.cpp SphereTreeUtil.cpp CustomCD.cpp)
target_link_libraries(findpath2 ${HMC2_LIBRARIES} )

add_executable(findpath3 findpath3.cpp problem.cpp SphereTree.cpp CdShape.cpp DistUtil.cpp problem.cpp CdShapeUtil.cpp SphereTreeUtil.cpp CustomCD.cpp RobotUtil.cpp)
target_link_libraries(findpath3 ${HMC2_LIBRARIES} )

add_executable(findpath4 findpath4.cpp problem.cpp SphereTree.cpp CdShape.cpp DistUtil.cpp problem.cpp CdShapeUtil.cpp SphereTreeUtil.cpp CustomCD.cpp)
target_link_libraries(findpath4 ${HMC2_LIBRARIES} )

add_executable(execpath execpath.cpp Filter.cpp problem.cpp SphereTreeUtil.cpp SphereTree.cpp DistUtil.cpp ExecUtil.cpp)
target_link_libraries(execpath ${HMC2_LIBRARIES} )

add_executable(findgoalandpath findgoalandpath.cpp problem.cpp SphereTree.cpp CdShape.cpp DistUtil.cpp problem.cpp CdShapeUtil.cpp SphereTreeUtil.cpp CustomCD.cpp RobotUtil.cpp)
target_link_libraries(findgoalandpath ${HMC2_LIBRARIES} )

add_executable(replanning replanning.cpp problem.cpp SphereTree.cpp CdShape.cpp DistUtil.cpp problem.cpp CdShapeUtil.cpp SphereTreeUtil.cpp CustomCD.cpp Filter.cpp ExecUtil.cpp RobotUtil.cpp)
target_link_libraries(replanning ${HMC2_LIBRARIES} )

add_executable(valve valve.cpp problem.cpp SphereTree.cpp CdShape.cpp DistUtil.cpp problem.cpp CdShapeUtil.cpp SphereTreeUtil.cpp CustomCD.cpp RobotUtil.cpp)
target_link_libraries(valve ${HMC2_LIBRARIES} )



