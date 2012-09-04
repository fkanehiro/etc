cmake_minimum_required(VERSION 2.4)

include(FindPkgConfig)

pkg_check_modules(HMC2 hmc2)
include_directories(${HMC2_INCLUDE_DIRS})
link_directories(${HMC2_LIBRARY_DIRS})

pkg_check_modules(HRPSYS hrpsys-base)
include_directories(${HRPSYS_INCLUDE_DIRS})
link_directories(${HRPSYS_LIBRARY_DIRS})

set(target polysplit)

add_executable(polysplit
  main.cpp
  polysplit.cpp
  )

target_link_libraries(polysplit
  ${HMC2_LIBRARIES} 
  hrpsysUtil
  )
