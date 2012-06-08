cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/)
include(FindPkgConfig)

find_package(Qt4 4.6.0 COMPONENTS QtCore QtGui REQUIRED )
set( QT_USE_QTGUI TRUE )
set( QT_USE_QTXML TRUE )
set( QT_USE_QTOPENGL TRUE )
set( QT_USE_PHONON TRUE )
set( QT_USE_QTNETWORK TRUE )
include(${QT_USE_FILE})

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

pkg_check_modules(OpenCV opencv)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(OpenHRP REQUIRED)
include_directories(${OPENHRP_INCLUDE_DIRS})
link_directories(${OPENHRP_LIBRARY_DIRS})

find_package(LibXml2)
include_directories(${LIBXML2_INCLUDE_DIR})

set(sources 
  monitor.cpp moc_monitor.cpp
  canvaswidget.cpp moc_canvaswidget.cpp 
  mymodel.cpp moc_mymodel.cpp
  SimulationThread.cpp moc_SimulationThread.cpp
  BodyState.cpp
  SceneState.cpp
  GLmodel.cpp
  BodyRTC.cpp
  Simulator.cpp
  Project.cpp
  ProjectUtil.cpp
  OpenRTMUtil.cpp
  main.cpp)

foreach( localOutPath ${sources} )
  if(${localOutPath} MATCHES "^moc_.*" )
    string(REGEX REPLACE "^moc_" "" localInPath ${localOutPath})
    string(REGEX REPLACE "cpp$" "h" localInPath ${localInPath})
    QT4_GENERATE_MOC(${localInPath} ${localOutPath})
  endif()
endforeach(localOutPath)

add_executable(qtgl ${sources})
target_link_libraries(qtgl ${QT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${OPENHRP_LIBRARIES} ${OpenCV_LIBRARIES} ${LIBXML2_LIBRARIES})


