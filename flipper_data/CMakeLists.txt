##############################################################################
# qtros Environment
##############################################################################

cmake_minimum_required(VERSION 3.0.2)
project(flipper_data)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    std_msgs
    DEPENDS
)





##############################################################################
# Qt Environment
##############################################################################

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 COMPONENTS Core Gui Widgets)



##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/${PROJECT_NAME}/*.h)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
#QT5_WARP_UI(QT_FORMS_HPP ${QT_FORMS})

include_directories(
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

add_definitions( -std=c++11 -fPIC)



##############################################################################
# Sources
##############################################################################

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set( QT_SOURCES
  src/main.cpp
  include/${PROJECT_NAME}/mainwindow.h
  src/mainwindow.cpp
)



##############################################################################
# Binaries
##############################################################################

add_executable(${PROJECT_NAME} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
)
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
