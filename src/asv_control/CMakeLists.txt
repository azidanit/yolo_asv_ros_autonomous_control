cmake_minimum_required(VERSION 3.0.2)
project(asv_control)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
        roscpp
        cv_bridge
        image_transport
        geometry_msgs
        sensor_msgs
        visualization_msgs
        rviz_plugin
        comm_stm
        nav_msgs
        tf
        vision_msgs
)

find_package( OpenCV REQUIRED )

#File GLOB Recursive
file(GLOB NALA_SRCS src/*.cpp src/misi/*.cpp src/widget/*.cpp)
file(GLOB NALA_HDRS include/*.h include/misi/*.h include/widget/*.h)

#QT AUTOCOMPILE UI TO CPP
find_package( Qt5 COMPONENTS Widgets Core Gui Concurrent REQUIRED )
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
qt5_wrap_ui(QT_MOC_UI ui/mainwindow.ui)
qt5_wrap_cpp(QT_MOC_HPP
        ${NALA_HDRS})


###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES asv_control
        CATKIN_DEPENDS roscpp std_msgs rviz_plugin
        DEPENDS roscpp
)

###########
## Build ##
###########
## Specify additional locations of header files
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Qt5Widgets_INCLUDE_DIRS}
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5Gui_INCLUDE_DIRS}
        ${Qt5Concurrent_INCLUDE_DIRS}
)

link_directories(
        ${catkin_LIBRARY_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
        ${NALA_SRCS}
        ${QT_MOC_HPP}
        ${QT_MOC_UI}

        )

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node ${NALA_SRCS} ${QT_MOC_HPP})

qt5_use_modules(${PROJECT_NAME}_node Widgets Core Gui Concurrent)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${QT_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############
install(DIRECTORY ui DESTINATION ${CMAKE_INSTALL_PREFIX}/share/asv_control)
install(DIRECTORY resource DESTINATION ${CMAKE_INSTALL_PREFIX}/share/asv_control)