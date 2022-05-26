cmake_minimum_required(VERSION 2.8.3)
project(rviz_plugin)
find_package(catkin REQUIRED COMPONENTS rviz rosbag
        message_generation nav_msgs sensor_msgs diagnostic_msgs geometry_msgs
        cv_bridge
        )

add_compile_options(-std=c++11 -pthread)
SET(CMAKE_CXX_FLAGS -pthread)

add_message_files(
   FILES
   Pid.msg
   Selectedwp.msg
   NextDest.msg
   ManualCtrl.msg
   userCmd.msg
)

generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
   nav_msgs
        geometry_msgs
   diagnostic_msgs
 )


catkin_package(CATKIN_DEPENDS roscpp
        DEPENDS roscpp)
include_directories(${catkin_INCLUDE_DIRS} include/)
link_directories(${catkin_LIBRARY_DIRS})

find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL)
# find_package(Qt5Multimedia)
# set variable names already used with Qt4
set(QT_LIBRARIES Qt5::Widgets)
set(QTVERSION ${Qt5Widgets_VERSION})

## Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -frounding-math -Werror=return-type")

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
qt5_wrap_cpp(MOC_FILES
        include/waypoint_nav_frame.h
        include/waypoint_nav_tool.h
        include/pid_plugin.h
        include/mission_plugin.h
        include/ManualControl.h
        include/drive_widget.h
)

  # Convert the Qt Signals and Slots for QWidget events
qt5_wrap_ui(UIC_FILES
  ui/WaypointNavigation.ui
  ui/camera.ui
  ui/mission.ui
  ui/pid.ui
        ui/pixhawkSensor.ui
        ui/car_communication.ui
        ui/camera_resp.ui
)

include_directories(${CMAKE_CURRENT_BINARY_DIR} inlcude/)


# Plugin Source
set(SOURCE_FILES
  src/waypoint_nav_frame.cpp
  src/waypoint_nav_tool.cpp
  src/pid_plugin.cpp
  src/mission_plugin.cpp
  src/ManualControl.cpp
  src/drive_widget.cpp
  ${MOC_FILES}
        src/imagelabel.cpp include/imagelabel.h
        src/imgreceiver.cpp include/imgreceiver.h
        src/camera_plugin.cpp
        include/camera_plugin.h
        include/pid_plugin.h
        include/json.hpp
        inlcude/ src/pixhawkSensor.cpp include/pixhawkSensor.h src/carCommunication.cpp include/carCommunication.h)

set(LIB_NAME rviz_plugin_lib)
add_library(${LIB_NAME} ${SOURCE_FILES} ${MOC_SOURCES} ${UIC_FILES})
target_link_libraries(${LIB_NAME}
  ${catkin_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(rviz_plugin_lib rviz_plugin_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
## Install rules

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

install(TARGETS ${LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

## Install rules
install(FILES
  plugin_description.xml
  #camera_plugin_description.xml
  #waypoint_nav_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)