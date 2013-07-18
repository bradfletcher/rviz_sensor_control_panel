cmake_minimum_required(VERSION 2.8.3)
project(rviz_sensor_control_panel)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package( catkin REQUIRED COMPONENTS std_msgs sensor_msgs message_generation)


## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
# add_definitions(-DQT_NO_KEYWORDS)



add_message_files(
  FILES
  FleaCommand.msg
  HokuyoCommand.msg
)

generate_messages(
    DEPENDENCIES
    sensor_msgs
    std_msgs
)



catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp rospy message_runtime std_msgs sensor_msgs 
    DEPENDS ach QtCore QtGui )
    
## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/hubo_init.cpp
  src/hubo_init_slots.cpp
  src/FlowLayout.cpp
  include/hubo_init.h
  include/FlowLayout.h
  ${MOC_FILES}
)

include(${QT_USE_FILE})

include_directories( include )

## Set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  include/hubo_init.h
  include/FlowLayout.h
)

add_library( ${PROJECT_NAME} ${SOURCE_FILES} )
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ach huboparams)
add_dependencies( ${PROJECT_NAME} ${PROJECT_NAME}_generate_message_cpp )

## Link the library with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above.
##
## Although this puts "rviz_plugin_tutorials" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
