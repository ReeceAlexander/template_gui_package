# MIT License

# Copyright (c) 2022 Kristopher Krasnosky

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


cmake_minimum_required(VERSION 2.8.3)

#get_filename_component(PACKAGE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
#message("Creating Package: ${PACKAGE_NAME}")

project(template_gui_package)  ## this package name is the name of the directory this cmake file is in


################################
##  CONFIGURE THIS FILE HERE  ##
################################

set(ROS_DEPENDS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  message_generation
  )

set(EXTERNAL_DEPENDS
  Qt5Widgets
  Qt5PrintSupport
  OpenCV
  )

set(EXTERNAL_DEPENDS_LIB
  Qt5::Widgets
  Qt5::PrintSupport
  Qt5::Core
  Qt5::Designer
  ${OpenCV_LIBS}
  )

set(INCLUDE_DIRECTORIES
  #${OpenCV_LIBS}
  )


##########################################
##  Be Carefull editing below this line ##
##########################################

## Compile as C++14
add_compile_options(-std=c++14)

## need this for qt auto generated headers to include!
set(CMAKE_INCLUDE_CURRENT_DIR ON)

###
###  Add Non ROS dependencies
###

foreach(DEPEND ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)

###
###  Add any required ros packages here
###
find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDS})

## Add message files (for Feedback.msg)
add_message_files(
  FILES
  MotorPosition.msg  # Include your custom message here
  MotorSpeed.msg
  Buttons.msg
  GoToInfo.msg
  MotorVoltage.msg
  MotorPower.msg
  MotorCurrent.msg
)

# Generate added messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${ROS_DEPENDS}
    message_runtime
  DEPENDS
    ${EXTERNAL_DEPENDS}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIRECTORIES}
  /usr/include/opencv4
)

###########
## Build ##
###########

file(GLOB HDRS
  "include/${PROJECT_NAME}/*.h"
  "include/${PROJECT_NAME}/widgets/*.h"
)
file(GLOB SRCS
  "src/*.cpp"
  "src/*.hpp"
  "src/widgets/*.cpp"
)
file(GLOB NODES
  "nodes/*.cpp"
)

###
### QT stuff (required if you want a GUI)
###
set(CMAKE_AUTOMOC ON)  #This will autogenerate qt gui stuff
set(CMAKE_AUTORCC ON)  #this needs to be on so cmake will know what to do with QT resource Files
#set(CMAKE_AUTOUIC ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets Designer)
#find_package(Qt5PrintSupport)

#find all the qt UI stuff
file(GLOB_RECURSE UI
    "include/${PROJECT_NAME}/*.ui"
    "include/${PROJECT_NAME}/widgets/*.ui"
)

message("Adding ${PROJECT_NAME} UI files: " )
foreach(source ${UI})
    message("  - " ${source})
endforeach(source)

#make them into headers
qt5_wrap_ui (UI_HDRS  ${UI})

message("Generating ${PROJECT_NAME} UI Headers: " )
foreach(source ${UI_HDRS})
    message("  - " ${source})
endforeach(source)

# find all resource files
file(GLOB QRC
    "rc/*.qrc"
)

###
### compile
###

add_library(JoystickPlugin SHARED
  "src/widgets/qjoystick.cpp"  # Ensure the path is correct
  "include/widgets/qjoystick.cpp"
)

target_link_libraries(JoystickPlugin
  ${catkin_LIBRARIES}
  ${EXTERNAL_DEPENDS_LIB}
  Qt5::Designer
)

install(TARGETS JoystickPlugin
  DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
)

message("buliding ${PROJECT_NAME} node: ")
foreach(NODE_FILE ${NODES})
  get_filename_component(NODE_NAME ${NODE_FILE} NAME_WE)
  message("  - " ${NODE_NAME})

  add_executable(${NODE_NAME} ${NODE_FILE} ${HDRS} ${SRCS} ${UI} ${QRC})
  add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries (${NODE_NAME} ${catkin_LIBRARIES}  ${EXTERNAL_DEPENDS_LIB} JoystickPlugin Qt5::Designer)
  target_include_directories(${NODE_NAME} PRIVATE "include/${PROJECT_NAME}/")

  list(APPEND NODE_LIST ${NODE_NAME})

endforeach(NODE_FILE)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install resource files
install(FILES ${QRC}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rc
)

install(TARGETS ${NODE_LIST}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

