cmake_minimum_required(VERSION 2.8.3)
project(ximea_camera)

set(XIMEA_INCLUDE_PATH /usr/include)
set(XIMEA_LIBRARY_PATH /usr/lib)
set(XIMEA_LIBRARY m3api)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  sensor_msgs
  cv_bridge
  camera_info_manager
  roslint
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV 2 REQUIRED COMPONENTS core highgui)

## Generate messages in the 'msg' folder ...
## -----------------------------------------
add_message_files(
   FILES
   XimeaCamSensorMsg.msg
   XimeaMeta.msg
)

## Generate added messages and services with any dependencies listed here ...
# ---------------------------------------------------------------------------
generate_messages(
   DEPENDENCIES
   sensor_msgs   
   std_msgs
)

catkin_package(
  LIBRARIES ximea_camera
  CATKIN_DEPENDS roscpp roslint sensor_msgs message_runtime
  DEPENDS system_lib TinyXML boost
  DEPENDS opencv2
)

include_directories(include)

include_directories(
  ${XIMEA_INCLUDE_PATH}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(XIMEA_SRC
    src/ximea_driver.cpp
    src/ximea_ros_driver.cpp
    src/ximea_ros_cluster.cpp
    src/ximea.cpp
)

add_executable(ximea_camera_node ${XIMEA_SRC})
target_link_libraries(
    ximea_camera_node
    ${XIMEA_LIBRARY}
    ${catkin_LIBRARIES}
    yaml-cpp
    ${Boost_LIBRARIES}
)

add_executable(saver_ximea src/saver_ximea.cpp)
add_dependencies(saver_ximea ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(saver_ximea
  ${catkin_LIBRARIES}
)


# ROSLINT
set(ROSLINT_CPP_OPTS  "--filter=-whitespace/line_length,-readability/todo" )
roslint_cpp()
