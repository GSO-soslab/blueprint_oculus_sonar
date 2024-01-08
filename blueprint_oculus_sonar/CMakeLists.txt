cmake_minimum_required(VERSION 3.0.2)
project(blueprint_oculus_sonar)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# SET(OpenCV_DIR /home/lin/develop/3rd/opencv/install/opencv-4.2.0/lib/cmake/opencv4/)
find_package(OpenCV 4.2.0 REQUIRED)

# Standard dependencies
# SET(cv_bridge_DIR /home/lin/develop/ros/ros_ws/devel/share/cv_bridge/cmake/)
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamic_reconfigure
  g3log_ros
  nodelet
  nodelet_topic_tools
  roscpp
  liboculus
  message_generation
  cv_bridge
)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/OculusConfig.cfg
)

add_message_files(
   FILES
   RawData.msg
   OculusInfo.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS liboculus g3log_ros image_transport message_runtime cv_bridge
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  include/
  ${catkin_INCLUDE_DIRS}
)

list(APPEND thirdparty_libraries
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(oculus_sonar_nodelet
            include/blueprint_oculus_sonar/oculus_nodelet.cpp)
target_link_libraries(oculus_sonar_nodelet ${thirdparty_libraries})
class_loader_hide_library_symbols(oculus_sonar_nodelet)

####
add_executable(oculus_sonar_driver src/oculus_node.cpp)

target_link_libraries(oculus_sonar_driver
  ${thirdparty_libraries}
)

add_dependencies(oculus_sonar_driver
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
)

####
# install(TARGETS 
#         oculus_sonar_driver
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

# install(TARGETS oculus_sonar_nodelet
# 	      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# install( DIRECTORY launch/
#          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )
# install( DIRECTORY rqt_config/
#          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rqt_config )

# install( FILES nodelet_plugins.xml
#   		   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})