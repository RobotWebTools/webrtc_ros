cmake_minimum_required(VERSION 2.8.3)
project(webrtc_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  webrtc
  image_transport
  nodelet
)

execute_process(
  COMMAND ./checkout_async_web_server_cpp.sh
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/
)

find_package(OpenSSL)

catkin_package(
  CATKIN_DEPENDS cv_bridge roscpp webrtc image_transport nodelet
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${WEBRTC_INCLUDE_DIRS}
  async_web_server_cpp_src/include
)

add_definitions(${WEBRTC_DEFINITIONS})

file(GLOB_RECURSE ASYNC_WEB_SERVER_CPP_SRCS "async_web_server_cpp_src/src/**.cpp")
add_library(async_web_server_cpp_cpp11 ${ASYNC_WEB_SERVER_CPP_SRCS})
target_link_libraries(async_web_server_cpp_cpp11 ${OPENSSL_LIBRARIES} boost_system boost_thread boost_regex pthread)

add_library(webrtc_ros_server
  src/ros_video_capturer.cpp
  src/ros_media_device_manager.cpp

  src/webrtc_ros_message.cpp
  src/configure_message.cpp
  src/sdp_message.cpp
  src/ice_candidate_message.cpp

  src/ros_trace_context.cpp

  src/webrtc_ros_server.cpp
  src/webrtc_client.cpp
  src/ros_video_capturer.cpp
  src/ros_video_renderer.cpp
)
target_link_libraries(webrtc_ros_server
  ${catkin_LIBRARIES}
  ${WEBRTC_LIBRARIES}
  async_web_server_cpp_cpp11
)

add_executable(webrtc_ros_server_node
  src/webrtc_ros_server_node.cpp
)
target_link_libraries(webrtc_ros_server_node
  ${catkin_LIBRARIES}
  webrtc_ros_server
)

add_library(webrtc_ros_server_nodelet
  src/webrtc_ros_server_nodelet.cpp
)
target_link_libraries(webrtc_ros_server_nodelet
  ${catkin_LIBRARIES}
  webrtc_ros_server
)


#############
## Install ##
#############

# Mark nodes and libraries for installation
install(TARGETS
  async_web_server_cpp_cpp11
  webrtc_ros_server
  webrtc_ros_server_node
  webrtc_ros_server_nodelet

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark nodelet plugin configuration for installation
install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY web
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
