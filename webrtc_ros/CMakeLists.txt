cmake_minimum_required(VERSION 2.8.3)
project(webrtc_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  webrtc
  image_transport
  nodelet
  async_web_server_cpp
)


find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK2 REQUIRED gtk+-2.0)

include_directories(${GTK2_INCLUDE_DIRS})
link_directories(${GTK2_LIBRARY_DIRS})
add_definitions(${GTK2_CFLAGS_OTHER})

add_definitions(-DDEBUG)
add_definitions(-D_DEBUG)

add_definitions(-DPOSIX)

add_definitions(-DVAR_ARRAYS)
add_definitions(-DHAVE_LRINT)
add_definitions(-DHAVE_LRINTF)
add_definitions(-DOPUS_EXPORT=)
add_definitions(-DOPUS_BUILD)
add_definitions(-D__OPTIMIZE__)

add_definitions(-DSCTP_PROCESS_LEVEL_LOCKS)
add_definitions(-DSCTP_SIMPLE_ALLOCATOR)
add_definitions(-D__Userspace__)
add_definitions(-D__Userspace_os_Linux)

add_definitions(-DWEBRTC_POSIX)
add_definitions(-DWEBRTC_LINUX)
add_definitions(-DWEBRTC_MODULE_UTILITY_VIDEO)
add_definitions(-DWEBRTC_NS_FLOAT)
add_definitions(-DWEBRTC_CODEC_OPUS)
add_definitions(-DWEBRTC_CODEC_GSFMR)
add_definitions(-DEXPAT_RELATIVE_PATH)
add_definitions(-DUSE_X11)
add_definitions(-DSSL_USE_OPENSSL)
add_definitions(-DFEATURE_ENABLE_SSL)
add_definitions(-DHAVE_OPENSSL_SSL_H)
add_definitions(-DLINUX_PULSE)
add_definitions(-DLINUX_ALSA)

add_definitions(-DLINUX)
add_definitions(-DUSE_WEBRTC_DEV_BRANCH)
add_definitions(-DHAVE_WEBRTC_VIDEO)
add_definitions(-DHAVE_WEBRTC_VOICE)
add_definitions(-DHAVE_SRTP)
add_definitions(-DSRTP_RELATIVE_PATH)
add_definitions(-DUSE_SYSTEM_LIBSRTP)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge roscpp webrtc image_transport nodelet async_web_server_cpp
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(ros_webrtc_devices
  src/ros_video_capturer.cpp
  src/ros_media_device_manager.cpp
)

add_library(webrtc_ros_messages
  src/webrtc_ros_message.cpp
  src/configure_message.cpp
  src/sdp_message.cpp
  src/ice_candidate_message.cpp
)

add_library(ros_trace_context
  src/ros_trace_context.cpp
)

add_library(webrtc_ros_server
  src/webrtc_ros_server.cpp
  src/webrtc_client.cpp
  src/ros_video_capturer.cpp
  src/ros_video_renderer.cpp
)
target_link_libraries(webrtc_ros_server
  ${catkin_LIBRARIES}
  ros_webrtc_devices
  webrtc_ros_messages
  ros_trace_context
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
  ros_webrtc_devices
  ros_trace_context
  webrtc_ros_messages
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
