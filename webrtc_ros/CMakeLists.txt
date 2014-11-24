cmake_minimum_required(VERSION 2.8.3)
project(webrtc_ros)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  webrtc
  image_transport
  nodelet
  cpp_web_server
)


find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK2 REQUIRED gtk+-2.0)

include_directories(${GTK2_INCLUDE_DIRS})
link_directories(${GTK2_LIBRARY_DIRS})
add_definitions(${GTK2_CFLAGS_OTHER})


#set(CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}")
#add_definitions(-DDEBUG)
#add_definitions(-D_DEBUG)
add_definitions(-DPOSIX)
add_definitions(-DLINUX)
add_definitions(-DUSE_WEBRTC_DEV_BRANCH)
add_definitions(-DHAVE_WEBRTC_VIDEO)
add_definitions(-DHAVE_WEBRTC_VOICE)
add_definitions(-DHAVE_SRTP)
add_definitions(-DSRTP_RELATIVE_PATH)
add_definitions(-DUSE_SYSTEM_LIBSRTP)
add_definitions(-DLIBPEERCONNECTION_LIB)
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

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge roscpp webrtc image_transport nodelet cpp_web_server
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable



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
)

add_executable(webrtc_ros_server_node
  src/webrtc_ros_server_node.cpp
)
target_link_libraries(webrtc_ros_server_node
  webrtc_ros_server
)


## Specify libraries to link a library or executable target against

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS webrtc_ros_server webrtc_ros_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_webrtc_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
