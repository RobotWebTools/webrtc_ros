cmake_minimum_required(VERSION 2.8.3)
project(webrtc)

find_package(catkin REQUIRED)

set(webrtc_src_root ${PROJECT_SOURCE_DIR}/src)
set(webrtc_build_root ${CMAKE_BINARY_DIR}/out/Release)

execute_process(
  COMMAND ./checkout_source.sh ${CMAKE_BINARY_DIR}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/
)

catkin_package(
  INCLUDE_DIRS
  CFG_EXTRAS webrtc-extras.cmake
)

#############
##  Build  ##
#############

add_custom_target(webrtc ALL
  COMMAND ./build_webrtc.sh ${webrtc_build_root}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(TARGETS jingle_peerconnection_so
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Copy header files
#install(DIRECTORY webrtc_src/talk
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)
