cmake_minimum_required(VERSION 2.8.3)
project(cpp_web_server)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenSSL)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES cpp_web_server
 DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIRS}
)

add_library(cpp_web_server src/http_server.cpp
  src/http_connection.cpp src/http_request_parser.cpp
  src/http_reply.cpp src/http_request_handler.cpp
  src/websocket_connection.cpp src/websocket_request_handler.cpp
  src/websocket_message.cpp src/http_request.cpp)
target_link_libraries(cpp_web_server
  ${OPENSSL_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############


## Mark executables and/or libraries for installation
install(TARGETS cpp_web_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cpp_web_server.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
