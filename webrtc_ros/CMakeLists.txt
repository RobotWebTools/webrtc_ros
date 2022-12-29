cmake_minimum_required(VERSION 3.5)
project(webrtc_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(async_web_server_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_generation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(webrtc REQUIRED)
find_package(X11 REQUIRED)
find_package(jsoncpp REQUIRED)

add_message_files(
    FILES
    IceServer.msg
)

add_service_files(
    FILES
    GetIceServers.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(
  include
  include/${PROJECT_NAME}
  ${webrtc_INCLUDE_DIRS}
)
add_definitions(${webrtc_DEFINITIONS})

# This library contains code that cannot be compiled with c++11 enabled
# This is due to ABI incompatibilities between the two
# While most of ROS appears to work with the webrtc compiler options,
# this is needed for async_web_server_cpp, which relies on Boost ASIO types
# Boost ASIO cannot be mixed between c++11 and non-c++11 compilation units


add_library(${PROJECT_NAME}_server_noncpp11
  src/webrtc_web_server.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}_server_noncpp11
    async_web_server_cpp
    cv_bridge
    image_transport
    message_runtime
    rclcpp
    std_msgs
    webrtc
)


# This library is built with the same options used to build the webrtc library
add_library(${PROJECT_NAME}_server
  src/configure_message.cpp
  src/ice_candidate_message.cpp
  src/image_transport_factory.cpp
  src/ros_video_capturer.cpp
  src/ros_video_renderer.cpp
  src/sdp_message.cpp
  src/webrtc_client.cpp
  src/webrtc_ros_message.cpp
  src/webrtc_ros_server.cpp
  src/webrtc_ros_json_parser.cpp
)
ament_target_dependencies(
    ${PROJECT_NAME}_server
    async_web_server_cpp
    cv_bridge
    image_transport
    message_runtime
    rclcpp
    std_msgs
    webrtc
    ${webrtc_LIBRARIES}
    ${X11_LIBRARIES}
    jsoncpp_object
)

set_target_properties(${PROJECT_NAME}_server PROPERTIES COMPILE_OPTIONS "-std=c++14")

add_executable(${PROJECT_NAME}_server_node
  src/webrtc_ros_server_node.cpp
)
ament_target_dependencies(
    ${PROJECT_NAME}_server_node
  ${PROJECT_NAME}_server
  ${PROJECT_NAME}_server_noncpp11
}

set_target_properties(${PROJECT_NAME}_server_node PROPERTIES COMPILE_OPTIONS "-std=c++14")

#add_library(${PROJECT_NAME}_server_nodelet
#  src/webrtc_ros_server_nodelet.cpp
#)
#target_link_libraries(${PROJECT_NAME}_server_nodelet
#  ${catkin_LIBRARIES}
#  ${PROJECT_NAME}_server
#)

#############
## Install ##
#############
install(TARGETS 
    ${PROJECT_NAME}_server
    ${PROJECT_NAME}_server_node
    ${PROJECT_NAME}_server_noncpp11
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install web files
install(DIRECTORY web
  DESTINATION share/${PROJECT_NAME}
  PATTERN "Gruntfile.js" EXCLUDE
  PATTERN "node_modules" EXCLUDE
  PATTERN "package.json" EXCLUDE
  PATTERN ".gitignore" EXCLUDE
)

# Install license files
install(FILES
  LICENSE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES
  LICENSE_webrtc_adapter
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
