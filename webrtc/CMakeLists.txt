cmake_minimum_required(VERSION 3.0.0)
project(webrtc)

if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build/depot_tools" OR
   NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build/webrtc")
    message(STATUS "*********************************************************")
    message(STATUS "*                                                       *")
    message(STATUS "*  On the first run, a lot of stuff will be downloaded  *")
    message(STATUS "*  This may take a while...                             *")
    message(STATUS "*                                                       *")
    message(STATUS "*********************************************************")
endif()

message(STATUS "Fetching latest depot_tools from chromium")
execute_process(
    COMMAND "build/get_depot_tools"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    RESULT_VARIABLE get_depot_tools_result
)
if(NOT ${get_depot_tools_result} EQUAL 0)
    message(FATAL_ERROR "cannot update depot_tools")
endif()

message(STATUS "Fetching latest WebRTC source code")
execute_process(
    COMMAND "build/get_webrtc_source"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    RESULT_VARIABLE get_webrtc_source_result
)
if(NOT ${get_webrtc_source_result} EQUAL 0)
    message(FATAL_ERROR "cannot update WebRTC source code")
endif()

message(STATUS "Configuring WebRTC build system")
execute_process(
    COMMAND "build/prepare_webrtc_build" "${CMAKE_CURRENT_BINARY_DIR}/ninja"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    RESULT_VARIABLE prepare_webrtc_build_result
)
if(NOT ${prepare_webrtc_build_result} EQUAL 0)
    message(FATAL_ERROR "cannot prepare WebRTC build system")
endif()

add_custom_target(webrtc ALL
    COMMAND ninja -v -C "${CMAKE_CURRENT_BINARY_DIR}/ninja" webrtc stunserver turnserver relayserver
    COMMAND "build/update_cmake_config" "${CMAKE_CURRENT_SOURCE_DIR}/build/webrtc/src" "${CMAKE_CURRENT_BINARY_DIR}/ninja" "${CMAKE_CURRENT_BINARY_DIR}/output" "${CMAKE_INSTALL_PREFIX}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

install(FILES "package.xml" DESTINATION "share/webrtc")
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/output/webrtcConfig.cmake"
    DESTINATION "share/webrtc/cmake"
)
install(
    DIRECTORY "build/webrtc/src/webrtc"
    DESTINATION "include"
    FILES_MATCHING PATTERN "*.h"
)
install(
    DIRECTORY "build/webrtc/src/third_party/jsoncpp/source/include/"
    DESTINATION "include/webrtc/3rdparty"
    FILES_MATCHING PATTERN "*.h"
)
install(
    DIRECTORY "build/webrtc/src/third_party/libyuv/include/"
    DESTINATION "include/webrtc/3rdparty"
    FILES_MATCHING PATTERN "*.h"
)
install(SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/output/make_install.cmake")

