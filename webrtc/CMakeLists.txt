cmake_minimum_required(VERSION 2.8.12)
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

message(STATUS "Fetching depot_tools from Chromium")
execute_process(
    COMMAND "build/get_depot_tools"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    RESULT_VARIABLE get_depot_tools_result
)
if(NOT ${get_depot_tools_result} EQUAL 0)
    message(FATAL_ERROR "cannot fetch depot_tools")
endif()

message(STATUS "Fetching Ninja build tool")
execute_process(
    COMMAND "build/get_ninja"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    RESULT_VARIABLE get_ninja_result
)
if(NOT ${get_ninja_result} EQUAL 0)
    message(FATAL_ERROR "cannot fetch Ninja build tool")
endif()

message(STATUS "Fetching WebRTC source code")
execute_process(
    COMMAND "build/get_webrtc_source"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    RESULT_VARIABLE get_webrtc_source_result
)
if(NOT ${get_webrtc_source_result} EQUAL 0)
    message(FATAL_ERROR "cannot fetch WebRTC source code")
endif()

message(STATUS "Fetching GN build tool from Google")
execute_process(
    COMMAND "build/get_gn"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    RESULT_VARIABLE get_gn_result
)
if(NOT ${get_gn_result} EQUAL 0)
    message(FATAL_ERROR "cannot fetch GN build tool")
endif()

message(STATUS "Configuring WebRTC build system")
message(${CMAKE_CURRENT_SOURCE_DIR})
execute_process(
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/build/webrtc/src/build/util/lastchange.py" "LASTCHANGE"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build/webrtc/src/build/util/"
)
execute_process(
    COMMAND "build/prepare_webrtc_build" "${CMAKE_CURRENT_BINARY_DIR}/ninja"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    RESULT_VARIABLE prepare_webrtc_build_result
)
if(NOT ${prepare_webrtc_build_result} EQUAL 0)
    message(FATAL_ERROR "cannot prepare WebRTC build system")
endif()

add_custom_target(webrtc ALL
    COMMAND "build/depot_tools/ninja" -C "${CMAKE_CURRENT_BINARY_DIR}/ninja" webrtc stunserver turnserver
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
    DIRECTORY "build/webrtc/src/"
    DESTINATION "include/webrtc"
    FILES_MATCHING PATTERN "*.h"
)
install(
    DIRECTORY "build/webrtc/src/third_party/jsoncpp/generated/"
    DESTINATION "include/webrtc/3rdparty/json"
    FILES_MATCHING PATTERN "*.h"
)
install(
    DIRECTORY "build/webrtc/src/third_party/libyuv/include/"
    DESTINATION "include/webrtc/3rdparty"
    FILES_MATCHING PATTERN "*.h"
)
install(
    DIRECTORY "build/webrtc/src/third_party/abseil-cpp/"
    DESTINATION "include/webrtc"
    FILES_MATCHING PATTERN "*.h"
)
install(SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/output/make_install.cmake")

