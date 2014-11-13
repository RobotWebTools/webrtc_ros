cmake_minimum_required(VERSION 2.8.3)
project(webrtc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(EXPAT REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_search_module(OPENSSL REQUIRED openssl)
pkg_check_modules(GTK2 REQUIRED gtk+-2.0)

###################################################
## Declare things to be passed to other projects ##
###################################################


catkin_package(
  INCLUDE_DIRS src
  LIBRARIES jingle webrtc
  DEPENDS
)

add_definitions(-DDEBUG)
add_definitions(-D_DEBUG)


include_directories(
  "src"
  "src/third_party/opus/src/celt"
  "src/third_party/opus/src/silk"
  "src/third_party/opus/src/silk/float"
  "src/third_party/opus/src/include"
  "src/third_party/opus/src/src"
  "src/third_party/libyuv/include"
  "src/third_party/jsoncpp/source/include"
  "src/webrtc/modules/audio_device/linux/"
  "src/webrtc/system_wrappers/interface/"
  "src/webrtc/common_audio/signal_processing/include"
  "src/webrtc/modules/audio_coding/codecs/ilbc/interface/"
  "src/webrtc/modules/audio_coding/codecs/pcm16b/include/"
  "src/webrtc/modules/audio_coding/codecs/isac/main/interface/"
  "src/webrtc/modules/audio_coding/codecs/g711/include"
  "src/webrtc/modules/audio_coding/codecs/cng/include"
  "src/webrtc/modules/audio_coding/codecs/g722/include"
  "src/third_party/usrsctp/usrsctplib"
  "src/third_party/usrsctp/"
  "/usr/include/srtp"
)
include_directories(${EXPAT_INCLUDE_DIRS})
include_directories(${OPENSSL_INCLUDE_DIRS})
include_directories(${GTK2_INCLUDE_DIRS})
link_directories(${GTK2_LIBRARY_DIRS})
add_definitions(${GTK2_CFLAGS_OTHER})

# Enable C++11
set(CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}")
add_definitions(-DPOSIX)




file(GLOB_RECURSE OPUS_SRCS "src/third_party/opus/src/**.c")
foreach(src ${OPUS_SRCS})
  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/third_party/opus/src/.*tests/.*$")
    list(REMOVE_ITEM OPUS_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/third_party/opus/src/.*tests/.*$")
  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/third_party/opus/src/.*doc/.*$")
    list(REMOVE_ITEM OPUS_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/third_party/opus/src/.*doc/.*$")
  if(src MATCHES "_demo.c$")
    list(REMOVE_ITEM OPUS_SRCS "${src}")
  endif(src MATCHES "_demo.c$")
  if(src MATCHES "_FIX.c$")
    list(REMOVE_ITEM OPUS_SRCS "${src}")
  endif(src MATCHES "_FIX.c$")
endforeach(src)
list(REMOVE_ITEM OPUS_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/third_party/opus/src/celt/dump_modes/dump_modes.c")
list(REMOVE_ITEM OPUS_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/third_party/opus/src/src/mlp_train.c")
list(REMOVE_ITEM OPUS_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/third_party/opus/src/src/opus_compare.c")

add_definitions(-DVAR_ARRAYS)
add_definitions(-DHAVE_LRINT)
add_definitions(-DHAVE_LRINTF)
add_definitions(-DOPUS_EXPORT=)
add_definitions(-DOPUS_BUILD)
add_definitions(-D__OPTIMIZE__)
add_library(opus ${OPUS_SRCS})









file(GLOB_RECURSE YUV_SRCS "src/third_party/libyuv/source/**.cc")
foreach(src ${YUV_SRCS})
endforeach(src)

add_library(yuv ${YUV_SRCS})





file(GLOB_RECURSE JSONCPP_SRCS "src/third_party/jsoncpp/source/src/**.cpp")
foreach(src ${JSONCPP_SRCS})
endforeach(src)

add_library(jsoncpp ${JSONCPP_SRCS})






file(GLOB_RECURSE USRSCTP_SRCS "src/third_party/usrsctp/**.c")
foreach(src ${USRSCTP_SRCS})
endforeach(src)

add_definitions(-DSCTP_PROCESS_LEVEL_LOCKS)
add_definitions(-DSCTP_SIMPLE_ALLOCATOR)
add_definitions(-D__Userspace__)
add_definitions(-D__Userspace_os_Linux)
add_library(usrsctp ${USRSCTP_SRCS})











file(GLOB_RECURSE WEBRTC_SRCS "src/webrtc/**.cc" "src/webrtc/**.c")
foreach(src ${WEBRTC_SRCS})
  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/test/.*$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/test/.*$")

  if(src MATCHES "_unittests?.cc?$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "_unittests?.cc?$")
  if(src MATCHES "_unittest_disabled.cc?$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "_unittest_disabled.cc?$")
  if(src MATCHES "_test_helper.cc?$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "_test_helper.cc?$")
  if(src MATCHES "_unittest_helper.cc?$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "_unittest_helper.cc?$")

  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/overrides.*$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/overrides.*$")

  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/examples.*$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/examples.*$")

  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.+/test/.+$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.+/test/.+$")
  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.+/test_framework/.+$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.+/test_framework/.+$")

  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.*tools.*$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.*tools.*$")

  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.+_tests?.cc?$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.+_tests?.cc?$")

  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.+_win.cc?$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.+_win.cc?$")

  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.+_mips.cc?$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.+_mips.cc?$")

  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.+_neon.cc?$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.+_neon.cc?$")

  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.+_windows_impl.cc?$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.+_windows_impl.cc?$")


  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.+_mac.cc?$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.+_mac.cc?$")

  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.+_android.cc?$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/.+_android.cc?$")

  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/audio_device/android/.*$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/audio_device/android/.*$")

  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/video_render/windows/.*$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/video_render/windows/.*$")

  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/video_capture/windows/.*$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/video_capture/windows/.*$")

  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/desktop_capture/win/.*$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/desktop_capture/win/.*$")

  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/desktop_capture/mac/.*$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/desktop_capture/mac/.*$")

  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/video_render/android/.*$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/video_render/android/.*$")

  if(src MATCHES "mac[^/]*\\.cc?$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "mac[^/]*\\.cc?$")
  if(src MATCHES "win32[^/]*\\.cc?$")
    list(REMOVE_ITEM WEBRTC_SRCS "${src}")
  endif(src MATCHES "win32[^/]*\\.cc?$")
endforeach(src)
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/system_wrappers/source/logcat_trace_context.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/system_wrappers/source/data_log_no_op.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/video_coding/codecs/vp8/vp8_sequence_coder.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/audio_coding/main/acm2/acm_send_test_oldapi.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/audio_coding/main/acm2/acm_receive_test_oldapi.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/audio_coding/main/acm2/audio_coding_module_unittest_oldapi.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/audio_coding/main/acm2/acm_receiver_unittest_oldapi.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/desktop_capture/window_capturer_null.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/desktop_capture/screen_capturer_null.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/desktop_capture/mouse_cursor_monitor_null.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/remote_bitrate_estimator/bwe_simulations.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/video_render/video_render_internal_impl.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/video_capture/external/device_info_external.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/modules/video_capture/external/video_capture_external.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/common_audio/real_fourier.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/common_audio/lapped_transform.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/video/full_stack.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/video/replay.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/video/loopback.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/base/winping.cc")
List(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/base/winfirewall.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/base/diskcache_win32.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/base/schanneladapter.cc")
list(REMOVE_ITEM WEBRTC_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/webrtc/base/unittest_main.cc")

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
add_library(webrtc ${WEBRTC_SRCS})
set_target_properties(webrtc PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(webrtc ${EXPAT_LIBRARIES} opus yuv jsoncpp ${CMAKE_THREAD_LIBS_INIT} vpx dl X11 Xext Xrender Xfixes Xcomposite Xdamage ${OPENSSL_LIBRARIES})



file(GLOB_RECURSE JINGLE_SRCS "src/talk/app/**.cc" "src/talk/session/**.cc" "src/talk/media/**.cc")
foreach(src ${JINGLE_SRCS})
  if(src MATCHES "_unittests?.cc?$")
    list(REMOVE_ITEM JINGLE_SRCS "${src}")
  endif(src MATCHES "_unittests?.cc?$")
  if(src MATCHES "_jni.cc?$")
    list(REMOVE_ITEM JINGLE_SRCS "${src}")
  endif(src MATCHES "_jni.cc?$")
  if(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/talk/app/.+/test/.+$")
    list(REMOVE_ITEM JINGLE_SRCS "${src}")
  endif(src MATCHES "^${CMAKE_CURRENT_SOURCE_DIR}/src/talk/app/.+/test/.+$")
  if(src MATCHES "mac[^/]*\\.cc?$")
    list(REMOVE_ITEM JINGLE_SRCS "${src}")
  endif(src MATCHES "mac[^/]*\\.cc?$")
  if(src MATCHES "win32[^/]*\\.cc?$")
    list(REMOVE_ITEM JINGLE_SRCS "${src}")
  endif(src MATCHES "win32[^/]*\\.cc?$")
endforeach(src)
list(REMOVE_ITEM JINGLE_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/talk/media/devices/carbonvideorenderer.cc")
list(REMOVE_ITEM JINGLE_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/talk/media/devices/dummydevicemanager.cc")
list(REMOVE_ITEM JINGLE_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/talk/media/devices/mobiledevicemanager.cc")
list(REMOVE_ITEM JINGLE_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/talk/media/webrtc/dummyinstantiation.cc")
list(REMOVE_ITEM JINGLE_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/talk/media/other/linphonemediaengine.cc")
list(REMOVE_ITEM JINGLE_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/talk/media/base/testutils.cc")

add_definitions(-DLINUX)
add_definitions(-DUSE_WEBRTC_DEV_BRANCH)
add_definitions(-DHAVE_WEBRTC_VIDEO)
add_definitions(-DHAVE_WEBRTC_VOICE)
add_definitions(-DHAVE_SRTP)
add_definitions(-DSRTP_RELATIVE_PATH)
add_definitions(-DUSE_SYSTEM_LIBSRTP)
add_definitions(-DLIBPEERCONNECTION_LIB)
add_library(jingle ${JINGLE_SRCS})
target_link_libraries(jingle webrtc usrsctp srtp gthread-2.0 gobject-2.0 rt)





add_executable(peerconnection_server
  "${CMAKE_CURRENT_SOURCE_DIR}/src/talk/examples/peerconnection/server/main.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/talk/examples/peerconnection/server/data_socket.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/talk/examples/peerconnection/server/peer_channel.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/talk/examples/peerconnection/server/utils.cc"
)
target_link_libraries(peerconnection_server jingle)

add_executable(peerconnection_client
  "${CMAKE_CURRENT_SOURCE_DIR}/src/talk/examples/peerconnection/client/linux/main.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/talk/examples/peerconnection/client/conductor.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/talk/examples/peerconnection/client/defaults.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/talk/examples/peerconnection/client/linux/main_wnd.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/talk/examples/peerconnection/client/peer_connection_client.cc"
)
target_link_libraries(peerconnection_client jingle ${GTK2_LIBRARIES})



#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(FILES ${JACO_COMM_LIB} ${JACO_USB_LIB}
#  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

## Copy header files
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
#)