#!/usr/bin/env bash

svn co http://webrtc.googlecode.com/svn/branches/40 webrtc_src
svn co https://libyuv.googlecode.com/svn/branches/m39 yuv_src
if [ -d opus_src ]; then (cd opus_src && git pull); else git clone https://chromium.googlesource.com/chromium/deps/opus.git opus_src;fi
svn co http://sctp-refimpl.googlecode.com/svn/trunk/KERN/usrsctp usrsctp_src
if [ -d chromium_src/build ]; then (cd chromium_src/build && git pull); else (mkdir -p chromium_src/build && git clone https://chromium.googlesource.com/chromium/src/build chromium_src/build);fi
