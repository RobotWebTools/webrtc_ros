#!/usr/bin/env bash

svn co http://webrtc.googlecode.com/svn/trunk webrtc_src
svn co https://libyuv.googlecode.com/svn/trunk yuv_src
git clone https://chromium.googlesource.com/chromium/deps/opus.git opus_src
svn co http://sctp-refimpl.googlecode.com/svn/trunk/KERN/usrsctp usrsctp_src
mkdir -p chromium_src/build
git clone https://chromium.googlesource.com/chromium/src/build chromium_src/build
