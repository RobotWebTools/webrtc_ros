#!/usr/bin/env bash

svn co http://webrtc.googlecode.com/svn/branches/41 webrtc_src && svn revert webrtc_src -R && patch -p0 -i ../fix_dtls_handshake.patch -d webrtc_src && patch -p0 -i ../bio_const.patch -d webrtc_src
svn co https://libyuv.googlecode.com/svn/branches/m39 yuv_src
if [ -d opus_src ]; then (cd opus_src && git fetch); else git clone https://chromium.googlesource.com/chromium/deps/opus.git opus_src;fi; (cd opus_src && git checkout cae6961)
svn co -r 9066 http://sctp-refimpl.googlecode.com/svn/trunk/KERN/usrsctp usrsctp_src
if [ -d chromium_src/build ]; then (cd chromium_src/build && git fetch); else (mkdir -p chromium_src/build && git clone https://chromium.googlesource.com/chromium/src/build chromium_src/build);fi; (cd chromium_src/build && git checkout 877467e)
if [ -d libvpx ]; then (cd libvpx && git pull); else git clone https://chromium.googlesource.com/webm/libvpx libvpx;fi

echo
echo "Source versions:"
echo -n "chromium: "; (cd chromium_src/build && git rev-parse HEAD)
echo -n "usrsctp: "; (cd usrsctp_src && svnversion .)
echo -n "opus: "; (cd opus_src && git rev-parse HEAD)
echo -n "libvpx: "; (cd libvpx && git rev-parse HEAD)
echo -n "yuv: "; (cd yuv_src && svnversion .)
echo -n "webrtc: "; (cd webrtc_src && svnversion .)
