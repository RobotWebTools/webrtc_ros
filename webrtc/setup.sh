#!/usr/bin/env bash

git clone https://chromium.googlesource.com/chromium/tools/depot_tools.git
export PATH=`pwd`/depot_tools:"$PATH"
gclient config --name src http://webrtc.googlecode.com/svn/trunk
gclient sync --force
src/build/install-build-deps.sh --syms
