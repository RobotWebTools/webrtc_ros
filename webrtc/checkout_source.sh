#!/usr/bin/env bash

# get depot_tools if we don't have it, it should auto update itself
if [ ! -d depot_tools ]; then git clone https://chromium.googlesource.com/chromium/tools/depot_tools.git; fi

(
    # load depot_tools into the enviornment
    export PATH=`pwd`/depot_tools:"$PATH"

    # Don't use clang (use gcc)
    # fastbuld=2 disables debug symbols and makes build faster
    export GYP_DEFINES="clang=0 fastbuild=2"

    if [ -z "$1" ]; then
	echo "No build directory specified, generating build files in source tree"
    else
	echo "Generating build files in: $1"
	export GYP_GENERATOR_OUTPUT="$1"
    fi

    WEBRTC_REVISION="src@70dfed74a3141b13849dcd19321523140b3e614b" # Chrome 41
    gclient sync --no-history --with_branch_heads --nohooks --revision $WEBRTC_REVISION

    ./sync_chromium.py

    python src/setup_links.py

    python src/webrtc/build/gyp_webrtc -I../build.gypi
)
