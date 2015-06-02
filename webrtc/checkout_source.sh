#!/usr/bin/env bash

# get depot_tools if we don't have it, it should auto update itself
if [ ! -d depot_tools ]; then git clone https://chromium.googlesource.com/chromium/tools/depot_tools.git; fi

(
    # load depot_tools into the enviornment
    export PATH=`pwd`/depot_tools:"$PATH"

    # Don't use clang (use gcc)
    # fastbuld=2 disables debug symbols and makes build faster
    # glibcxx_debug flags will not compile with ROS
    export GYP_DEFINES="clang=0 fastbuild=2 use_gnome_keyring=0 use_gio=0 use_gconf=0 use_libpci=0 use_x11=0 use_system_icu=1 disable_glibcxx_debug=1"

    WEBRTC_REVISION="src@0a8a5e5bd1cec4782d1aa30fec5e85dddfc0d2b4" # Chrome 43
    echo "Syncing webrtc"
    gclient sync --no-history --with_branch_heads --with_tags --nohooks --revision $WEBRTC_REVISION

    ./sync_chromium.py

    python src/setup_links.py

    if [ -z "$1" ]; then
	echo "No build directory specified, generating build files in source tree"
    else
	echo "Generating build files in: $1"
	export GYP_GENERATOR_OUTPUT="$1"
    fi

    cd src
    python webrtc/build/gyp_webrtc -I../build.gypi ../bare.gyp all.gyp
)
