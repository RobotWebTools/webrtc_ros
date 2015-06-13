#!/usr/bin/env bash
set -e

export PATH=`pwd`/depot_tools:"$PATH"

if [[ $TRAVIS == "true" ]]; then
    echo "Building on travis"
    TRAVIS_NINJA_ARGS=("-j" "2")
else
    TRAVIS_NINJA_ARGS=()
fi
ninja "${TRAVIS_NINJA_ARGS[@]}" -C $1 bare_executable relayserver stunserver turnserver
