#!/usr/bin/env bash

export PATH=`pwd`/depot_tools:"$PATH"
ninja -j 1 -C $1 peerconnection_client
