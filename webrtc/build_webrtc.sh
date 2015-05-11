#!/usr/bin/env bash

export PATH=`pwd`/depot_tools:"$PATH"
ninja -C $1 peerconnection_client
