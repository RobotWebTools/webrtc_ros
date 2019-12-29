#!/usr/bin/env bash

docker build -f released -t webrtc_ros/released:latest .
docker build -f develop -t webrtc_ros/develop:latest .
docker build -f dev_awsc14 -t webrtc_ros/dev_awsc14:latest .
