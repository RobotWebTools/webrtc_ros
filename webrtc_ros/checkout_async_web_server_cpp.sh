#!/usr/bin/env bash

if [ -d async_web_server_cpp_src ]; then (cd async_web_server_cpp_src && git fetch); else (git clone https://github.com/WPI-RAIL/async_web_server_cpp.git async_web_server_cpp_src);fi; (cd async_web_server_cpp_src && git checkout master)
