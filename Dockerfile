FROM ros:foxy


# ROS dependencies
RUN apt-get update && \
    apt dist-upgrade -y && \
    apt-get install -y --no-install-recommends \
        python-is-python3 \
        libopencv-dev \
        python3-opencv \
        libboost-all-dev \
        openssl \
        git \
        gdb \
        libcurl4-openssl-dev \
        libssl-dev \
        curl \
        ros-foxy-cv-bridge \
        ros-foxy-image-transport \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*


RUN apt-get install -y --no-install-recommends \
    gmodule-2.0 \
    libgtk-3-dev \
    libglib2.0-dev \
    pulseaudio \
    libasound2-dev \
    libpulse-dev \
    ninja-build \
    stow

RUN apt-get install -y --no-install-recommends \
    libjpeg-turbo8 \
    libjpeg-turbo8-dev

WORKDIR /home/3rdparty/jsoncpp/
RUN git clone https://github.com/open-source-parsers/jsoncpp.git . && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_STATIC_LIBS=ON -DBUILD_SHARED_LIBS=OFF -DARCHIVE_INSTALL_DIR=. -G "Unix Makefiles" .. &&  \
    make && \
    make install

ENV LD_LIBRARY_PATH /usr/local/lib/:$LD_LIBRARY_PATH

RUN git clone https://github.com/GT-RAIL/async_web_server_cpp.git /home/webrtc_ws/src/async_web_server_cpp/

RUN /ros_entrypoint.sh catkin_make_isolated --install --install-space "/usr/local/webrtc/" \
    && sed -i '$isource "/usr/local/webrtc/setup.bash"' /ros_entrypoint.sh \
    && rm -rf /home/webrtc_ws/

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD [ "bash" ]
