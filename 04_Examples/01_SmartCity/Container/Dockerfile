FROM debian:buster-slim

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/London

# Install the essential packages
RUN mkdir -p /usr/share/man/man1
RUN apt-get update -y && apt-get install -y git autoconf build-essential \
    cmake pkg-config yasm git gfortran \
    libjpeg-dev libpng-dev libsdl-image1.2-dev libsdl-net1.2-dev libtiff5-dev \
    libtiff-dev libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev libxine2-dev \
    libgtk2.0-dev libtbb-dev qt5-default libatlas-base-dev python3-numpy \
    libmp3lame-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev \
    libopencore-amrnb-dev libopencore-amrwb-dev libavresample-dev x264 v4l-utils \
    libprotobuf-dev protobuf-compiler libgoogle-glog-dev libgflags-dev \
    libgphoto2-dev libeigen3-dev libhdf5-dev doxygen libconfig++-dev libyaml-cpp-dev \
    python3-dev python3-dev python3-pip python3-testresources vim qv4l2 \
    feh libconfig++-dev libgtk2.0-dev pkg-config libssl-dev

RUN cd /usr/include/linux && ln -s -f ../libv4l1-videodev.h videodev.h

RUN git clone https://github.com/samir-souza/powergine && \
    cd powergine && mkdir build && cd build && cmake .. && \
    make -j$((`nproc`-1)) && make install && \
    cd ~ && rm -rf /powergine

RUN git clone https://github.com/AprilRobotics/apriltag.git && \
    cd apriltag && git checkout tags/3.1.1 && \
    sed -i 's#    td->nthreads = 1;#    td->nthreads = 2;#' apriltag.c && \
    sed -i 's#    td->quad_decimate = 2.0;#    td->quad_decimate = 0.9;#' apriltag.c && \
    mkdir build && cd build && cmake .. && \
    make -j$((`nproc`-1)) && make install && \
    cd ~ && rm -rf /apriltag

RUN git clone https://github.com/opencv/opencv.git && \
    cd opencv && git checkout tags/4.1.1 && cd .. && \
    git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv_contrib && git checkout tags/4.1.1 && cd .. && \
    sed -i 's#dl m pthread rt#dl m pthread rt atomic#' opencv/CMakeLists.txt

# If on Raspberry, change: ENABLE_NEON=ON WITH_V4L=ON ENABLE_VFPV3=ON
RUN mkdir opencv/build && cd opencv/build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \ 
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
        -D WITH_TBB=ON \
        -D ENABLE_NEON=OFF \
        -D WITH_LIBV4L=OFF \
        -D WITH_V4L=OFF \
        -D WITH_OPENGL=ON \
        -D WITH_QT=ON \
        -D INSTALL_C_EXAMPLES=OFF \
        -D ENABLE_PRECOMPILED_HEADERS=OFF \
        -D ENABLE_VFPV3=OFF \
        -D BUILD_TESTS=OFF \
        -D OPENCV_ENABLE_NONFREE=ON \
        -D INSTALL_PYTHON_EXAMPLES=OFF \
        -D PYTHON_DEFAULT_EXECUTABLE=/usr/bin/python3 \
        -D BUILD_EXAMPLES=OFF .. && \
    make -j$((`nproc`-1)) && make install
RUN rm -rf opencv opencv_contrib

RUN git clone https://github.com/samir-souza/minibots.git && \
    cd minibots/04_Examples/01_SmartCity/Backend && mkdir build && cd build && cmake .. && \
    make -j$((`nproc`-1)) && \
    ln -s ../assets/bot_config . && ln -s ../assets/config.yaml . && ./trackBuilder -i ../assets/track_test.png

RUN rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/bin/bash"]

