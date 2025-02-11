# syntax=docker/dockerfile:1.7.0-labs
FROM ros:noetic-ros-core AS build_environment

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    software-properties-common \
    build-essential \
    wget && \
    rm -rf /var/lib/apt/lists/*

RUN add-apt-repository ppa:ubuntu-toolchain-r/test

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git \
    libprotobuf-dev \
    protobuf-compiler \
    libopencv-dev \
    default-jdk \
    gcc-11 \
    g++-11 \
    ros-noetic-sparse-bundle-adjustment \
    ros-noetic-map-server \
    ros-noetic-laser-filters && \ 
    rm -rf /var/lib/apt/lists/*

RUN wget https://github.com/Kitware/CMake/releases/download/v3.24.1/cmake-3.24.1-Linux-$(uname -m).sh \
      -q -O /tmp/cmake-install.sh \
      && chmod u+x /tmp/cmake-install.sh \
      && mkdir /opt/cmake-3.24.1 \
      && /tmp/cmake-install.sh --skip-license --prefix=/opt/cmake-3.24.1 \
      && rm /tmp/cmake-install.sh \
      && ln -s /opt/cmake-3.24.1/bin/* /usr/local/bin

ENV CC=gcc-11
ENV CXX=g++-11

FROM build_environment AS wpilib_build

RUN mkdir /wpilib
WORKDIR /wpilib

ADD https://github.com/wpilibsuite/allwpilib.git#v2025.3.1 /wpilib/allwpilib
RUN mkdir build/
RUN --mount=type=cache,target=build/ \
    cd build && \
    cmake ../allwpilib -DWITH_JAVA=OFF -DWITH_JAVA_SOURCE=OFF -DWITH_SIMULATION_MODULES=OFF -DWITH_WPILIB=OFF -DWITH_WPIMATH=OFF -DWITH_GUI=OFF -DWITH_WPIUNITS=OFF -DWITH_EXAMPLES=OFF -DWITH_TESTS=OFF && \
    cmake --build . --parallel 10 && \
    cmake --build . --target install

FROM build_environment

WORKDIR /ros_ws
COPY --parents ros_ws/src/*/package.xml /

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-catkin-tools &&\
    rm -rf /var/lib/apt/lists/*

RUN rosdep init
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/*

COPY --from=wpilib_build /usr/local/include/ /usr/local/include/
COPY --from=wpilib_build /usr/local/lib/ /usr/local/lib/
COPY --from=wpilib_build /usr/local/share/ /usr/local/share/

COPY ros_ws/src /ros_ws/src

RUN --mount=type=cache,target=build \
    catkin config --extend /opt/ros/$ROS_DISTRO && \
    catkin build

RUN sed --in-place --expression \
    '$isource "/ros_ws/devel/setup.bash"' \
    /ros_entrypoint.sh
