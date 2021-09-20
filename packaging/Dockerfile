# The prupose of this Dockerfile is to create a container to build
# the TeleSculptor package on Linux in a clean environment.
# To build the package and extract it from the container, run the following
#
#     $ docker build -t telesculptor . 
#     $ docker create -ti --name tempts telesculptor bash
#     $ docker cp tempts:/TeleSculptor/bld/TeleSculptor-1.0.0-Linux-x86_64.sh .
#     $ docker rm -fv tempts

# Use a base image with CUDA development libraries
FROM nvidia/cuda:9.2-devel-ubuntu18.04

# Specify which TeleSculptor branch to build
ARG branch=release

# Install required system packages to build TeleSculptor
# Everything else is built from Fletch
RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
apt-get install -y --no-install-recommends \
build-essential file git \
wget automake \
libgl1-mesa-dev libxt-dev libexpat1-dev libgtk2.0-dev liblapack-dev \
libx11-xcb-dev libxcb1-dev libxcb-glx0-dev libxkbcommon-x11-dev \
python3-dev python3-sphinx python3-sphinx-rtd-theme && \
rm -rf /var/lib/apt/lists/*

# Install CMake 3.15
RUN wget --no-check-certificate \
https://github.com/Kitware/CMake/releases/download/v3.15.1/cmake-3.15.1-Linux-x86_64.sh \
&& chmod +x cmake-3.15.1-Linux-x86_64.sh \
&& ./cmake-3.15.1-Linux-x86_64.sh --skip-license \
&& rm -rf cmake-3.15.1-Linux-x86_64.sh

# Set Python3 as the default
# Make sure `python` is available and calls `python3`
# Some build steps fail without `python` available
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 10

# patchelf packaged for Ubuntu 18.04 has bugs that interfer with TeleSculptor
# packaging. Build a newer version from source instead.
RUN wget https://github.com/NixOS/patchelf/archive/0.11.tar.gz
RUN tar -xzf 0.11.tar.gz
WORKDIR patchelf-0.11
RUN ./bootstrap.sh
RUN ./configure
RUN make install
WORKDIR ..

# Clone the source code from Git
RUN git clone -b ${branch} https://github.com/Kitware/TeleSculptor.git TeleSculptor/src

# Make a build directory and set it as the working directory
RUN mkdir TeleSculptor/bld
WORKDIR TeleSculptor/bld

# Configure with CMake
# Turn on options for a full-featured build
RUN cmake ../src -DTELESCULPTOR_ENABLE_CUDA=ON \
                 -DTELESCULPTOR_ENABLE_PYTHON=ON \
                 -DTELESCULPTOR_ENABLE_MANUALS=ON

# run the build step with onre more than the number of available processors
RUN LD_LIBRARY_PATH=/TeleSculptor/bld/external/fletch-build/install/lib && \
make -j$((`nproc`+1))
#end of Dockerfile
