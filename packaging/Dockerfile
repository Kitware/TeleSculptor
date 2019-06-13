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

# Install required system packages to build TeleSculptor
# Everything else is built from Fletch
RUN apt-get update && apt-get install -y --no-install-recommends \
build-essential file git cmake \
libgl1-mesa-dev libxt-dev libexpat1-dev libgtk2.0-dev liblapack-dev \
python3-dev python3-docutils && \
rm -rf /var/lib/apt/lists/*

# Set Python3 as the default
# Make sure `python` is available and calls `python3`
# Some build steps fail without `python` available
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 10

# Clone the source code from Git
RUN git clone https://github.com/Kitware/TeleSculptor.git TeleSculptor/src

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
