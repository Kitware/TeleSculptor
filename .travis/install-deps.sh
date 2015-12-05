#!/bin/sh
set -e

INSTALL_DIR=$HOME/deps
export PATH=$INSTALL_DIR/bin:$PATH
HASH_DIR=$INSTALL_DIR/hashes
mkdir -p $HASH_DIR

# check if directory is cached
if [ ! -f "$INSTALL_DIR/bin/cmake" ]; then
  cd /tmp
  wget --no-check-certificate https://cmake.org/files/v3.4/cmake-3.4.0-Linux-x86_64.sh
  bash cmake-3.4.0-Linux-x86_64.sh --skip-license --prefix="$INSTALL_DIR/"
else
  echo 'Using cached CMake directory.';
fi

# Build and install a repository from source only.
# First check Git hash of code installed in the cache
# and only build if the cache is missing or out of date
#
# usage: build_repo name git_url
#
# Optionally set name_branch before calling to use a
# branch other than "master"
#
# Optionally set name_install_cmd to use an install command
# other than "make install"
#
# Optionally set name_cmake_opts to include additional options
# to pass to CMake
build_repo ()
{
  NAME=$1
  URL=$2
  REF="${NAME}_branch"
  BRANCH=${!REF}
  BRANCH=${BRANCH:-"master"}
  REF="${NAME}_cmake_opts"
  CMAKE_OPTS=${!REF}
  REF="${NAME}_install_cmd"
  INSTALL_CMD=${!REF}
  INSTALL_CMD=${INSTALL_CMD:-"make install"}

  # Get the Git hash on the remote server
  RHASH=`git ls-remote -h $URL $BRANCH | cut -f1`
  # The file containing the hash of the cached build
  HASH_FILE=$HASH_DIR/$NAME.sha
  echo "Current $NAME hash:" $RHASH
  # If we have the hash file and the hash matches
  if [ -f $HASH_FILE ] && grep -q $RHASH $HASH_FILE ; then
    echo "Using cached $NAME build: `cat $HASH_FILE`"
  else
    # For debugging, echo whether the hash is missing or out of date
    if [ -f $HASH_FILE ]; then
      echo "Cached $NAME build is not current: `cat $HASH_FILE`"
    else
      echo "No cached $NAME build"
    fi
    # checkout and build the code in the tmp directory
    cd /tmp
    git clone $URL -b $BRANCH $NAME/source
    mkdir $NAME/build
    cd $NAME/build
    cmake ../source \
          -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/ \
          -DCMAKE_BUILD_TYPE=Release \
          $CMAKE_OPTS
    make -j2
    $INSTALL_CMD
    # update the Git hash file in the cache for next time
    echo $RHASH > $HASH_FILE
  fi
}


# Build and install Fletch
fletch_install_cmd="cp -r install/* $INSTALL_DIR/"
fletch_cmake_opts="\
 -Dfletch_ENABLE_Eigen=ON \
 -Dfletch_ENABLE_Ceres=ON \
 -Dfletch_ENABLE_SuiteSparse=ON \
 -Dfletch_ENABLE_OpenCV=ON \
 -Dfletch_ENABLE_VTK=ON"
build_repo fletch https://github.com/Kitware/fletch.git

# Build and install Vital
vital_cmake_opts="-DVITAL_ENABLE_C_LIB=ON"
build_repo vital https://github.com/Kitware/vital.git

# Build and install VXL
vxl_cmake_opts="\
  -DCMAKE_CXX_STANDARD=11 \
  -DBUILD_RPL=ON \
  -DBUILD_BRL=OFF \
  -DBUILD_MUL=OFF \
  -DBUILD_GEL=OFF \
  -DBUILD_OXL=OFF \
  -DBUILD_OUL=OFF \
  -DBUILD_PRIP=OFF \
  -DBUILD_TBL=OFF"
build_repo vxl https://github.com/vxl/vxl.git

# Build and install QtExtensions
build_repo qtextensions https://github.com/Kitware/qtextensions.git
