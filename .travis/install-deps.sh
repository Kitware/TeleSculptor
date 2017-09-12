#!/bin/sh
set -e

INSTALL_DIR=$HOME/deps
KITWARE_DIR=/opt/kitware
export PATH=$INSTALL_DIR/bin:$KITWARE_DIR/fletch/bin:$KITWARE_DIR/kwiver/bin:$PATH
HASH_DIR=/opt/kitware/hashes
mkdir -p $HASH_DIR
mkdir -p $INSTALL_DIR

# Make a directory to test installation of MAP-Tk into
mkdir -p $HOME/install

# check if directory is cached
if [ ! -f "$INSTALL_DIR/bin/cmake" ]; then
  cd /tmp
  wget --no-check-certificate https://cmake.org/files/v3.4/cmake-3.4.0-Linux-x86_64.sh
  bash cmake-3.4.0-Linux-x86_64.sh --skip-license --prefix="$INSTALL_DIR/"
else
  echo 'Using cached CMake directory.';
fi



# download and unpack Fletch
cd /tmp
if [ -f $TRAVIS_BUILD_DIR/doc/release-notes/master.txt ]; then
  FLETCH_TAR_FILE_ID=599c39468d777f7d33e9cbe5
  kwiver_branch="master"
  echo "Using master branches of Fletch and KWIVER"
else
  FLETCH_TAR_FILE_ID=599f2db18d777f7d33e9cc9e
  kwiver_branch="release"
  echo "Using release branches of Fletch and KWIVER"
fi

wget https://data.kitware.com/api/v1/file/$FLETCH_TAR_FILE_ID/hashsum_file/sha512 -O fletch.sha512
RHASH=`cat fletch.sha512`
HASH_FILE="$HASH_DIR/fletch.sha512"
echo "Current Fletch tarball hash: " $RHASH
if [ -f $HASH_FILE ] && [ -n "$RHASH" ] && grep -q $RHASH $HASH_FILE ; then
  echo "Using cached Fletch download"
else
  wget https://data.kitware.com/api/v1/file/$FLETCH_TAR_FILE_ID/download -O fletch.tgz
  rm -rf $KITWARE_DIR/fletch/*
  tar -xzf fletch.tgz -C $KITWARE_DIR
  cp fletch.sha512 $HASH_FILE
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
  if [ -f $HASH_FILE ] && [ -n "$RHASH" ] && grep -q $RHASH $HASH_FILE ; then
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
    git clone $URL $NAME/source
    mkdir $NAME/build
    cd $NAME/source
    # checkout branch after clone in case $BRANCH is actually a hash
    git checkout $BRANCH
    cd ../build
    cmake ../source \
          -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/ \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_CXX_COMPILER=$CXX_COMPILER \
          -DCMAKE_C_COMPILER=$C_COMPILER \
          $CMAKE_OPTS
    make -j2
    $INSTALL_CMD
    # update the Git hash file in the cache for next time
    cd ../source
    RHASH=`git rev-parse HEAD`
    echo $RHASH > $HASH_FILE
  fi
}


# Build and install KWIVER, minimum need to build MAP-Tk
kwiver_cmake_opts="\
 -DKWIVER_ENABLE_ARROWS=ON \
 -Dfletch_DIR=$KITWARE_DIR/fletch"
build_repo kwiver https://github.com/Kitware/kwiver.git

# Build and install QtExtensions
build_repo qtextensions https://github.com/Kitware/qtextensions.git
