#!/bin/sh
set -e

export PATH=$HOME/deps/bin:$PATH
HASH_DIR=$HOME/deps/hashes
mkdir -p $HASH_DIR

# check if directory is cached
if [ ! -f "$HOME/deps/bin/cmake" ]; then
  cd /tmp
  wget --no-check-certificate https://cmake.org/files/v3.4/cmake-3.4.0-Linux-x86_64.sh
  bash cmake-3.4.0-Linux-x86_64.sh --skip-license --prefix="$HOME/deps/"
else
  echo 'Using cached CMake directory.';
fi

FLETCH_URL=https://github.com/Kitware/fletch.git
FLETCH_RHASH=`git ls-remote -h $FLETCH_URL master | cut -f1`
FLETCH_HASH_FILE=$HASH_DIR/fletch.sha
echo "Current Fletch Hash: $FLETCH_RHASH"
if [ -f $FLETCH_HASH_FILE ] && grep -q $FLETCH_RHASH $FLETCH_HASH_FILE ; then
  echo "Using cached Fletch build: `cat $FLETCH_HASH_FILE`"
else
  if [ -f $FLETCH_HASH_FILE ]; then
    echo "Cached Fletch build is not current: `cat $FLETCH_HASH_FILE`"
  else
    echo "No cached Fletch build"
  fi
  cd /tmp
  git clone https://github.com/Kitware/fletch.git fletch/source
  mkdir fletch/build
  cd fletch/build
  cmake ../source \
        -DCMAKE_BUILD_TYPE=Release \
        -Dfletch_ENABLE_Eigen=ON \
        -Dfletch_ENABLE_Ceres=ON \
        -Dfletch_ENABLE_SuiteSparse=ON \
        -Dfletch_ENABLE_OpenCV=ON \
        -Dfletch_ENABLE_VTK=ON
  make -j2
  cp -r install/* $HOME/deps/
  echo $FLETCH_RHASH > $FLETCH_HASH_FILE
fi


VITAL_URL=https://github.com/Kitware/vital.git
VITAL_RHASH=`git ls-remote -h $VITAL_URL master | cut -f1`
VITAL_HASH_FILE=$HASH_DIR/vital.sha
echo "Current Vital Hash: $VITAL_RHASH"
if [ -f $VITAL_HASH_FILE ] && grep -q $VITAL_RHASH $VITAL_HASH_FILE; then
  echo "Using cached Vital build: `cat $VITAL_HASH_FILE`"
else
  if [ -f $VITAL_HASH_FILE ]; then
    echo "Cached Vital build is not current: `cat $VITAL_HASH_FILE`"
  else
    echo "No cached Vital build"
  fi
  cd /tmp
  git clone $VITAL_URL vital/source
  mkdir vital/build
  cd vital/build
  cmake ../source \
        -DCMAKE_INSTALL_PREFIX=$HOME/deps/ \
        -DCMAKE_BUILD_TYPE=Release \
        -DVITAL_ENABLE_C_LIB=ON
  make -j2
  make install
  echo $VITAL_RHASH > $VITAL_HASH_FILE
fi


QTE_URL=https://github.com/Kitware/qtextensions.git
QTE_RHASH=`git ls-remote -h $QTE_URL master | cut -f1`
QTE_HASH_FILE=$HASH_DIR/qtextensions.sha
echo "Current QtExtensions Hash: $QTE_RHASH"
if [ -f $QTE_HASH_FILE ] && grep -q $QTE_RHASH $QTE_HASH_FILE; then
  echo "Using cached QtExtenions build: `cat $QTE_HASH_FILE`"
else
  if [ -f $QTE_HASH_FILE ]; then
    echo "Cached QtExtenions build is not current: `cat $QTE_HASH_FILE`"
  else
    echo "No cached QtExtenions build"
  fi
  cd /tmp
  git clone $QTE_URL qtextensions/source
  mkdir qtextensions/build
  cd qtextensions/build
  cmake ../source \
        -DCMAKE_INSTALL_PREFIX=$HOME/deps/ \
        -DCMAKE_BUILD_TYPE=Release
  make -j2
  make install
  echo $QTE_RHASH > $QTE_HASH_FILE
fi
