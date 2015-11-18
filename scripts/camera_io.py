#ckwg +28
# Copyright 2015 by Kitware, Inc. All Rights Reserved. Please refer to
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
#  * Neither name of Kitware, Inc. nor the names of any contributors may be used
#    to endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""This module contains functions to load and save camera files.
"""

import numpy as np
import glob
import os



def parse_camera_krtd(fin):
    """Parse a single camera in KRT format from the file object.

    Returns a (K,R,t,d) tuple using numpy.matrix types where:

        K is a 3x3 calibration matrix
        R is a 3x3 rotation matrix
        t is a 1x3 translation vector (transposed).
        d is a tuple of distortion parameters
    """
    K1 = map(float, fin.next().split())
    K2 = map(float, fin.next().split())
    K3 = map(float, fin.next().split())
    K = np.matrix([K1, K2, K3])
    fin.next()
    R1 = map(float, fin.next().split())
    R2 = map(float, fin.next().split())
    R3 = map(float, fin.next().split())
    R = np.matrix([R1, R2, R3])
    fin.next()
    t = np.matrix(map(float, fin.next().split()))
    fin.next()
    d = map(float, fin.next().split())
    return (K, R, t.transpose(), d)


def load_camera_krtd_file(filename):
    """Loads an ASCII camera file in KRTD format.

    Returns a dictionary mapping file names to (K, R, t, d) tuples.
        K is a 3x3 calibration matrix
        R is a 3x3 rotation matrix
        t is a 1x3 translation vector (transposed).
        d is a tuple of distortion parameters
    """
    with open(filename, 'r') as f:
        return parse_camera_krtd(f)


def load_camera_krtd_glob(fileglob):
    """Loads a set of KRTD camera files specified by a file glob

    Returns a dictionary mapping file names to cameras (K, R, t, d)
    """
    cameras = dict()
    files = sorted(glob.glob(fileglob))
    for filename in files:
        cameras[filename] = load_camera_krtd_file(filename)
    return cameras


def write_camera_krtd(camera, fout):
    """Write a single camera in ASCII KRTD format to the file object.
    """
    K, R, t, d = camera
    t = np.reshape(np.array(t), 3)
    fout.write('%.12g %.12g %.12g\n' % tuple(K.tolist()[0]))
    fout.write('%.12g %.12g %.12g\n' % tuple(K.tolist()[1]))
    fout.write('%.12g %.12g %.12g\n\n' % tuple(K.tolist()[2]))
    fout.write('%.12g %.12g %.12g\n' % tuple(R.tolist()[0]))
    fout.write('%.12g %.12g %.12g\n' % tuple(R.tolist()[1]))
    fout.write('%.12g %.12g %.12g\n\n' % tuple(R.tolist()[2]))
    fout.write('%.12g %.12g %.12g\n\n' % tuple(t.tolist()))
    for v in d:
        fout.write('%.12g ' % v)


def write_camera_krtd_file(camera, filename):
    """Write a camera to a krtd file
    """
    with open(filename,'w') as f:
        write_camera_krtd(camera, f)


def write_camera_krtd_files(camera_map, dirname):
    """Write a map from image filenames to cameras into KRTD files in dirname

    For each (filename, camera) pair
      - get the file basename and append to dirname
      - strip the extenstion and add ".krtd"
      - write the camera to the file
    """
    for fname, camera in sorted(camera_map.iteritems()):
        out_file = os.path.splitext(os.path.basename(fname))[0] + '.krtd'
        out_file = os.path.join(dirname, out_file)
        print out_file
        write_camera_krtd_file(camera, out_file)
