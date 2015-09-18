#!/usr/bin/env python
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

"""
This script is used to read the NVM file format produced by VisualSFM and
write out KRTD files.
"""

from optparse import OptionParser
import numpy as np
import numpy.linalg as npla
import os.path

from camera_io import *


def quaternion_to_rot_mat(q):
    """Convert a quaternion [WXYZ] to a 3x3 rotation matrix
    """
    q = np.array(q)
    q = q / npla.norm(q)
    x2 = q[1]**2
    xy = q[1]*q[2]
    wx = q[0]*q[1]
    y2 = q[2]**2
    yz = q[2]*q[3]
    wy = q[0]*q[2]
    z2 = q[3]**2
    zx = q[1]*q[3]
    wz = q[0]*q[3]
    w2 = q[0]**2

    return np.matrix([[w2 + x2 - y2 - z2, 2 * (xy - wz), 2 * (zx + wy)],
                     [2 * (xy + wz), w2 - x2 + y2 - z2, 2 * (yz - wx)],
                     [2 * (zx - wy), 2 * (yz + wx), w2 - x2 - y2 + z2]])


def load_nvm_file(filename, img_width, img_height):
    """Loads cameras from an NVM file
    """
    cameras = dict()
    with open(filename, 'r') as fin:
        line = fin.next().strip()
        if line != "NVM_V3":
            print "first line:", line
            raise IOError("does not appear to be an NVM_V3 file")
        line = fin.next().strip()
        while not line:
            line = fin.next().strip()
        num_cameras = int(line)
        for i in range(num_cameras):
            line = fin.next().strip()
            fname, rest = line.split(None,1)
            data = map(float, rest.split())
            foc = data[0]
            q = data[1:5]
            c = data[5:8]
            d = data[8:]
            ppx = img_width / 2.0
            ppy = img_height / 2.0
            K = np.matrix([[foc, 0, ppx], [0, foc, ppy], [0, 0, 1]])
            R = quaternion_to_rot_mat(q)
            t = -R*np.matrix(c).T
            cameras[fname] = [K, R, t, d]
    return cameras


def main():
    usage = "usage: %prog [options] nvm_file [output_file]"
    description = "Read a nvm file and write KRTD"
    parser = OptionParser(usage=usage, description=description)

    parser.add_option("-x", "--width", type='int', default=1920,
                      action="store", dest="width")
    parser.add_option("-y", "--height", type='int', default=1080,
                      action="store", dest="height")

    (options, args) = parser.parse_args()

    nvm_filename = args[0]

    output_dir = None
    if len(args) >= 2:
        output_dir = args[1]
        if not os.path.isdir(output_dir):
            os.mkdir(output_dir)

    cameras = load_nvm_file(nvm_filename, options.width, options.height)

    if output_dir:
        write_camera_krtd_files(cameras, output_dir)




if __name__ == "__main__":
    main()
