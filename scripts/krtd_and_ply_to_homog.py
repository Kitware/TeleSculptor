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
This script is used to read KRTD cameras files and a PLY file and produce a
homography file.  It fits a plane to the landmarks in the PLY and then
extracts the sequence of homographies induced by that plane (assuming
no radial distortion).
"""

from optparse import OptionParser
import numpy as np
import numpy.linalg as npla
import os.path

from camera_io import *
from landmark_io import *


def homography_from_plane(camera1, camera2, plane):
    """Computes the homography induced by a plane between images of
    camera1 and camera2.  The resulting homography maps a point from
    camera1 to camera2 via the specified plane.
    """
    K1, R1, t1, _ = camera1
    K2, R2, t2, _ = camera2
    R3 = R2 * R1.T
    t3 = t2 - R2 * R1.T * t1

    n = np.matrix(plane[:3]).T
    d = float(plane[3])

    n = R1 * n
    d -= float(t1.T * n)

    return K2 * (R3 - t3 * n.T / d) * K1.I


def estimate_plane(points):
    """Estimate a plane and fit a set of 3D point
    """
    C = np.mean(points, 0)
    N = npla.svd(np.cov(points.T))[2][2]
    d = np.dot(N,C)
    return np.hstack((N,-d))


def write_homog_file(filename, homogs):
    """Write a homography file
    """
    with open(filename, 'w') as fin:
        for i, H in enumerate(homogs):
            fin.write("%d %d\n" % (i,i))
            fin.write("%.12g %.12g %.12g\n" % tuple(H.tolist()[0]))
            fin.write("%.12g %.12g %.12g\n" % tuple(H.tolist()[1]))
            fin.write("%.12g %.12g %.12g\n\n" % tuple(H.tolist()[2]))


def main():
    usage = "usage: %prog [options] ply_file krtd_glob out_homogs"
    description = "Read a PLY and set of KRTDs and produce a homography file"
    parser = OptionParser(usage=usage, description=description)

    (options, args) = parser.parse_args()

    ply_filename = args[0]
    krtd_glob = args[1]
    out_homog_file = args[2]

    L , _ = load_landmark_ply_file(ply_filename)
    C = load_camera_krtd_glob(krtd_glob)

    plane = estimate_plane(L)

    cams = sorted(C.iteritems())
    cam0 = cams[0][1]
    homogs = []
    for f, cam in cams:
        print f
        H = homography_from_plane(cam, cam0, plane)
        homogs.append(H)

    write_homog_file(out_homog_file, homogs)




if __name__ == "__main__":
    main()
