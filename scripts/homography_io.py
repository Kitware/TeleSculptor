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

"""This module contains functions to load and save homography files
"""

import numpy as np
import glob



def parse_homography(fin):
    """Parse a single homography from the file object.

    Returns a homography matrix as a numpy.matrix
    """
    H1 = map(float, fin.next().split())
    H2 = map(float, fin.next().split())
    H3 = map(float, fin.next().split())
    H = np.matrix([H1, H2, H3])
    return H


def load_homography_file(filename):
    """Loads an ASCII homography file.

    Returns a list of (metadata, H) tuples.
        metadata is the homography metadata line
        H is a 3x3 homography as a numpy.matrix
    """
    with open(filename, 'r') as f:
        homogs = []
        for line in f:
            line = line.strip()
            if line == "":
                continue
            homogs.append((line, parse_homography(f)))
        return homogs


def write_homography(md, H, fout):
    """Write a single homography in ASCII KRTD format to the file object.
    """
    if md:
        fout.write("%s\n", md)
    fout.write("%.12g %.12g %.12g\n" % tuple(H.tolist()[0]))
    fout.write("%.12g %.12g %.12g\n" % tuple(H.tolist()[1]))
    fout.write("%.12g %.12g %.12g\n\n" % tuple(H.tolist()[2]))


def write_homography_file(homogs, filename):
    """Write a homography file
    """
    with open(filename,'w') as f:
        for md, H in homogs:
            write_homography(md, H, f)
