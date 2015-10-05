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

"""Compute the output extents for a homography sequence.

Assuming all input images of of the size (W,H) as specified, this
script computes a bounding box around all the images in the output
space.
"""

from optparse import OptionParser

import numpy as np

import homography_io


def compute_extents(homogs, img_shape):
    """Compute output extents from a sequence of homographies and an image shape
    """
    h, w = img_shape

    corners = np.matrix([[0, 0, 1],[w, 0, 1],[w, h, 1],[0, h, 1]], dtype='float').T

    min_x = np.inf
    max_x = -np.inf
    min_y = np.inf
    max_y = -np.inf
    for H in homogs:
        mapped = H * corners
        X = mapped[0,:] / mapped[2,:]
        Y = mapped[1,:] / mapped[2,:]
        min_x = min(min_x, np.min(X))
        max_x = max(max_x, np.max(X))
        min_y = min(min_y, np.min(Y))
        max_y = max(max_y, np.max(Y))
    return (min_x, max_x), (min_y, max_y)


def main():
    usage  = "usage: %prog [options] homog_file img_width img_height\n\n"
    usage += "  Compute the output extents of a homography file\n"
    usage += "  in_homog is input homography sequence\n"
    usage += "  img_width, img_height are the width and height of the input\n"
    parser = OptionParser(usage=usage)

    (options, args) = parser.parse_args()

    in_homog = args[0]
    w, h = map(float, args[1:3])

    homogs = homography_io.load_homography_file(in_homog)

    (min_x, max_x), (min_y, max_y) = compute_extents([H for _, H, in homogs], (h, w))

    print "X range:", min_x, max_x
    print "Y range:", min_y, max_y


if __name__ == "__main__":
    main()
