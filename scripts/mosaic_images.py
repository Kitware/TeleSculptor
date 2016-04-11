#!/usr/bin/env python
#ckwg +28
# Copyright 2013-2016 by Kitware, Inc. All Rights Reserved. Please refer to
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
This script creates a mosaic image from a sequence of images and homographies.
"""

import sys
from optparse import OptionParser
import cv2
import numpy as np
import math

import homography_io
from homography_extents import compute_extents


def main():
    usage = "usage: %prog [options] image_list_file homography_file mosaic_file"
    description = "warp all of the images by homography and overlay into a mosaic"
    parser = OptionParser(usage=usage, description=description)

    (options, args) = parser.parse_args()

    image_filename = args[0]
    homog_filename = args[1]
    mosaic_filename = args[2]

    with open(image_filename, 'r') as f:
        image_files = [line.rstrip() for line in f]

    homogs = homography_io.load_homography_file(homog_filename)

    if len(homogs) != len(image_files):
        sys.exit("Number of homographies ("+str(len(homogs))+
                 ") does not match number of images ("+
                 str(len(image_files))+")")

    images = [cv2.imread(fn) for fn in image_files]
    shapes = [img.shape[:2] for img in images]

    x_rng, y_rng = compute_extents([H for _, H in homogs], shapes)
    H_offset = np.matrix([[1, 0, -x_rng[0]], [0, 1, -y_rng[0]], [0, 0, 1]])
    mosaic_shape = (int(math.ceil(x_rng[1] - x_rng[0])),
                    int(math.ceil(y_rng[1] - y_rng[0])))

    mosaic = np.zeros(mosaic_shape[::-1] + (4,), images[0].dtype)
    for fname, img, (_, H) in zip(image_files, images, homogs):
        H = H_offset * H
        imgt = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)
        warped = cv2.warpPerspective(imgt, H, mosaic_shape)
        mask = np.nonzero(warped[:,:,3])
        mosaic[mask] = warped[mask]
    cv2.imwrite(mosaic_filename, mosaic)



if __name__ == "__main__":
    main()
