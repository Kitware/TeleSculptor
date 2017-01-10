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
import scipy.ndimage.morphology as morph
import math

import homography_io
from homography_extents import compute_extents


def image_file_dims(filename):
    """Return the image size (height, width) without loading all the pixels"""
    from PIL import Image
    img = Image.open(filename)
    return img.size[::-1]


def main():
    usage = "usage: %prog [options] image_list_file homography_file mosaic_file"
    description = "warp all of the images by homography and overlay into a mosaic"
    parser = OptionParser(usage=usage, description=description)

    parser.add_option("-e", "--expand", default=False,
                      action="store_true", dest="expand",
                      help="expand the output image to fit entire mosaic")

    parser.add_option("-b", "--blend", default=False,
                      action="store_true", dest="blend",
                      help="blend the pixels when creating the mosaic")

    parser.add_option("-f", "--frame-width", default=0,
                      type="int", dest="frame",
                      help="draw a frame around each image")

    parser.add_option("-s", "--scale", default=1.0,
                      type="float", dest="scale",
                      help="scale of the output mosaic size")

    parser.add_option("-a", "--all-frame", default=False,
                      action="store_true", dest="all_frame",
                      help="draw all parts of the frames, even when obscured")

    (options, args) = parser.parse_args()

    image_filename = args[0]
    homog_filename = args[1]
    mosaic_filename = args[2]

    with open(image_filename, 'r') as f:
        image_files = [line.rstrip() for line in f]

    homogs = homography_io.load_homography_file(homog_filename)

    sH = np.diag([options.scale, options.scale, 1.0])
    homogs = [(f, sH * H) for f, H in homogs]

    if len(homogs) != len(image_files):
        sys.exit("Number of homographies ("+str(len(homogs))+
                 ") does not match number of images ("+
                 str(len(image_files))+")")

    shapes = [image_file_dims(fn) for fn in image_files]
    img = cv2.imread(image_files[0])
    img_dtype = img.dtype

    if options.expand:
        x_rng, y_rng = compute_extents([H for _, H in homogs], shapes)
        H_offset = np.matrix([[1, 0, -x_rng[0]], [0, 1, -y_rng[0]], [0, 0, 1]])
        mosaic_shape = (int(math.ceil(x_rng[1] - x_rng[0])),
                        int(math.ceil(y_rng[1] - y_rng[0])))
    else:
        H_offset = np.eye(3)
        mosaic_shape = img.shape[1::-1]

    if options.blend:
        out_type = np.int32
    else:
        out_type = img_dtype

    print "creating mosaic of size", mosaic_shape
    mosaic = np.zeros(mosaic_shape[::-1] + (4,), out_type)
    edges = np.zeros(mosaic.shape[:2], np.bool)
    for fname, (_, H) in zip(image_files, homogs):
        print "processing", fname
        H = H_offset * H
        img = cv2.imread(fname)
        imgt = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)
        warped = cv2.warpPerspective(imgt, H, mosaic_shape)
        if options.frame > 0:
            edge_map = warped[:,:,3] == 255
            structure = np.ones((2*options.frame+1,)*2)
            edge_map = np.logical_xor(edge_map, morph.binary_erosion(edge_map, structure))
            if options.all_frame:
                edges = np.logical_or(edges, edge_map)
            else:
                warped[edge_map] = (0,0,255,255)
        mask = np.nonzero(warped[:,:,3] == 255)
        if options.blend:
            mosaic[mask] += warped[mask]
        else:
            mosaic[mask] = warped[mask]
    if options.blend:
        mask = np.nonzero(mosaic[:,:,3] > 255)
        for i in range(4):
            mosaic[:,:,i][mask] /= mosaic[:,:,3][mask] / 255
        mosaic.astype(img_dtype)
    if options.frame > 0 and options.all_frame:
        mosaic[edges] = (0,0,255,255)
    cv2.imwrite(mosaic_filename, mosaic)



if __name__ == "__main__":
    main()
