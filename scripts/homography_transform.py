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
from homography_extents import compute_extents


def translate_homog(H, x, y):
    """translate a homography by X, Y in the output space
    """
    return np.matrix([[1, 0, x], [0, 1, y], [0, 0, 1]]) * H


def scale_homog(H, scale):
    """scale a homography by scale in the output space
    """
    return np.matrix([[scale, 0, 0], [0, scale, 0], [0, 0, 1]]) * H


def main():
    usage  = "usage: %prog [options] in_homogs out_homogs\n\n"
    usage += "  Transform a homography file by translating then scaling\n"
    usage += "  in_homog is input homography sequence\n"
    usage += "  out_homog is output homography sequence\n"
    parser = OptionParser(usage=usage)

    parser.add_option("-x", "--x-trans", type='float', default=0.0,
                      action="store", dest="x_trans",
                      help="X translation, or image width if used with -a")
    parser.add_option("-y", "--y-trans", type='float', default=0.0,
                      action="store", dest="y_trans",
                      help="Y translation, or image height if used with -a")
    parser.add_option("-s", "--scale", type='float', default=1.0,
                      action="store", dest="scale",
                      help="scale factor")
    parser.add_option("-a", "--auto-translate", default=False,
                      action="store_true", dest="auto_trans",
                      help="compute the translation from image bounds")

    (options, args) = parser.parse_args()

    in_homog_file = args[0]
    out_homog_file = args[1]

    homogs = homography_io.load_homography_file(in_homog_file)

    x, y = options.x_trans, options.y_trans
    s = options.scale
    if options.auto_trans:
        x_rng, y_rng = compute_extents([H for _, H in homogs], (y,x))
        x = -x_rng[0]
        y = -y_rng[0]
        print "output image size:", s * (x_rng[1] - x_rng[0]), \
                                    s * (y_rng[1] - y_rng[0])

    print "translating output by (%g, %g)" % (x,y)
    print "scaling output by ", s

    trans_homogs = []
    for md, H in homogs:
        H = translate_homog(H, x, y)
        H = scale_homog(H, s)
        trans_homogs.append((md,H))

    homography_io.write_homography_file(trans_homogs, out_homog_file)


if __name__ == "__main__":
    main()
