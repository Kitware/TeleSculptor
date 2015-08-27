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
The script is for reading match matrix files and displaying them.
"""

from optparse import OptionParser

import matplotlib.pyplot as plt
import numpy as np
import scipy.io as sio
import os


def main():
    usage = "usage: %prog [options] match_matrix_file [saved_plot]"
    description = "Read and display a match matrix file"
    parser = OptionParser(usage=usage, description=description)

    (options, args) = parser.parse_args()

    matrix_filename = args[0]

    ext = os.path.splitext(matrix_filename)[1]
    if ext == ".mtx" or ext == "mtx.gz":
        MM = sio.mmread(matrix_filename).toarray()
    else:
        M = np.loadtxt(matrix_filename)
        MM = M[1:,:]
    print MM.shape
    print MM.dtype
    plt.imshow(MM)

    output_filename = None
    if len(args) >= 2:
        output_filename = args[1]

    if output_filename:
        plt.savefig(output_filename)

    plt.show()


if __name__ == "__main__":
    main()
