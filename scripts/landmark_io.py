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

"""This module contains functions to load and save landmark files.
"""

import numpy as np
import glob



def parse_ply_header(fin):
    """Parse the PLY format header

    Retuns a dictionary of keys to numpy arrays
    """
    types = {"char"   : np.dtype("int8"),
             "uchar"  : np.dtype("uint8"),
             "short"  : np.dtype("int16"),
             "ushort" : np.dtype("uint16"),
             "int"    : np.dtype("int32"),
             "int32"  : np.dtype("int32"),
             "uint"   : np.dtype("uint32"),
             "float"  : np.dtype("float32"),
             "double" : np.dtype("float64")}

    if not fin.next().startswith("ply\n"):
        raise IOError("does not appear to be a PLY file")
    if not fin.next().startswith("format ascii"):
        raise IOError("PLY file does not appear to be ASCII")
    # ignore comments
    tokens = fin.next().split()
    while tokens[0] == "comment":
        tokens = fin.next().split()
    if not (tokens[0] == "element" and tokens[1] == "vertex"):
        raise IOError("PLY file does not contain only vertices")
    num_pts = int(tokens[2])
    line = fin.next()
    attr_key = []
    attr_type = []
    while line.startswith("property"):
        dtype, key = line.split()[1:3]
        attr_key.append(key)
        attr_type.append(types[dtype])
        line = fin.next()
    while not line.startswith("end_header"):
        line = fin.next()

    return num_pts, attr_key, attr_type


def parse_ply(fin):
    """Parse vertex data from a PLY format

    Retuns a dictionary of keys to numpy arrays
    """
    num_pts, attr_key, attr_type = parse_ply_header(fin)

    data = [[] for k in attr_key]
    for i in range(num_pts):
        line = fin.next()
        tokens = line.split()
        for j, t in enumerate(tokens):
            data[j].append(np.array(t, dtype=attr_type[j]))

    npdata = {}
    for i, k in enumerate(attr_key):
      npdata[k] = np.array(data[i], dtype=attr_type[i])
    return npdata


def load_landmark_ply_file(filename):
    """Load landmarks from a PLY file.

    Retuns a list of numpy 3D vectors
    """
    with open(filename, 'r') as fin:
        data = parse_ply(fin)
        if not 'x' in data or not 'y' in data or not 'z' in data:
            raise IOError("PLY vertices do not contain x, y, and z values")
        landmarks = np.vstack([data["x"], data["y"], data["z"]]).T
        return landmarks, data


def write_ply_header(data, fout, fields):
    """Write out the PLY header
    """
    types = {np.dtype("int8")    : "char",
             np.dtype("uint8")   : "uchar",
             np.dtype("int16")   : "short",
             np.dtype("uint16")  : "ushort",
             np.dtype("int32")   : "int",
             np.dtype("uint32")  : "uint",
             np.dtype("float32") : "float",
             np.dtype("float64") : "double"}

    num_pts = len(data["x"])
    fout.write("ply\nformat ascii 1.0\n")
    fout.write("element vertex %d\n" % num_pts)
    for f in fields:
        fout.write("property %s %s\n" % (types[data[f].dtype], f))
    fout.write("end_header\n")


def write_landmarks_ply(data, fout, fields=None):
    """Write a landmark points as vertices in an ASCII PLY format.
    """
    if not fields:
        fields = data.keys()
    write_ply_header(data, fout, fields)
    fmt_map = {np.dtype("int8")    : "%d",
               np.dtype("uint8")   : "%d",
               np.dtype("int16")   : "%d",
               np.dtype("uint16")  : "%d",
               np.dtype("int32")   : "%d",
               np.dtype("uint32")  : "%d",
               np.dtype("float32") : "%.8g",
               np.dtype("float64") : "%.12g"}
    fmt_str = " ".join([fmt_map[data[f].dtype] for f in fields]) + "\n"

    for d in zip(*[data[f].tolist() for f in fields]):
        fout.write(fmt_str % d)


def write_landmarks_ply_file(landmarks, filename, other_data={}, fields=None):
    """Open a PLY file and write landmarks as vertices.

    landmarks is a Nx3 array of (x,y,z) coordinates
    filename is the path the file to write
    other_data is an optional dictionary of other data to write
    fields is a list of properties to write in a specific order

    The other_data keys are PLY property names (other than x, y, z) that map
    to a numpy array of values for each landmark for that property.  The arrays
    should each by of length N.

    The fields list must be either None or a subset of properties "x", "y", "z",
    and the keys of other_data.  The order of the properties is the order they appear
    in the file.
    """
    L = np.array(landmarks)
    other_data["x"] = L[:,0]
    other_data["y"] = L[:,1]
    other_data["z"] = L[:,2]
    if not fields:
        fields = ["x", "y", "z"]
    with open(filename,'w') as f:
        write_landmarks_ply(other_data, f, fields)
