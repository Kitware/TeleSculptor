"""
ckwg +31
Copyright 2015 by Kitware, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

 * Neither name of Kitware, Inc. nor the names of any contributors may be used
   to endorse or promote products derived from this software without specific
   prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

==============================================================================

Interface to maptk::camera class.

"""
# -*- coding: utf-8 -*-
__author__ = 'purg'

import ctypes

from maptk.util import MaptkObject, MaptkErrorHandle


class Camera (MaptkObject):
    """ maptk::camera interface class """

    @classmethod
    def from_krtd_file(cls, filepath):
        """
        :return: New Camera instance from a KRTD format file
        :rtype: Camera
        """
        cam_read_krtd = cls.MAPTK_LIB.maptk_camera_read_krtd_file
        cam_read_krtd.argtypes = [ctypes.c_char_p, MaptkErrorHandle.C_TYPE_PTR]
        cam_read_krtd.restype = cls.C_TYPE_PTR

        with MaptkErrorHandle() as eh:
            return cls.from_c_pointer( cam_read_krtd(filepath, eh) )

    def _destroy(self):
        """ Delete instance through C API """
        if self._inst_ptr:
            cam_del = self.MAPTK_LIB.maptk_camera_destroy
            cam_del.argtypes = [self.C_TYPE_PTR, MaptkErrorHandle.C_TYPE_PTR]

            with MaptkErrorHandle() as eh:
                cam_del(self, eh)

    def write_krtd_file(self, filepath):
        """
        Write camera object in KRTD format to the specified file.

        :param filepath: Path to the file to write to.
        :type filepath: str

        """
        cam_write_krtd = self.MAPTK_LIB.maptk_camera_write_krtd_file
        cam_write_krtd.argtypes = [self.C_TYPE_PTR, ctypes.c_char_p,
                                   MaptkErrorHandle.C_TYPE_PTR]

        with MaptkErrorHandle() as eh:
            cam_write_krtd(self, filepath, eh)
