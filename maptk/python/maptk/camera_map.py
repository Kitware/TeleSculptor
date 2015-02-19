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

Interface to maptk::camera_map class.

"""
# -*- coding: utf-8 -*-
__author__ = 'purg'

import ctypes

from maptk import MaptkCamera
from maptk.util import (
    MaptkObject,
    propagate_exception_from_handle
)


# noinspection PyPep8Naming
class _maptk_camera_map_t (ctypes.Structure):
    """ C interface opaque handle structure """
    pass


class MaptkCameraMap (MaptkObject):
    """ maptk::camera_map interface class """

    # C API structure + pointer
    C_TYPE = _maptk_camera_map_t
    C_TYPE_PTR = ctypes.POINTER(_maptk_camera_map_t)

    def __init__(self, frame2cam_map):
        """
        :param frame2cam_map: Association of frame number to camera instance
        :type frame2cam_map: dict of (int, maptk.MaptkCamera)

        """
        super(MaptkCameraMap, self).__init__()

        cm_new = self.MAPTK_LIB.maptk_camera_map_new
        cm_new.argtypes = [ctypes.c_size_t, ctypes.POINTER(ctypes.c_uint),
                           ctypes.POINTER(MaptkCamera.C_TYPE_PTR)]
        cm_new.restype = self.C_TYPE_PTR

        # Construct input frame and camera arrays
        fn_list = []
        cam_list = []
        for fn, c in frame2cam_map.iteritems():
            fn_list.append(fn)
            cam_list.append(c)
        fn_array_t = ctypes.c_uint * len(frame2cam_map)
        cam_array_t = MaptkCamera.C_TYPE_PTR * len(frame2cam_map)
        # noinspection PyCallingNonCallable
        c_fn_array = fn_array_t(*fn_list)
        # noinspection PyCallingNonCallable,PyProtectedMember
        c_cam_array = cam_array_t(*(c._inst_ptr for c in cam_list))

        self._inst_ptr = cm_new(len(frame2cam_map),
                                c_fn_array, c_cam_array)

        if not self._inst_ptr:
            raise RuntimeError("Failed to construct a valid MaptkCameraMap "
                               "instance")

    def _destroy(self):
        cm_del = self.MAPTK_LIB.maptk_camera_map_destroy
        cm_del.argtypes = [self.C_TYPE_PTR, self.EH_TYPE_PTR]
        eh = self.EH_NEW()
        cm_del(self._inst_ptr, eh)
        try:
            propagate_exception_from_handle(eh)
        finally:
            self.EH_DEL(eh)

    def size(self):
        """
        :return: Number of elements in this mapping.
        :rtype: int
        """
        cm_size = self.MAPTK_LIB.maptk_camera_map_size
        cm_size.argtypes = [self.C_TYPE_PTR, self.EH_TYPE_PTR]
        cm_size.restype = ctypes.c_size_t
        eh = self.EH_NEW()
        try:
            s = cm_size(self._inst_ptr, eh)
            propagate_exception_from_handle(eh)
            return s
        finally:
            self.EH_DEL(eh)
