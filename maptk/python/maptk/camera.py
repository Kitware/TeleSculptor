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

from maptk.util import (
    MaptkObject,
    c_maptk_error_handle_p,
    propagate_exception_from_handle
)


# noinspection PyPep8Naming
class _maptk_camera_t (ctypes.Structure):
    """ C interface opaque handle structure """
    pass


class MaptkCamera (MaptkObject):
    """ maptk::camera interface class """

    # C API config_block_t structure + pointer
    C_TYPE = _maptk_camera_t
    C_TYPE_PTR = ctypes.POINTER(_maptk_camera_t)

    def __del__(self):
        """ Delete instance through C API """
        if self._inst_ptr:
            eh_new = self.MAPTK_LIB.maptk_eh_new
            eh_new.restype = c_maptk_error_handle_p

            eh_del = self.MAPTK_LIB.maptk_eh_destroy
            eh_del.argtypes = [c_maptk_error_handle_p]

            cam_del = self.MAPTK_LIB.maptk_camera_destroy
            cam_del.argtypes = [self.C_TYPE_PTR, c_maptk_error_handle_p]

            eh = eh_new()
            cam_del(self._inst_ptr, eh)

            try:
                propagate_exception_from_handle(eh)
            finally:
                eh_del(eh)
