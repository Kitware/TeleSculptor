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

Interface to MAPTK track class.

"""
# -*- coding: utf-8 -*-
__author__ = 'purg'

import ctypes

from maptk.util import MaptkObject, propagate_exception_from_handle


class MaptkTrack (MaptkObject):
    """
    maptk::track interface class
    """

    class _maptk_track_t (ctypes.Structure):
        """
        Opaque structure type used in C interface.
        """
        pass

    # C API structure + pointer
    C_TYPE = _maptk_track_t
    C_TYPE_PTR = ctypes.POINTER(C_TYPE)

    def __init__(self):
        super(MaptkTrack, self).__init__()

        t_new = self.MAPTK_LIB['maptk_track_new']
        t_new.restype = self.C_TYPE_PTR
        self._inst_ptr = t_new()

        if not bool(self._inst_ptr):
            raise RuntimeError("Failed to construct a new track instance (NULL "
                               "pointer returned)")

    def _destroy(self):
        t_del = self.MAPTK_LIB['maptk_track_destroy']
        t_del.argtypes = [self.C_TYPE_PTR]

        eh = self.EH_NEW()
        try:
            t_del(self.c_pointer, eh)
            propagate_exception_from_handle(eh)
        finally:
            self.EH_DEL(eh)

    def __len__(self):
        """
        :return: The number of states in this track
        :rtype: int
        """
        t_size = self.MAPTK_LIB['maptk_track_size']
        t_size.argtypes = [self.C_TYPE_PTR, self.EH_TYPE_PTR]
        t_size.restype = ctypes.c_size_t

        eh = self.EH_NEW()
        try:
            s = t_size(self.c_pointer, eh)
            propagate_exception_from_handle(eh)
            return s
        finally:
            self.EH_DEL(eh)

    @property
    def size(self):
        """
        :return: The number of states in this track
        :rtype: int
        """
        return len(self)

    @property
    def is_empty(self):
        """
        :return: If this track has no track states or not
        :rtype: bool
        """
        t_empty = self.MAPTK_LIB['maptk_track_empty']
        t_empty.argtypes = [self.C_TYPE_PTR, self.EH_TYPE_PTR]
        t_empty.restype = ctypes.c_bool

        eh = self.EH_NEW()
        try:
            b = t_empty(self.c_pointer, eh)
            propagate_exception_from_handle(eh)
            return b
        finally:
            self.EH_DEL(eh)
