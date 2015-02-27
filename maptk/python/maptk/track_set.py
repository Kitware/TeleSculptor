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

Interface to MAPTK track_set class.

"""
# -*- coding: utf-8 -*-
__author__ = 'purg'

import ctypes

from maptk import MaptkTrack
from maptk.util import MaptkObject, MaptkErrorHandle


class MaptkTrackSet (MaptkObject):
    """
    maptk::track_set interface class
    """

    def __init__(self, track_list=None):
        """
        :param track_list: List of tracks to initialize the set with
        :type track_list: list of MaptkTrack
        """
        super(MaptkTrackSet, self).__init__()

        ts_new = self.MAPTK_LIB['maptk_trackset_new']
        ts_new.argtypes = [ctypes.c_size_t,
                           ctypes.POINTER(MaptkTrack.C_TYPE_PTR)]
        ts_new.restype = self.C_TYPE_PTR

        if track_list is None:
            track_list = []
        # noinspection PyCallingNonCallable
        c_track_array = (MaptkTrack.C_TYPE_PTR * len(track_list))(
            *(t.c_pointer for t in track_list)
        )
        self._inst_ptr = ts_new(len(track_list), c_track_array)
        if not self._inst_ptr:
            raise RuntimeError("Failed to construct a new track set instance "
                               "(NULL pointer returned)")

    def _destroy(self):
        ts_del = self.MAPTK_LIB['maptk_trackset_destroy']
        ts_del.argtypes = [self.C_TYPE_PTR, MaptkErrorHandle.C_TYPE_PTR]
        with MaptkErrorHandle() as eh:
            ts_del(self, eh)

    def __len__(self):
        ts_size = self.MAPTK_LIB['maptk_trackset_size']
        ts_size.argtypes = [self.C_TYPE_PTR, MaptkErrorHandle.C_TYPE_PTR]
        ts_size.restype = ctypes.c_size_t
        with MaptkErrorHandle() as eh:
            return ts_size(self, eh)

    def size(self):
        return len(self)
