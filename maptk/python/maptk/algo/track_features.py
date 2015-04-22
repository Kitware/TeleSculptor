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

MAPTK track_features algorithm interface

"""
# -*- coding: utf-8 -*-
__author__ = 'purg'

import ctypes

from maptk import ImageContainer, TrackSet
from maptk.algo import MaptkAlgorithm
from maptk.util import MaptkErrorHandle


class TrackFeatures (MaptkAlgorithm):
    """
    maptk::algo::track_features interface
    """

    TYPE_NAME = "track_features"

    def track(self, prev_tracks, frame_num, image, mask=None):
        """
        Extend a previous set of tracks using the given image.

        An optional mask image may be provided, where positive valued regions
        indicate regions of the input image to consider for feature tracking.
        The mask image must be of the same dimensions as the input image, other
        wise an exception is raised.

        :param prev_tracks:
        :param frame_num:
        :param image:
        :param mask:
        :return:

        """
        tf_track_argtypes = [self.C_TYPE_PTR, TrackSet.C_TYPE_PTR,
                             ctypes.c_uint, ImageContainer.C_TYPE_PTR]
        tf_track_args = [self, prev_tracks, frame_num, image]
        tf_track_restype = TrackSet.C_TYPE_PTR

        if mask:
            tf_track = self.MAPTK_LIB['maptk_algorithm_track_features_'
                                      'track_with_mask']
            tf_track_argtypes.append(ImageContainer.C_TYPE_PTR)
            tf_track_args.append(mask)
        else:
            tf_track = self.MAPTK_LIB['maptk_algorithm_track_features_track']
        tf_track_argtypes.append(MaptkErrorHandle.C_TYPE_PTR)

        tf_track.argtypes = tf_track_argtypes
        tf_track.restype = tf_track_restype

        with MaptkErrorHandle() as eh:
            tf_track_args.append(eh)
            return TrackSet.from_c_pointer(
                tf_track(*tf_track_args)
            )
