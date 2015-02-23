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

MAPTK image_io algorithm interface

"""
# -*- coding: utf-8 -*-
__author__ = 'purg'

import ctypes

from maptk import MaptkImageContainer
from maptk.algo import MaptkAlgorithm
from maptk.util import propagate_exception_from_handle


class MaptkAlgoImageIo (MaptkAlgorithm):
    """
    maptk::algo::image_io interface
    """

    TYPE_NAME = 'image_io'

    def load(self, filepath):
        """
        Load an image into a MaptkImageContainer

        :param filepath: Path to the image to load
        :type filepath: str

        :return: New MaptkImageContainer containing the loaded image
        :rtype: MaptkImageContainer

        """
        iio_load = self.MAPTK_LIB.maptk_algorithm_image_io_load
        iio_load.argtypes = [self.C_TYPE_PTR, ctypes.c_char_p,
                             self.EH_TYPE_PTR]
        iio_load.restype = MaptkImageContainer.C_TYPE_PTR

        eh = self.EH_NEW()
        try:
            ic_ptr = iio_load(self._inst_ptr, filepath, eh)
            propagate_exception_from_handle(eh)
            return MaptkImageContainer.from_c_pointer(ic_ptr)
        finally:
            self.EH_DEL(eh)

    def save(self, filepath, image_container):
        """
        Save an image to the specified path

        :param filepath: The path to save the image to
        :type filepath: str

        :param image_container: MaptkImageContainer contianing the image to save
        :type image_container: MaptkImageContainer

        """
        iio_save = self.MAPTK_LIB.maptk_algorithm_image_io_save
        iio_save.argtypes = [self.C_TYPE_PTR, ctypes.c_char_p,
                             MaptkImageContainer.C_TYPE_PTR]

        eh = self.EH_NEW()
        try:
            iio_save(self.c_pointer, filepath, image_container.c_pointer, eh)
            propagate_exception_from_handle(eh)
        finally:
            self.EH_DEL(eh)
