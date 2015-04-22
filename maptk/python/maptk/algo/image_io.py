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

from maptk import ImageContainer
from maptk.algo import MaptkAlgorithm
from maptk.util import MaptkErrorHandle


class ImageIo (MaptkAlgorithm):
    """
    maptk::algo::image_io interface
    """

    TYPE_NAME = 'image_io'

    def load(self, filepath):
        """
        Load an image into a ImageContainer

        :param filepath: Path to the image to load
        :type filepath: str

        :return: New ImageContainer containing the loaded image
        :rtype: ImageContainer

        """
        iio_load = self.MAPTK_LIB.maptk_algorithm_image_io_load
        iio_load.argtypes = [self.C_TYPE_PTR, ctypes.c_char_p,
                             MaptkErrorHandle.C_TYPE_PTR]
        iio_load.restype = ImageContainer.C_TYPE_PTR
        with MaptkErrorHandle() as eh:
            ic_ptr = iio_load(self, filepath, eh)
        return ImageContainer.from_c_pointer(ic_ptr)

    def save(self, image_container, filepath):
        """
        Save an image to the specified path

        :param image_container: ImageContainer containing the image to save
        :type image_container: ImageContainer

        :param filepath: The path to save the image to
        :type filepath: str

        """
        iio_save = self.MAPTK_LIB.maptk_algorithm_image_io_save
        iio_save.argtypes = [self.C_TYPE_PTR, ctypes.c_char_p,
                             ImageContainer.C_TYPE_PTR,
                             MaptkErrorHandle.C_TYPE_PTR]
        with MaptkErrorHandle() as eh:
            iio_save(self, filepath, image_container, eh)
