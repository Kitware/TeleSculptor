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

Interface to MAPTK image_container class.

"""
# -*- coding: utf-8 -*-
__author__ = 'purg'

import ctypes

from maptk import Image
from maptk.util import MaptkObject
from maptk.util import MaptkErrorHandle


class ImageContainer (MaptkObject):
    """
    maptk::image_container interface class
    """

    def __init__(self, image):
        """
        Create a simple image container from a Image instance

        :param image: Image to contain
        :type image: Image

        """
        super(ImageContainer, self).__init__()

        imgc_new = self.MAPTK_LIB.maptk_image_container_new_simple
        imgc_new.argtypes = [Image.C_TYPE_PTR]
        imgc_new.restype = self.C_TYPE_PTR
        self._inst_ptr = imgc_new(image.c_pointer)

        if not bool(self._inst_ptr):
            raise RuntimeError("Failed to construct new ImageContainer "
                               "instance.")

    def _destroy(self):
        imgc_del = self.MAPTK_LIB.maptk_image_container_destroy
        imgc_del.argtypes = [self.C_TYPE_PTR, MaptkErrorHandle.C_TYPE_PTR]
        with MaptkErrorHandle() as eh:
            imgc_del(self, eh)

    def size(self):
        """
        Get the size in bytes of this image container

        Size includes all allocated image memory, which could be larger than
        the product of width, height and depth.

        :return: Size in bytes
        :rtype: long

        """
        ic_size = self.MAPTK_LIB.maptk_image_container_size
        ic_size.argtypes = [self.C_TYPE_PTR]
        ic_size.restype = ctypes.c_size_t
        return ic_size(self)

    def width(self):
        """
        :return: The pixel width of the image
        :rtype: long
        """
        ic_width = self.MAPTK_LIB.maptk_image_container_width
        ic_width.argtypes = [self.C_TYPE_PTR]
        ic_width.restype = ctypes.c_size_t
        return ic_width(self)

    def height(self):
        """
        :return: The pixel height of the image
        :rtype: long
        """
        ic_height = self.MAPTK_LIB.maptk_image_container_height
        ic_height.argtypes = [self.C_TYPE_PTR]
        ic_height.restype = ctypes.c_size_t
        return ic_height(self)

    def depth(self):
        """
        :return: The depth (number of channels) of the image
        :rtype: long
        """
        ic_depth = self.MAPTK_LIB.maptk_image_container_depth
        ic_depth.argtypes = [self.C_TYPE_PTR]
        ic_depth.restype = ctypes.c_size_t
        return ic_depth(self)

    def get_image(self):
        """
        Return a new pointer the to contained image. This instance shares the
        same internal memory as the contained image.

        :return: New ImageContainer instance
        :rtype: ImageContainer

        """
        ic_getimg = self.MAPTK_LIB['maptk_image_container_get_image']
        ic_getimg.argtypes = [self.C_TYPE_PTR]
        ic_getimg.restype = Image.C_TYPE_PTR
        return Image.from_c_pointer(ic_getimg(self))
