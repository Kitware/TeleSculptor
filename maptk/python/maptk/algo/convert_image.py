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

MAPTK convert_image algorithm interface

"""
# -*- coding: utf-8 -*-
__author__ = 'purg'

from . import MaptkAlgorithm

from maptk import MaptkImageContainer
import maptk.util


class MaptkAlgoConvertImage (MaptkAlgorithm):

    TYPE_NAME = 'convert_image'

    def convert(self, image_container):
        """
        :param image_container: The image container with image data to convert
        :type image_container: MaptkImageContainer

        :return: A new MaptkImageContainer with the converted underlying data
        :rtype: MaptkImageContainer

        """
        ci_convert = self.MAPTK_LIB['maptk_algorithm_convert_image_convert']
        ci_convert.argtypes = [self.C_TYPE_PTR, MaptkImageContainer.C_TYPE_PTR,
                               self.EH_TYPE_PTR]
        ci_convert.restype = MaptkImageContainer.C_TYPE_PTR

        eh = self.EH_NEW()
        try:
            new_ic_ptr = ci_convert(self.c_pointer, image_container.c_pointer,
                                    eh)
            maptk.util.propagate_exception_from_handle(eh)
            return MaptkImageContainer.from_c_pointer(new_ic_ptr)
        finally:
            self.EH_DEL(eh)
