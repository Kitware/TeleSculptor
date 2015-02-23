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

Base MAPTK algorithm structure

"""
# -*- coding: utf-8 -*-
__author__ = 'purg'

import abc
import ctypes

from maptk.util import MaptkObject, propagate_exception_from_handle


# noinspection PyPep8Naming
class _maptk_algorithm_t (ctypes.Structure):
    """
    Opaque structure type used in C interface.
    """
    pass


class MaptkAlgorithm (MaptkObject):
    """
    Base class for MAPTK algorithms
    """
    __metaclass__ = abc.ABCMeta

    # C API config_block_t structure + pointer
    C_TYPE = _maptk_algorithm_t
    C_TYPE_PTR = ctypes.POINTER(_maptk_algorithm_t)

    # Subclass defined type string as would be returned by
    # maptk::algo::*::type_name. This allows this base class to know how to
    # automatically call common algorithm_def level methods.
    TYPE_NAME = None

    @classmethod
    def type_name(cls):
        """
        :return: String name for this algorithm type.
        :rtype: str
        """
        if cls.TYPE_NAME is None:
            raise RuntimeError("Derived class did not define TYPE_NAME, which "
                               "is required in order to know what to call in "
                               "the C API.")

        return cls.TYPE_NAME

    @classmethod
    def registered_names(cls):
        """
        :return: list of string implementation names currently registered
        """
        algo_reg_names = cls.MAPTK_LIB['maptk_algorithm_%s_registered_names'
                                       % cls.type_name()]
        sl_free = cls.MAPTK_LIB.maptk_common_free_string_list

        algo_reg_names.argtypes = [ctypes.POINTER(ctypes.c_uint),
                                   ctypes.POINTER(ctypes.POINTER(ctypes.c_char_p))]
        sl_free.argtypes = [ctypes.c_uint, ctypes.POINTER(ctypes.c_char_p)]

        length = ctypes.c_uint(0)
        names = ctypes.POINTER(ctypes.c_char_p)()
        algo_reg_names(ctypes.byref(length), ctypes.byref(names))

        # Constructing return array
        r = []
        for i in xrange(length.value):
            r.append(names[i])

        # Free allocated key listing
        sl_free(length, names)

        return r

    @classmethod
    def create(cls, impl_name):
        algo_create = cls.MAPTK_LIB['maptk_algorithm_%s_create'
                                    % cls.type_name()]
        algo_create.argtypes = [ctypes.c_char_p]
        algo_create.restype = cls.C_TYPE_PTR

        inst_ptr = algo_create(impl_name)
        if not bool(inst_ptr):
            raise RuntimeError("Failed to construct algorithm instance (null "
                               "pointer returned)")

        return cls.from_c_pointer(inst_ptr)

    def __init__(self):
        """
        Construct an un-initialize algorithm of the derived type.
        """
        super(MaptkAlgorithm, self).__init__()
        # Initialize to NULL pointer
        self._inst_ptr = self.C_TYPE_PTR()

    def _destroy(self):
        """
        Destroy internal instance
        """
        if self._inst_ptr:
            algo_destroy = self.MAPTK_LIB["maptk_algorithm_%s_destroy"
                                          % self.type_name()]
            algo_destroy.argtypes = [self.C_TYPE_PTR]
            algo_destroy.restype = ctypes.c_uint

            eh = self.EH_NEW()
            algo_destroy(self._inst_ptr, eh)
            try:
                propagate_exception_from_handle(eh)
            finally:
                self.EH_DEL(eh)

    def impl_name(self):
        if self._inst_ptr:
            algo_impl_name = self.MAPTK_LIB.maptk_algorithm_impl_name
            algo_impl_name.argtypes = [self.C_TYPE_PTR]
            algo_impl_name.restype = ctypes.c_char_p

            eh = self.EH_NEW()
            try:
                n = algo_impl_name(self._inst_ptr, eh)
                propagate_exception_from_handle(eh)
                return n
            finally:
                self.EH_DEL(eh)
        else:
            return None

    def clone(self):
        """
        :return: A new copy of this algorithm
        :rtype: MaptkAlgorithm
        """
        algo_clone = self.MAPTK_LIB['maptk_algorithm_%s_clone'
                                    % self.type_name()]
        algo_clone.argtypes = [self.C_TYPE_PTR]
        algo_clone.restype = self.C_TYPE_PTR

        eh = self.EH_NEW()
        try:
            new_ptr = algo_clone(self._inst_ptr, eh)
            propagate_exception_from_handle(eh)
            return self.from_c_pointer(new_ptr)
        finally:
            self.EH_DEL(eh)


# Convenience imports of algorithm definition types to the module level
from .convert_image import MaptkAlgoConvertImage
from .image_io import MaptkAlgoImageIo
