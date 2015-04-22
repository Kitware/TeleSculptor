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

import ctypes

from maptk import ConfigBlock
from maptk.exceptions.base import MaptkNullPointerException
from maptk.util import MaptkObject, MaptkErrorHandle


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
    # Subclass defined type string as would be returned by
    # maptk::algo::*::type_name. This allows this base class to know how to
    # automatically call common algorithm_def level methods.
    TYPE_NAME = None

    @classmethod
    def from_c_pointer(cls, ptr, shallow_copy_of=None, name=None):
        """
        Create a named algorithm instance of the derived class from a C API
        opaque pointer.

        If this the C pointer given to ptr is taken from an existing Python
        object instance, that object instance should be given to the
        shallow_copy_of argument. This ensures that the underlying C reference
        is not destroyed prematurely.

        This class-method override allows the passing of the algorithm instance
        name.

        :param ptr: C API opaque structure pointer type instance
        :type ptr: MaptkAlgorithm.C_TYPE_PTR

        :param shallow_copy_of: Optional parent object instance when the ptr
            given is coming from an existing python object.
        :type shallow_copy_of: MaptkAlgorithm or None

        :param name: Name of the new algorithm. This must be provided if this is
            not a copy of another algorithm instance.
        :type name: str or None

        :return: New Python object using the given underlying C object pointer.
        :rtype: MaptkAlgorithm

        """
        n = super(MaptkAlgorithm, cls).from_c_pointer(ptr, shallow_copy_of)

        if shallow_copy_of is None and not name:
            raise ValueError("Empty name given.")

        if name:
            n._name = name
        elif shallow_copy_of is not None:
            n._name = shallow_copy_of._name

        return n

    @classmethod
    def type_name(cls):
        """
        :return: String name for this algorithm type.
        :rtype: str

        :raises AttributeError: Class type name not correctly defined in derived
            class

        """
        if cls.TYPE_NAME is None:
            raise AttributeError("Derived class did not define TYPE_NAME, "
                                 "which is required in order to know what to "
                                 "call in the C API.")

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
    def create(cls, algo_name, impl_name):
        """
        Create a new algorithm instance of the derived type, initialized with
        the given implementation.

        :raises RuntimeError: The implementation name given does not match a
            registered implementation name.

        :param algo_name: Name to be assigned to this algorithm instance.
        :type algo_name: str

        :param impl_name: Name of the implementation. This should be one of the
            registered implementation names (check <type>.registered_names()).
        :type impl_name: str

        :return: New instance of derived type initialized to the given
            implementation.
        :rtype: cls

        """
        algo_create = cls.MAPTK_LIB['maptk_algorithm_%s_create'
                                    % cls.type_name()]
        algo_create.argtypes = [ctypes.c_char_p]
        algo_create.restype = cls.C_TYPE_PTR

        inst_ptr = algo_create(impl_name)
        if not bool(inst_ptr):
            raise MaptkNullPointerException(
                "Failed to construct algorithm instance"
            )

        return cls.from_c_pointer(inst_ptr, name=algo_name)

    def __init__(self, name):
        """
        Construct an un-initialized algorithm of the derived type with the given
        name.

        The name is used for configuration purposes in order to delineate this
        algorithm instance from others of the same type.

        :raises ValueError: The given name was empty.
        :raises AttributeError: Class type name not correctly defined in derived
            class

        """
        super(MaptkAlgorithm, self).__init__()
        # Initialize to NULL pointer
        self._inst_ptr = self.C_TYPE_PTR()

        # Checking that derived class has a valid typename
        self.type_name()

        if not name:
            raise ValueError("Empty name given.")
        self._name = name

    @property
    def name(self):
        """
        :return: The string name of this algorithm instance.
        :rtype: str
        """
        return self._name

    def set_name(self, new_name):
        """
        Reset the name of this algorithm instance.

        This effects interaction with configuration.

        :param new_name: The new string name to assign to this algorithm.
        :type new_name: str

        """
        if not new_name:
            raise ValueError("Invalid new algorithm name: '%s'" % new_name)
        self._name = new_name

    def _destroy(self):
        """
        Destroy internal instance
        """
        if self._inst_ptr:
            algo_destroy = self.MAPTK_LIB["maptk_algorithm_%s_destroy"
                                          % self.type_name()]
            algo_destroy.argtypes = [self.C_TYPE_PTR,
                                     MaptkErrorHandle.C_TYPE_PTR]
            with MaptkErrorHandle() as eh:
                algo_destroy(self, eh)

    def impl_name(self):
        """
        :return: String name for the current implementation, or None if this
            algorithm is not initialized to an implementation
        :rtype: str
        """
        if self._inst_ptr:
            algo_impl_name = self.MAPTK_LIB.maptk_algorithm_impl_name
            algo_impl_name.argtypes = [self.C_TYPE_PTR,
                                       MaptkErrorHandle.C_TYPE_PTR]
            algo_impl_name.restype = self.MST_TYPE_PTR
            with MaptkErrorHandle() as eh:
                s_ptr = algo_impl_name(self, eh)
                s = s_ptr.contents.str
                self.MST_FREE(s_ptr)
                return s
        else:
            return None

    def clone(self, new_name=None):
        """
        :param new_name: An optional new instance name for the cloned algorithm
        :type new_name: str

        :return: A new copy of this algorithm
        :rtype: MaptkAlgorithm
        """
        algo_clone = self.MAPTK_LIB['maptk_algorithm_%s_clone'
                                    % self.type_name()]
        algo_clone.argtypes = [self.C_TYPE_PTR, MaptkErrorHandle.C_TYPE_PTR]
        algo_clone.restype = self.C_TYPE_PTR
        with MaptkErrorHandle() as eh:
            return self.from_c_pointer(algo_clone(self, eh),
                                       name=(new_name or self.name))

    def get_config(self, cb=None):
        """
        Return this algorithm's current configuration.

        As algorithms are named, the configuration returned will be contained
        inside a sub-block whose name matches this algorithm's name in order
        to separate

        :param cb: An optional existing ConfigBlock instance to add this
            algorithm's configuration to.
        :type cb: ConfigBlock

        :return: This named algorithm's current configuration. If a config block
            was given to this method, the same config block is returned.
        :rtype: maptk.ConfigBlock

        """
        algo_gtc = self.MAPTK_LIB['maptk_algorithm_%s_get_type_config'
                                  % self.type_name()]
        algo_gtc.argtypes = [ctypes.c_char_p, self.C_TYPE_PTR,
                             ConfigBlock.C_TYPE_PTR,
                             MaptkErrorHandle.C_TYPE_PTR]
        algo_gtc.restype = ConfigBlock.C_TYPE_PTR

        if cb is None:
            cb = ConfigBlock()

        with MaptkErrorHandle() as eh:
            algo_gtc(self.name, self, cb, eh)

        return cb

    def set_config(self, cb):
        """
        Set this algorithm's configuration based on the given config block

        If there is no configuration for this algorithm in the given block, or
        if the recorded implementation type is invalid, this algorithm remains
        unchanged.

        NOTE: Setting an algorithm configuration with a valid implementation
        type, the underlying instance is reset (deleted and reconstructed). This
        is important to note for algorithms that are state-based as their states
        would implicitly reset due to instance reconstruction.

        :param cb: Configuration block to draw this algorithm's configuration
            from.
        :type cb: ConfigBlock

        """
        algo_stc = self.MAPTK_LIB['maptk_algorithm_%s_set_type_config'
                                  % self.type_name()]
        algo_stc.argtypes = [ctypes.c_char_p, ConfigBlock.C_TYPE_PTR,
                             ctypes.POINTER(self.C_TYPE_PTR),
                             MaptkErrorHandle.C_TYPE_PTR]
        with MaptkErrorHandle() as eh:
            algo_stc(self.name, cb, ctypes.byref(self.c_pointer), eh)

    def check_config(self, cb):
        """
        Check the given configuration block for valid algorithm configuration

        :param cb: Configuration to check.
        :type cb: ConfigBlock

        :return: True if the given configuration block contains a valid
            configuration for this algorithm type and implementation.
        :rtype: bool

        """
        algo_ctc = self.MAPTK_LIB['maptk_algorithm_%s_check_type_config'
                                  % self.type_name()]
        algo_ctc.argtypes = [ctypes.c_char_p,
                             ConfigBlock.C_TYPE_PTR,
                             MaptkErrorHandle.C_TYPE_PTR]
        algo_ctc.restype = ctypes.c_bool
        with MaptkErrorHandle() as eh:
            return algo_ctc(self.name, cb, eh)
