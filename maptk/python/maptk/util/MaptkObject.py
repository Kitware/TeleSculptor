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

Base class for all MAPTK Python interface classes

"""
# -*- coding: utf-8 -*-
__author__ = 'purg'

import abc
import ctypes

from maptk.util import find_maptk_library
from maptk.util.string import maptk_string_t


class MaptkClassMeta (abc.ABCMeta):
    """
    Metaclass for Maptk object types.

    Ensures that C_TYPE and C_TYPE_PTR are defined in derived classes.
    """

    def __new__(cls, name, bases, attrs):

        # Create a new structure type for the class if it has not already
        # defined one for itself
        if 'C_TYPE' not in attrs:
            class OpaqueStruct (ctypes.Structure):
                pass
            OpaqueStruct.__name__ = "%sOpaqueStruct" % name
            attrs['C_TYPE'] = OpaqueStruct
            attrs['C_TYPE_PTR'] = ctypes.POINTER(OpaqueStruct)

        return super(MaptkClassMeta, cls).__new__(cls, name, bases, attrs)


class MaptkObject (object):
    """
    Basic MAPTK python interface class.
    """
    __metaclass__ = MaptkClassMeta

    MAPTK_LIB = find_maptk_library()

    # C API opaque structure + pointer
    C_TYPE = None
    C_TYPE_PTR = None

    # Common string structure stuff
    MST_TYPE = maptk_string_t
    MST_TYPE_PTR = maptk_string_t.PTR_t
    MST_NEW = MAPTK_LIB['maptk_string_new']
    MST_NEW.argtypes = [ctypes.c_size_t, ctypes.c_char_p]
    MST_NEW.restype = maptk_string_t.PTR_t
    MST_FREE = MAPTK_LIB['maptk_string_free']
    MST_FREE.argtypes = [maptk_string_t.PTR_t]

    @classmethod
    def from_c_pointer(cls, ptr, shallow_copy_of=None):
        """
        Create an instance of the derived class from a C API opaque pointer.

        If this the C pointer given to ptr is taken from an existing Python
        object instance, that object instance should be given to the
        shallow_copy_of argument. This ensures that the underlying C reference
        is not destroyed prematurely.

        :param ptr: C API opaque structure pointer type instance
        :type ptr: MaptkAlgorithm.C_TYPE_PTR

        :param shallow_copy_of: Optional parent object instance when the ptr
            given is coming from an existing python object.
        :type shallow_copy_of: MaptkObject or None

        :return: New Python object using the given underlying C object pointer.
        :rtype: MaptkObject

        """
        # As this is a generalized method, make sure that the derived class
        # has a C opaque structure pointer type defined.
        if cls.C_TYPE_PTR is None:
            raise RuntimeError("Derived class '%s' did not define C_TYPE_PTR"
                               % cls.__name__)

        assert isinstance(ptr, cls.C_TYPE_PTR), \
            "Required a C_TYPE_PTR instance of this class (%s)" \
            % cls.__name__

        # Custom child class of calling type in order to bypass constructor of
        # calling type. Since we already have the underlying instance, we
        # fundamentally don't want to "construct" again.
        # noinspection PyPep8Naming,PyMissingConstructor,PyAbstractClass
        class _from_c_pointer (cls):
            # Need to set from parent class in order to prevent the metaclass
            # from creating a different opaque structure, which messes with
            # type-checking when calling C API functions.
            C_TYPE = cls.C_TYPE
            C_TYPE_PTR = cls.C_TYPE_PTR

            def __init__(self, _ptr, _is_copy_of):
                self._inst_ptr = _ptr
                self._parent = _is_copy_of

        # TODO: Can we just make this the same name as the parent class?
        #       Or would that cause issues.
        _from_c_pointer.__name__ = "%s_from_c_pointer" % cls.__name__

        return _from_c_pointer(ptr, shallow_copy_of)

    def __init__(self):
        if None in (self.C_TYPE, self.C_TYPE_PTR):
            raise RuntimeError("Derived class did not define opaque handle "
                               "structure types.")
        self._inst_ptr = None

        # When this instance is copied in python, carry a copy of the instance
        # it was copied from to leverage Python's internal GC ref counting
        self._parent = None

    def __del__(self):
        if self._parent is None:
            self._destroy()

    def __nonzero__(self):
        """ bool() operator for 2.x """
        return bool(self.c_pointer)

    def __bool__(self):
        """ bool() operator for 3.x """
        return bool(self.c_pointer)

    @property
    def _as_parameter_(self):
        """
        Ctypes interface attribute for passing object instance as argument to
        C function. This basically means that when an instance of this class
        is passed as an argument, the underlying opaque pointer is passed.
        """
        return self.c_pointer

    @property
    def c_pointer(self):
        """
        :return: The ctypes opaque structure pointer
        """
        return self._inst_ptr

    @abc.abstractmethod
    def _destroy(self):
        """ Call C API destructor for derived class """
        raise NotImplementedError("Calling MaptkObject class abstract _destroy "
                                  "function.")
