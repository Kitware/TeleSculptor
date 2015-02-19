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

from maptk.util import find_maptk_library
from maptk.util.error_handle import c_maptk_error_handle_p


class MaptkObject (object):
    """
    Basic MAPTK python interface class.
    """
    __metaclass__ = abc.ABCMeta

    MAPTK_LIB = find_maptk_library()

    # C API opaque structure + pointer
    C_TYPE = None
    C_TYPE_PTR = None

    # Configured common error handle constructor and destructor
    EH_TYPE_PTR = c_maptk_error_handle_p
    EH_NEW = MAPTK_LIB.maptk_eh_new
    EH_NEW.restype = c_maptk_error_handle_p
    EH_DEL = MAPTK_LIB.maptk_eh_destroy
    EH_DEL.argtypes = [c_maptk_error_handle_p]

    @classmethod
    def from_c_pointer(cls, ptr, is_copy_of=None):
        """
        Create an instance of the derived class from a C API opaque pointer.

        If this the C pointer given to ptr is taken form an existing Python
        object instance, that object instance should be given to the
        is_copy_of argument. This ensures that the underlying C reference is
        not destroyed prematurely.

        :param ptr: C API opaque structure pointer type instance
        :type ptr: cls.C_TYPE_PTR

        :param is_copy_of: Optional parent object instance when the ptr given
            is coming from an existing python object.
        :type ptr: cls

        :return: New Python object using the given underlying C object pointer.

        """
        # As this is a generalized method, make sure that the derived class
        # has a C opaque structure pointer type defined.
        if cls.C_TYPE_PTR is None:
            raise RuntimeError("Derived class '%s' did not define C_TYPE_PTR"
                               % cls.__name__)

        assert isinstance(ptr, cls.C_TYPE_PTR), \
            "Required a C_TYPE_PTR instance of this class (%s)" \
            % cls.__name__

        # noinspection PyPep8Naming,PyMissingConstructor,PyAbstractClass
        class _from_c_pointer (cls):
            def __init__(self, _ptr, _is_copy_of):
                self._inst_ptr = _ptr
                self._parent = _is_copy_of

        _from_c_pointer.__name__ = "%s_from_c_pointer" % cls.__name__

        return _from_c_pointer(ptr, is_copy_of)

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
            print "!!! Destroying parent python instance"
            self._destroy()

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
