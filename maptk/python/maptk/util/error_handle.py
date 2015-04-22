# -*- coding: utf-8 -*-
__author__ = 'purg'

import ctypes

from maptk.util.MaptkObject import MaptkObject
from maptk.exceptions.base import MaptkBaseException


# noinspection PyPep8Naming
class MaptkErrorHandle (MaptkObject):
    """ Error handling structure used in C interface """

    # noinspection PyPep8Naming
    class C_TYPE (ctypes.Structure):
        """
        C Interface structure
        """
        _fields_ = [
            ("error_code", ctypes.c_int),
            ("message", ctypes.c_char_p),
        ]

    C_TYPE_PTR = ctypes.POINTER(C_TYPE)

    def __init__(self):
        """
        Create a new error handle instance
        """
        super(MaptkErrorHandle, self).__init__()

        eh_new = self.MAPTK_LIB['maptk_eh_new']
        eh_new.restype = self.C_TYPE_PTR
        self._inst_ptr = eh_new()
        if not self._inst_ptr:
            raise RuntimeError("Failed construct new error handle instance")

        self._ec_exception_map = {}

    def _destroy(self):
        eh_del = self.MAPTK_LIB['maptk_eh_destroy']
        eh_del.argtypes = [self.C_TYPE_PTR]
        eh_del(self._inst_ptr)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is not None:
            return False
        else:
            self.propagate_exception()
        return True

    def set_exception_map(self, ec_exception_map):
        """
        Extend the current return code to exception mapping.

        :param ec_exception_map: Dictionary mapping integer return code to an
            exception, or function returning an exception instance, that should
            be raised.
        :type ec_exception_map: dict of (int, BaseException or types.FunctionType)

        """
        self._ec_exception_map.update(ec_exception_map)

    def propagate_exception(self):
        """
        Raise appropriate Python exception if our current error code is non-zero

        By default, if a non-zero error code is observed, a generic
        MaptkBaseException is raised with the provided error handle message.

        If an exception map was set via set_exception_map(...) and the error
        code matches an entry, that will be raised instead.

        """
        c_ptr = self.c_pointer
        if c_ptr[0].error_code != 0:
            if c_ptr[0].error_code in self._ec_exception_map:
                raise self._ec_exception_map[c_ptr[0].error_code](c_ptr[0].message)
            else:
                raise MaptkBaseException(c_ptr[0].message)
