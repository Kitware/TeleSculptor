# -*- coding: utf-8 -*-
__author__ = 'purg'

import ctypes

from maptk.exceptions.base import MaptkBaseException


# noinspection PyPep8Naming
class c_maptk_error_handle (ctypes.Structure):
    """ Error handling structure used in C interface """
    _fields_ = [
        ("error_code", ctypes.c_int),
        ("message", ctypes.c_char_p),
    ]

c_maptk_error_handle_p = ctypes.POINTER(c_maptk_error_handle)


def propagate_exception_from_handle(eh_ptr, ec_exception_map=None):
    """
    Raise appropriate Python exception if the given error handle has a non-zero
    error code.

    By default, if a non-zero error code is observed, a generic
    MaptkBaseException is raised with the provided error handle message.

    Optionally, ec_exception_map may be defined as a mapping of integers to
    an exception class, or function that returns an exception instance, if a
    specific exception or action should occur instead. Such an exception class
    or function should take a single argument (the exception message).

    :param eh_ptr: error handle instance pointer
    :type eh_ptr: c_maptk_error_handle_p

    :param ec_exception_map: Optional exception handle association map
    :type ec_exception_map: dict of (int, types.ClassType or types.FunctionType)

    """
    if eh_ptr[0].error_code != 0:
        if ec_exception_map and ec_exception_map.has_key(eh_ptr[0].error_code):
            raise ec_exception_map[eh_ptr[0].error_code](eh_ptr[0].message)
        else:
            raise MaptkBaseException(eh_ptr[0].message)
