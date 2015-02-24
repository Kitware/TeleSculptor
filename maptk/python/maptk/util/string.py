# -*- coding: utf-8 -*-
__author__ = 'purg'

import ctypes


# noinspection PyPep8Naming
class maptk_string_t (ctypes.Structure):
    """
    Python analogue structure for C interface maptk_string_t
    """
    _fields_ = [
        ("length", ctypes.c_size_t),
        ("str", ctypes.c_char_p),
    ]

    # Structure pointer type
    PTR_t = None

maptk_string_t.PTR_t = ctypes.POINTER(maptk_string_t)
