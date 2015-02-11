# -*- coding: utf-8 -*-
__author__ = 'purg'

from maptk.util import find_maptk_library


class MaptkObject (object):
    """
    Basic MAPTK python interface class.
    """
    MAPTK_LIB = find_maptk_library()
