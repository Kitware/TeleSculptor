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

Interface to MAPTK config_block class.

"""
# -*- coding: utf-8 -*-
__author__ = 'purg'

import ctypes

from maptk.exceptions.config_block_io import (
    MaptkConfigBlockIoException,
    MaptkConfigBlockIoFileNotFoundException,
    MaptkConfigBlockIoFileNotReadException,
    MaptkConfigBlockIoFileNotParsed,
    MaptkConfigBlockIoFileWriteException,
)
from maptk.util import MaptkObject, propagate_exception_from_handle


# noinspection PyPep8Naming
class _maptk_config_block_t (ctypes.Structure):
    """
    Opaque structure type used in C interface.
    """
    pass


class MaptkConfigBlock (MaptkObject):
    """
    maptk::config_block interface class
    """
    # C API structure + pointer
    C_TYPE = _maptk_config_block_t
    C_TYPE_PTR = ctypes.POINTER(_maptk_config_block_t)

    # MaptkConfigBlock Constants
    BLOCK_SEP = \
        ctypes.c_char_p.in_dll(MaptkObject.MAPTK_LIB,
                               "maptk_config_block_block_sep").value
    GLOBAL_VALUE = \
        ctypes.c_char_p.in_dll(MaptkObject.MAPTK_LIB,
                               "maptk_config_block_global_value").value

    @classmethod
    def from_file(cls, filepath):
        """
        Return a new MaptkConfigBlock object based on the given configuration
        file

        :raises MaptkBaseException: Error occurred in reading from configuration
            file path given.

        :param filepath: Path to a configuration file
        :type filepath: str

        :return: New MaptkConfigBlock instance
        :rtype: MaptkConfigBlock

        """
        cb_read = cls.MAPTK_LIB.maptk_config_block_file_read
        cb_read.argtypes = [ctypes.c_char_p, cls.EH_TYPE_PTR]
        cb_read.restype = cls.C_TYPE_PTR

        eh = cls.EH_NEW()
        cb_ptr = cb_read(str(filepath), eh)

        try:
            propagate_exception_from_handle(eh, {
                -1: MaptkConfigBlockIoException,
                1: MaptkConfigBlockIoFileNotFoundException,
                2: MaptkConfigBlockIoFileNotReadException,
                3: MaptkConfigBlockIoFileNotParsed
            })
        finally:
            cls.EH_DEL(eh)

        return MaptkConfigBlock.from_c_pointer(cb_ptr)

    def __init__(self, name=None):
        super(MaptkConfigBlock, self).__init__()

        if name:
            cb_new = self.MAPTK_LIB.maptk_config_block_new_named
            cb_new.argtypes = [ctypes.c_char_p]
            cb_new.restype = self.C_TYPE_PTR
            self._inst_ptr = cb_new(str(name))
        else:
            cb_new = self.MAPTK_LIB.maptk_config_block_new
            cb_new.restype = self.C_TYPE_PTR
            self._inst_ptr = cb_new()

    def _destroy(self):
        # print "Destroying CB: \"%s\" %d" % (self.name, self._inst_ptr)
        self.MAPTK_LIB.maptk_config_block_destroy(self._inst_ptr)

    @property
    def name(self):
        """
        :return: The name assigned to this MaptkConfigBlock instance.
        :rtype: str
        """
        cb_get_name = self.MAPTK_LIB.maptk_config_block_get_name
        cb_get_name.argtypes = [self.C_TYPE_PTR]
        cb_get_name.restype = ctypes.c_char_p
        return cb_get_name(self._inst_ptr)

    def subblock(self, key):
        """
        Get a new MaptkConfigBlock that is deep copy of this MaptkConfigBlock
        starting at the given key.

        :return: New MaptkConfigBlock
        :rtype: MaptkConfigBlock

        """
        cb_subblock = self.MAPTK_LIB.maptk_config_block_subblock
        cb_subblock.argtypes = [self.C_TYPE_PTR]
        cb_subblock.restype = self.C_TYPE_PTR
        return MaptkConfigBlock.from_c_pointer(
            cb_subblock(self._inst_ptr, key)
        )

    def subblock_view(self, key):
        """
        Get a new MaptkConfigBlock that is a view into this MaptkConfigBlock
        starting at the given key.

        Modifications on the returned MaptkConfigBlock are also reflected in
        this instance.

        :return: New MaptkConfigBlock instance with this instance as its parent.
        :rtype: MaptkConfigBlock

        """
        cb_subblock_view = self.MAPTK_LIB.maptk_config_block_subblock_view
        cb_subblock_view.argtypes = [self.C_TYPE_PTR]
        cb_subblock_view.restype = self.C_TYPE_PTR
        return MaptkConfigBlock.from_c_pointer(
            cb_subblock_view(self._inst_ptr, key)
        )

    def get_value(self, key, default=None):
        """ Get the string value for a key.

        The given key may not exist in this configuration. If no default was
        given, then None is returned. Otherwise, the provided default value is
        returned.

        :param key: The index of the configuration value to retrieve.
        :type key: str

        :param default: Optional default value that will be returned if the
            given is not found in this configuration.
        :type default: str

        :return: The string value stored within the configuration
        :rtype: str

        """
        # Not using the get_value_default C API call since the string version
        # getter handily returns a None value (yay ctypes), providing a simpler
        # code path than calling the C API default func.
        cb_get_value = self.MAPTK_LIB.maptk_config_block_get_value
        cb_get_value.argtypes = [self.C_TYPE_PTR, ctypes.c_char_p]
        cb_get_value.restype = ctypes.c_char_p
        r = cb_get_value(self._inst_ptr, key)
        if r is None:
            return default
        return r

    def get_description(self, key):
        """
        Get the string description for a given key.

        If the provided key exists but has no description associated with it, an
        empty string is returned.

        If the key provided does not exist, a None is returned.

        :param key: The name of the parameter to get the description of.
        :type key: str

        :returns: The string description of the give key or None if the key was
                  not found.
        :rtype: str or None

        """
        cb_get_descr = self.MAPTK_LIB.maptk_config_block_get_description
        cb_get_descr.argtypes = [self.C_TYPE_PTR, ctypes.c_char_p]
        cb_get_descr.restype = ctypes.c_char_p
        return cb_get_descr(self._inst_ptr, key)

    def set_value(self, key, value, description=None):
        """
        Set a string value within the configuration.

        If the provided key has been marked as read-only, nothing is set.

        If this key already exists, has a description and no new description
        was passed with this call, the previous description is retained. We
        assume that the previous description is still valid and this a value
        overwrite. If it is intended for the description to also be overwritten,
        an unset_value call should be performed on the key first, and then this
        set_value call.

        :param key: The index of the configuration value to set.
        :type key: str

        :param value: The value to set for the \p key.

        :param description: Optional description to associate with the key.
        :type description: str

        """
        if description is None:
            cb_set_value = self.MAPTK_LIB.maptk_config_block_set_value
            cb_set_value.argtypes = [self.C_TYPE_PTR, ctypes.c_char_p,
                                     ctypes.c_char_p]
            cb_set_value(self._inst_ptr, str(key), str(value))
        else:
            cb_set_value = self.MAPTK_LIB.maptk_config_block_set_value_descr
            cb_set_value.argtypes = [self.C_TYPE_PTR, ctypes.c_char_p,
                                     ctypes.c_char_p, ctypes.c_char_p]
            cb_set_value(self._inst_ptr, str(key), str(value), str(description))

    def unset_value(self, key):
        """
        Remove a value from the configuration

        If the provided key has been marked as read-only, nothing is unset.

        :param key: The index of the configuration value to unset
        :type key: str

        """
        cb_unset_value = self.MAPTK_LIB.maptk_config_block_unset_value
        cb_unset_value.argtypes = [self.C_TYPE_PTR]
        cb_unset_value(self._inst_ptr, key)

    def is_read_only(self, key):
        """
        Query if a value is read-only.

        :param key: Key to check
        :type key: str

        :return: True if the given key has been marked as read-only, and false
            otherwise
        :rtype: bool

        """
        cb_is_ro = self.MAPTK_LIB.maptk_config_block_is_read_only
        cb_is_ro.argtypes = [self.C_TYPE_PTR, ctypes.c_char_p]
        cb_is_ro.restype = ctypes.c_bool
        return cb_is_ro(self._inst_ptr, key)

    def mark_read_only(self, key):
        """
        Mark the given key as read-only

        This provided key is marked as read-only even if it doesn't currently
        exist in the given config_block instance.

        :param key: The key to mark as read-only.
        :type key: str

        """
        cb_mark_ro = self.MAPTK_LIB.maptk_config_block_mark_read_only
        cb_mark_ro.argtypes = [self.C_TYPE_PTR, ctypes.c_char_p]
        cb_mark_ro(self._inst_ptr, key)

    def merge_config(self, other):
        """
        Merge the values in given MaptkConfigBlock into the this instance.

        NOTE: Any values currently set within \c *this will be overwritten if
        conflicts occur.

        :param other: Other MaptkConfigBlock instance whose key/value pairs are
            to be merged into this instance.
        :type other: MaptkConfigBlock

        """
        cb_merge = self.MAPTK_LIB.maptk_config_block_merge_config
        cb_merge.argtypes = [self.C_TYPE_PTR, self.C_TYPE_PTR]
        cb_merge(self._inst_ptr, other._inst_ptr)

    def has_value(self, key):
        """
        Check if a value exists for key in this configuration

        :param key: The index of the configuration value to check.
        :type key: str

        :returns: Whether the key exists or not
        :rtype: bool

        """
        cb_has_value = self.MAPTK_LIB.maptk_config_block_has_value
        cb_has_value.argtypes = [self.C_TYPE_PTR, ctypes.c_char_p]
        cb_has_value.restype = ctypes.c_bool
        return cb_has_value(self._inst_ptr, key)

    def available_keys(self):
        """
        Return a list of available keys in this configuration instance.
        """
        cb_ak = self.MAPTK_LIB.maptk_config_block_available_values
        sl_free = self.MAPTK_LIB.maptk_common_free_string_list

        cb_ak.argtypes = [self.C_TYPE_PTR, ctypes.POINTER(ctypes.c_uint),
                          ctypes.POINTER(ctypes.POINTER(ctypes.c_char_p))]
        sl_free.argtypes = [ctypes.c_uint, ctypes.POINTER(ctypes.c_char_p)]

        length = ctypes.c_uint(0)
        keys = ctypes.POINTER(ctypes.c_char_p)()
        cb_ak(self._inst_ptr, ctypes.byref(length), ctypes.byref(keys))

        # Constructing return array
        r = []
        for i in xrange(length.value):
            r.append(keys[i])

        # Free allocated key listing
        sl_free(length, keys)

        return r

    def read(self, filepath):
        """
        Update this configuration instance with the configuration at the given
        filepath.

        :param filepath: Path to a configuration file
        :type filepath: str

        """
        cb = MaptkConfigBlock.from_file(filepath)
        self.merge_config(cb)

    def write(self, filepath):
        """
        Output this configuration to the specified file path

        :raises MaptkBaseException: Error occurred in writing configuration to
            the given file path.

        :param filepath: Output file path.

        """
        cb_write = self.MAPTK_LIB.maptk_config_block_file_write
        cb_write.argtypes = [self.C_TYPE_PTR, ctypes.c_char_p,
                             self.EH_TYPE_PTR]

        eh = self.EH_NEW()
        cb_write(self._inst_ptr, filepath, eh)

        try:
            propagate_exception_from_handle(eh, {
                -1: MaptkConfigBlockIoException,
                1: MaptkConfigBlockIoFileWriteException
            })
        finally:
            self.EH_NEW(eh)
