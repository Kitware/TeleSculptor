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

import ctypes

from maptk.util import find_maptk_library


class ConfigBlock (object):
    """
    MAPTK config_block class
    """
    MAPTK_LIB = find_maptk_library()

    # Assigned later when the subclass is defined
    __FROM_PTR_SUBCLASS__ = None

    def __init__(self, name=None):
        if name:
            self._cb_p = self.MAPTK_LIB.maptk_config_block_new_named(str(name))
        else:
            self._cb_p = self.MAPTK_LIB.maptk_config_block_new()

    def __del__(self):
        print "Destroying CB: \"%s\" %d" % (self.name, self._cb_p)
        self.MAPTK_LIB.maptk_config_block_destroy(self._cb_p)

    @property
    def name(self):
        """
        :return: The name assigned to this ConfigBlock instance.
        :rtype: str
        """
        self.MAPTK_LIB.maptk_config_block_get_name.restype = ctypes.c_char_p
        return self.MAPTK_LIB.maptk_config_block_get_name(self._cb_p)

    def subblock(self, key):
        """
        Get a new ConfigBlock that is deep copy of this ConfigBlock starting
        at the given key.

        :return: New ConfigBlock
        :rtype: ConfigBlock

        """
        return self.__FROM_PTR_SUBCLASS__(
            self.MAPTK_LIB.maptk_config_block_subblock(self._cb_p, key)
        )

    def subblock_view(self, key):
        """
        Get a new ConfigBlock that is a view into this ConfigBlock starting at
        the given key.

        Modifications on the returned ConfigBlock are also reflected in this
        instance.

        :return: New ConfigBlock instance with this instance as its parent.
        :rtype: ConfigBlock

        """
        return self.__FROM_PTR_SUBCLASS__(
            self.MAPTK_LIB.maptk_config_block_subblock_view(self._cb_p, key)
        )


class __cb_from_pointer (ConfigBlock):
    """
    Utility class to construct
    """

    # noinspection PyMissingConstructor
    def __init__(self, ptr):
        """
        Purposely not calling super constructor so we can manually assign the
        pointer value.
        """
        if ptr:
            self._cb_p = ptr
        else:
            raise RuntimeError("Attempted construction of ConfigBlock instance "
                               "from a NULL pointer (ptr = %s)." % ptr)


ConfigBlock.__FROM_PTR_SUBCLASS__ = __cb_from_pointer
