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

Tests for the Python interface to MAPTK class config_block.

"""
# -*- coding: utf-8 -*-
__author__ = 'purg'

from maptk import ConfigBlock
from maptk.exceptions.config_block import *
from maptk.exceptions.config_block_io import *

import nose.tools


# noinspection PyMethodMayBeStatic
class TestMaptkConfigBlock (object):
    """
    Python version of test_config_block.cxx
    """

    def test_block_set_size(self):
        nose.tools.assert_equal(len(ConfigBlock.BLOCK_SEP), 1)

    def test_construction(self):
        cb1 = ConfigBlock()
        nose.tools.assert_true(cb1._inst_ptr, "Received null pointer from "
                                              "ConfigBlock construction")
        cb2 = ConfigBlock("A Name")
        nose.tools.assert_true(cb2._inst_ptr, "Received a null pointer "
                                              "from named ConfigBlock "
                                              "construction.")

    def test_name_access(self):
        cb = ConfigBlock()
        nose.tools.assert_equal(cb.name, "",
                                "Default constructor should have an empty "
                                "string name.")
        cb = ConfigBlock("FooBar")
        nose.tools.assert_equal(cb.name, "FooBar",
                                "Named constructor didn't seem to retain name")

    def test_set_value(self):
        cb = ConfigBlock()
        # Basic value string
        cb.set_value('foo', 'bar')
        # Should attempt casting non-string to string
        cb.set_value('bar', 124789)
        # Setting with a description
        cb.set_value('baz', 'a', "This is a description")

    def test_has_value(self):
        cb = ConfigBlock()

        cb.set_value('foo', 'bar')
        cb.set_value('bar', 124789)
        cb.set_value('baz', 'a', "This is a description")

        nose.tools.assert_true(cb.has_value("foo"))
        nose.tools.assert_true(cb.has_value('bar'))
        nose.tools.assert_true(cb.has_value('baz'))

        nose.tools.assert_false(cb.has_value('a'))
        nose.tools.assert_false(cb.has_value('not a value'))

    def test_get_value(self):
        cb = ConfigBlock()

        cb.set_value('a', 'b')
        nose.tools.assert_equal(cb.get_value('a'), 'b')

    def test_get_value_bool(self):
        cb = ConfigBlock()

        cb.set_value('a', 'true')
        nose.tools.assert_true(cb.get_value_bool('a'))

        cb.set_value('b', 'false')
        nose.tools.assert_false(cb.get_value_bool('b'))

        cb.set_value('a', 'yes')
        nose.tools.assert_true(cb.get_value_bool('a'))

        cb.set_value('b', 'no')
        nose.tools.assert_false(cb.get_value_bool('b'))

    def test_get_value_bool_default(self):
        cb = ConfigBlock()

        nose.tools.assert_raises(
            MaptkConfigBlockNoSuchValueException,
            cb.get_value_bool, 'not-a-key'
        )

        nose.tools.assert_true(
            cb.get_value_bool('not-a-key', True)
        )
        nose.tools.assert_false(
            cb.get_value_bool('not-a-key', False)
        )

    def test_get_value_nested(self):
        cb = ConfigBlock()

        k1 = 'a'
        k2 = 'b'
        v = 'c'

        cb.set_value(k1 + ConfigBlock.BLOCK_SEP + k2, v)
        nose.tools.assert_equal(cb.get_value(k1 + ConfigBlock.BLOCK_SEP + k2),
                                v)

        sb = cb.subblock(k1)
        nose.tools.assert_equal(sb.get_value(k2), v)

    def test_get_value_no_exist(self):
        cb = ConfigBlock()
        k1 = 'a'
        k2 = 'b'
        v2 = '2'

        nose.tools.assert_is_none(cb.get_value(k1))
        nose.tools.assert_equal(cb.get_value(k2, v2), v2)

    def test_unset_value(self):
        cb = ConfigBlock()

        cb.set_value('a', '1')
        cb.set_value('b', '2')

        cb.unset_value('a')

        nose.tools.assert_false(cb.has_value('a'))
        nose.tools.assert_is_none(cb.get_value('a'))

        nose.tools.assert_equal(cb.get_value('b'), '2')
        nose.tools.assert_true(cb.has_value('b'))

    def test_available_keys(self):
        cb = ConfigBlock()
        cb.set_value("foo", 1)
        cb.set_value('bar', 'baz')
        r = cb.available_keys()
        nose.tools.assert_set_equal(set(r),
                                    {'foo', 'bar'},
                                    "Returned key list was incorrect.")

    def test_read_only(self):
        cb = ConfigBlock()

        cb.set_value('a', '1')
        cb.mark_read_only('a')
        nose.tools.assert_equal(cb.get_value('a'), '1')
        cb.set_value('a', '2')
        nose.tools.assert_equal(cb.get_value('a'), '1')

    def test_read_only_unset(self):
        cb = ConfigBlock()
        cb.set_value('a', '1')
        cb.mark_read_only('a')
        cb.unset_value('a')
        nose.tools.assert_true(cb.has_value('a'))
        nose.tools.assert_equal(cb.get_value('a'), '1')

    def test_subblock(self):
        cb = ConfigBlock()

        block_name = 'block'
        other_name = 'other_block'
        ka = 'keya'
        kb = 'keyb'
        kc = 'keyc'
        va = 'va'
        vb = 'vb'
        vc = 'vc'

        cb.set_value(block_name + ConfigBlock.BLOCK_SEP + ka, va)
        cb.set_value(block_name + ConfigBlock.BLOCK_SEP + kb, vb)
        cb.set_value(other_name + ConfigBlock.BLOCK_SEP + kc, vc)

        sb = cb.subblock(block_name)

        nose.tools.assert_true(sb.has_value(ka))
        nose.tools.assert_equal(sb.get_value(ka), va)
        nose.tools.assert_true(sb.has_value(kb))
        nose.tools.assert_equal(sb.get_value(kb), vb)
        nose.tools.assert_false(sb.has_value(kc))

    def test_subblock_nested(self):
        cb = ConfigBlock()

        block_name = 'block'
        other_name = 'other'
        nestd_name = block_name + ConfigBlock.BLOCK_SEP + other_name

        ka = 'ka'
        kb = 'kb'
        va = 'va'
        vb = 'vb'

        cb.set_value(nestd_name + ConfigBlock.BLOCK_SEP + ka, va)
        cb.set_value(nestd_name + ConfigBlock.BLOCK_SEP + kb, vb)

        sb = cb.subblock(nestd_name)

        nose.tools.assert_true(sb.has_value(ka))
        nose.tools.assert_equal(sb.get_value(ka), va)
        nose.tools.assert_true(sb.has_value(kb))
        nose.tools.assert_equal(sb.get_value(kb), vb)

    def test_subblock_match(self):
        cb = ConfigBlock()

        b_name = 'block'
        va = 'va'

        cb.set_value(b_name, va)
        sb = cb.subblock(b_name)
        keys = sb.available_keys()
        nose.tools.assert_equal(len(keys), 0)

    def test_subblock_prefix_match(self):
        cb = ConfigBlock()

        b_name = 'block'
        ka = 'ka'
        va = 'va'

        # intentionally not adding block separator
        cb.set_value(b_name + ka, va)
        sb = cb.subblock(b_name)
        keys = sb.available_keys()
        nose.tools.assert_equal(len(keys), 0)

    def test_subblock_view(self):
        cb = ConfigBlock()

        b_name = 'block'
        o_name = 'other_block'
        ka = 'ka'
        kb = 'kb'
        kc = 'kc'
        va = 'va'
        vb = 'vb'
        vc = 'vc'

        cb.set_value(b_name + ConfigBlock.BLOCK_SEP + ka, va)
        cb.set_value(b_name + ConfigBlock.BLOCK_SEP + kb, vb)
        cb.set_value(o_name + ConfigBlock.BLOCK_SEP + kc, vc)
        sb = cb.subblock_view(b_name)

        nose.tools.assert_true(sb.has_value(ka))
        nose.tools.assert_false(sb.has_value(kc))

        cb.set_value(b_name + ConfigBlock.BLOCK_SEP + ka, vb)
        nose.tools.assert_equal(sb.get_value(ka), vb)
        sb.set_value(ka, va)
        nose.tools.assert_equal(cb.get_value(b_name + ConfigBlock.BLOCK_SEP + ka), va)

        sb.unset_value(kb)
        nose.tools.assert_false(cb.has_value(b_name + ConfigBlock.BLOCK_SEP + kb))

        cb.set_value(b_name + ConfigBlock.BLOCK_SEP + kc, vc)
        sb_keys = sb.available_keys()
        nose.tools.assert_set_equal(set(sb_keys), {ka, kc})

    def test_subblock_view_nested(self):
        cb = ConfigBlock()

        b_name = 'block'
        o_name = 'other_block'
        n_name = b_name + ConfigBlock.BLOCK_SEP + o_name
        ka = 'ka'
        kb = 'kb'
        kc = 'kc'
        va = 'va'
        vb = 'vb'
        vc = 'vc'

        cb.set_value(n_name + ConfigBlock.BLOCK_SEP + ka, va)
        cb.set_value(n_name + ConfigBlock.BLOCK_SEP + kb, vb)
        cb.set_value(o_name + ConfigBlock.BLOCK_SEP + kc, vc)
        sb = cb.subblock_view(n_name)

        nose.tools.assert_true(sb.has_value(ka))
        nose.tools.assert_false(sb.has_value(kc))

        cb.set_value(n_name + ConfigBlock.BLOCK_SEP + ka, vb)
        nose.tools.assert_equal(sb.get_value(ka), vb)
        sb.set_value(ka, va)
        nose.tools.assert_equal(cb.get_value(n_name + ConfigBlock.BLOCK_SEP + ka), va)

        sb.unset_value(kb)
        nose.tools.assert_false(cb.has_value(n_name + ConfigBlock.BLOCK_SEP + kb))

        cb.set_value(n_name + ConfigBlock.BLOCK_SEP + kc, vc)
        sb_keys = sb.available_keys()
        nose.tools.assert_set_equal(set(sb_keys), {ka, kc})

    def test_subblock_view_match(self):
        cb = ConfigBlock()
        bname = 'block'
        va = 'va'
        cb.set_value(bname, va)
        sb = cb.subblock_view(bname)
        keys = sb.available_keys()
        nose.tools.assert_equal(len(keys), 0)

    def test_subblock_view_prefix_match(self):
        cb = ConfigBlock()
        bname = 'block'
        ka = 'ka'
        va = 'va'
        # intentionally not adding block separator
        cb.set_value(bname + ka, va)
        sb = cb.subblock_view(bname)
        keys = sb.available_keys()
        nose.tools.assert_equal(len(keys), 0)

    def test_merge_config(self):
        cb1 = ConfigBlock()
        cb2 = ConfigBlock()
        ka = 'ka'
        kb = 'kb'
        kc = 'kc'
        va = 'va'
        vb = 'vb'
        vc = 'vc'

        cb1.set_value(ka, va)
        cb1.set_value(kb, va)
        cb2.set_value(kb, vb)
        cb2.set_value(kc, vc)

        cb1.merge_config(cb2)

        nose.tools.assert_equal(cb1.get_value(ka), va)
        nose.tools.assert_equal(cb1.get_value(kb), vb)
        nose.tools.assert_equal(cb1.get_value(kc), vc)

    def test_set_value_description(self):
        cb = ConfigBlock()
        bname = 'sub'
        ka = 'ka'
        kb = 'kb'
        kc = 'kc'
        va = 'va'
        vb = 'vb'
        vc = 'vc'
        da = 'da'
        db = 'db'

        cb.set_value(ka, va, da)
        cb.set_value(bname + ConfigBlock.BLOCK_SEP + kb, vb, db)
        cb.set_value(kc, vc)
        sb = cb.subblock('sub')

        nose.tools.assert_equal(cb.get_description(ka), da)
        nose.tools.assert_equal(sb.get_description(kb), db)
        nose.tools.assert_equal(cb.get_description(kc), "")

    def test_read_no_file(self):
        nose.tools.assert_raises(MaptkConfigBlockIoFileNotFoundException,
                                 ConfigBlock.from_file,
                                 'not-a-file.foobar')

    def test_write_fail(self):
        cb = ConfigBlock()
        cb.set_value('foo', 'bar')

        nose.tools.assert_raises(MaptkConfigBlockIoException,
                                 cb.write,
                                 '/not/valid')
