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

Tests for maptk::algo::convert_image and general algorithm tests

"""
# -*- coding: utf-8 -*-
__author__ = 'purg'

import ctypes

from maptk import (
    AlgorithmPluginManager,
    ConfigBlock,
    Image,
    ImageContainer,
)
from maptk.algo import ConvertImage
from maptk.exceptions.base import MaptkNullPointerException

import nose.tools as nt


def mem_address(inst_ptr):
    return int(bool(inst_ptr)) and ctypes.addressof(inst_ptr.contents)


class TestMaptkAlgoConvertImage (object):
    """
    Doubles as the tests for generic algorithm and algorithm_def methods as this
    is an algorithm with a core implementation.
    """

    @classmethod
    def setup_class(cls):
        AlgorithmPluginManager.register_plugins()

    def test_from_c_ptr_null(self):
        ci = ConvertImage.from_c_pointer(
            ConvertImage.C_TYPE_PTR(),
            name='ci'
        )
        nt.assert_false(ci.c_pointer)

    def test_from_c_ptr_no_name(self):
        nt.assert_raises(
            ValueError,
            ConvertImage.from_c_pointer,
            ConvertImage.C_TYPE_PTR(),
        )

    def test_from_c_ptr_copy(self):
        ci = ConvertImage('ci')
        ci_new = ConvertImage.from_c_pointer(ci.c_pointer, ci)
        nt.assert_is(ci.c_pointer, ci_new.c_pointer)
        nt.assert_equal(ci_new.name, ci.name)

    def test_registered_names(self):
        nt.assert_in('bypass', ConvertImage.registered_names())

    def test_create(self):
        ci = ConvertImage.create('ci', 'bypass')
        nt.assert_equal(ci.impl_name(), 'bypass')

    def test_create_invalid(self):
        nt.assert_raises(
            MaptkNullPointerException,
            ConvertImage.create,
            'ci', 'notAnImpl'
        )

    def test_new(self):
        ci = ConvertImage('ci')
        nt.assert_false(ci)
        nt.assert_false(ci.c_pointer)

    def test_typename(self):
        ci = ConvertImage('ci')
        nt.assert_equal(ci.type_name(), "convert_image")

    def test_name(self):
        name = 'algo_name'
        ci = ConvertImage(name)
        nt.assert_equal(ci.name, name)

    def test_set_name(self):
        algo_name = 'ci'
        other_name = "other_name"

        ci = ConvertImage(algo_name)
        nt.assert_equal(ci.name, algo_name)
        ci.set_name(other_name)
        nt.assert_not_equal(ci.name, algo_name)
        nt.assert_equal(ci.name, other_name)

    def test_impl_name(self):
        ci_empty = ConvertImage('ci')
        nt.assert_is_none(ci_empty.impl_name())
        ci_bypass = ConvertImage.create('ci', 'bypass')
        nt.assert_equal(ci_bypass.impl_name(), 'bypass')

    def test_clone_empty(self):
        ci_empty = ConvertImage('ci')
        ci_empty2 = ci_empty.clone()
        nt.assert_false(ci_empty)
        nt.assert_false(ci_empty2)

    def test_clone(self):
        ci1 = ConvertImage.create('ci', 'bypass')
        ci2 = ci1.clone()
        nt.assert_true(ci1)
        nt.assert_true(ci2)
        nt.assert_not_equal(ci1.c_pointer, ci2.c_pointer)
        nt.assert_not_equal(mem_address(ci1.c_pointer),
                                    mem_address(ci2.c_pointer))

    def test_get_conf(self):
        ci = ConvertImage('ci')
        c = ci.get_config()
        nt.assert_list_equal(c.available_keys(), ['ci:type'])
        nt.assert_equal(c.get_value('ci:type'), '')

        ci = ConvertImage.create('ci', 'bypass')
        c = ci.get_config()
        nt.assert_equal(c.get_value('ci:type'), 'bypass')

    def test_set_conf(self):
        ci = ConvertImage('ci')
        nt.assert_false(ci)
        nt.assert_is_none(ci.impl_name())

        c = ConfigBlock()
        c.set_value('ci:type', 'bypass')
        ci.set_config(c)
        nt.assert_true(ci)
        nt.assert_equal(ci.impl_name(), 'bypass')

    def test_check_conf(self):
        ci = ConvertImage('ci')
        c = ConfigBlock()
        nt.assert_false(ci.check_config(c))

        c.set_value('ci:type', '')
        nt.assert_false(ci.check_config(c))

        c.set_value('ci:type', 'not_an_impl')
        nt.assert_false(ci.check_config(c))

        c.set_value('ci:type', 'bypass')
        nt.assert_true(ci.check_config(c))

    def test_convert(self):
        ic1 = ImageContainer(Image())
        ci = ConvertImage.create('ci', 'bypass')
        ic2 = ci.convert(ic1)

        nt.assert_not_equal(ic1, ic2)
        nt.assert_not_equal(ic1.c_pointer, ic2.c_pointer)
        nt.assert_equal(hex(mem_address(ic1.c_pointer)),
                        hex(mem_address(ic2.c_pointer)))

    # def test_image_load_save_diff(self):
    #     fd, filename = tempfile.mkstemp()
    #
    #     c = ConfigBlock()
    #     c.set_value('iio:type', 'vxl')
    #     iio = ImageIo('iio')
    #     iio.set_config(c)
