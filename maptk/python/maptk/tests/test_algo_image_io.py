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

Tests for maptk::algo::image_io

"""
# -*- coding: utf-8 -*-
__author__ = 'purg'

from maptk import (
    AlgorithmPluginManager,
    ConfigBlock,
)
from maptk.algo import ImageIo
from maptk.tests import TEST_DATA_DIR

import nose.tools as nt
import os
import os.path as osp
import tempfile


class TestMaptkAlgoImageIo (object):

    @classmethod
    def setup_class(cls):
        AlgorithmPluginManager.register_plugins()

        cls.test_image_filepath = osp.join(TEST_DATA_DIR,
                                           'test_kitware_logo.jpg')

    def test_image_load_save_diff(self):
        fd, tmp_filename = tempfile.mkstemp()

        c = ConfigBlock()
        c.set_value('iio:type', 'vxl')
        iio = ImageIo('iio')
        iio.set_config(c)

        nt.assert_true(osp.isfile(self.test_image_filepath),
                       "Couldn't find image file")
        ic_orig = iio.load(self.test_image_filepath)
        iio.save(ic_orig, tmp_filename)
        ic_test = iio.load(tmp_filename)

        nt.assert_equal(ic_orig.size(), ic_test.size())
        nt.assert_equal(ic_orig.width(), ic_test.width())
        nt.assert_equal(ic_orig.height(), ic_test.height())
        nt.assert_equal(ic_orig.depth(), ic_test.depth())

        os.remove(tmp_filename)
        os.close(fd)
