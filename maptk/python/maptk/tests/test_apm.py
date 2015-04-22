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

Tests for maptk::algorithm_plugin_manager interface

"""
# -*- coding: utf-8 -*-
__author__ = 'purg'

from maptk import AlgorithmPluginManager

import nose.tools


# noinspection PyPep8Naming
class Test_AlgorithmPluginManager (object):

    def test_load(self):
        AlgorithmPluginManager.register_plugins()

    def test_load_named(self):
        AlgorithmPluginManager.register_plugins("maptk_core")

    def test_add_search_path(self):
        AlgorithmPluginManager.add_search_path("/")
        self.test_load_named()

    def test_add_search_path_bad_dir(self):
        AlgorithmPluginManager.add_search_path("/probably/not/a/directory/foo")
        self.test_load_named()

    def test_registered_names(self):
        AlgorithmPluginManager.register_plugins('maptk_core')
        nose.tools.assert_in('maptk_core',
                             AlgorithmPluginManager.registered_module_names())
