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

Tests for Python interface to maptk::track_set

"""
# -*- coding: utf-8 -*-
__author__ = 'purg'

from maptk import Track
from maptk import TrackSet

import nose.tools as nt


class TestMaptkTrackSet (object):

    def test_new(self):
        ts = TrackSet()
        nt.assert_true(ts, "Invalid track set instance constructed")

    def test_empty_len_size(self):
        ts = TrackSet()
        nt.assert_true(ts, "Invalid track set instance constructed")
        l = len(ts)
        s = ts.size()
        nt.assert_equal(l, 0)
        nt.assert_equal(l, s)

    def test_new_nonempty(self):
        n = 10
        tracks = [Track() for _ in xrange(n)]
        ts = TrackSet(tracks)
        nt.assert_true(ts, "Invalid track set instance constructed")
        nt.assert_equal(len(ts), n)
        nt.assert_equal(ts.size(), n)
