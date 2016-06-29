#!/usr/bin/env python
"""
ckwg +31
Copyright 2016 by Kitware, Inc.
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

Python equivalent of the C++ MapTK tool

"""
import argparse
import errno
import itertools
import logging
import os
import sys

from vital import ConfigBlock
from vital.apm import register_plugins_once
from vital.algo import ImageIo, ConvertImage, TrackFeatures
from vital.types import TrackSet


def parse_args():
    parser = argparse.ArgumentParser()

    g_config = parser.add_argument_group("Configuration")
    g_config.add_argument('-c', '--config',
                          metavar='PATH',
                          help='Path to the configuration file')
    g_config.add_argument('-g', '--generate-config',
                          metavar='PATH',
                          help='Generate a configuration file and save it at '
                               'this location.')

    return parser.parse_args()


def base_algorithms(c=ConfigBlock()):
    """
    Determine the algorithms and algorithm names used in this utility
    :type c: vital.ConfigBlock
    """
    iio = ImageIo('image_reader')
    iio.set_config(c)

    ci = ConvertImage('image_converter')
    ci.set_config(c)

    tf = TrackFeatures('track_features')
    tf.set_config(c)

    return iio, ci, tf


def default_config_block():
    c = ConfigBlock()

    c.set_value('image_list_file',
                '',
                'Path to an input file containing new-line separated paths to '
                'sequential image files')
    c.set_value('mask_list_file',
                '',
                'Optional path to an input file containing new-line '
                'separated paths to mask images. This list should be '
                'parallel in association to files specified in '
                '``image_list_file``. Mask image must be the same size as '
                'the image they are associated with.\n'
                '\n'
                'Leave this blank if no image masking is desired.')
    c.set_value('output_tracks_file',
                '',
                'Path to a file to write output tracks to. If this file '
                'exists, it will be overwritten.')

    image_reader, image_converter, feature_tracker = base_algorithms()
    image_reader.get_config(c)
    image_converter.get_config(c)
    feature_tracker.get_config(c)

    return c


def check_config(c):
    """
    :param c: ConfigBlock to check
    :return: vital.ConfigBlock
    """
    log = logging.getLogger()

    algo_check = True
    for a in base_algorithms():
        algo_check &= a.check_config(c)

    ilist_file = c.get_value('image_list_file')
    ilist_valid = True
    if not (ilist_file and os.path.exists(ilist_file) and not os.path.isdir(ilist_file)):
        log.warn("Invalid image list filepath: '%s'", ilist_file)
        ilist_valid = False

    mlist_file = c.get_value('mask_list_file')
    mlist_valid = True
    if not mlist_file or (os.path.isfile(mlist_file) and not os.path.isdir(mlist_file)):
        log.warn("Invalid image list filepath: '%s'", ilist_file)
        mlist_valid = False

    return algo_check and ilist_valid and mlist_valid


def safe_create_dir(d):
    """
    Recursively create the given directory, ignoring the already-exists
    error if thrown.

    :param d: Directory filepath to create
    :type d: str

    :return: The directory that was created, i.e. the directory that was passed
        (in absolute form).
    :rtype: str

    """
    d = os.path.abspath(os.path.expanduser(d))
    try:
        os.makedirs(d)
    except OSError, ex:
        if ex.errno == errno.EEXIST and os.path.exists(d):
            pass
        else:
            raise
    return d


def main():
    # Same logging format as C++, except for ',' separating milliseconds
    logging.basicConfig(format="%(asctime)s %(levelname)s %(filename)s(%(lineno)s): %(message)s")
    logging.getLogger().setLevel(logging.INFO)
    log = logging.getLogger(__name__)

    args = parse_args()

    register_plugins_once()
    c = default_config_block()

    # Read in config file if given
    if args.config:
        c.read(args.config)

    # Set algorithm configurations + update config-block
    image_reader, image_converter, feature_tracker = base_algorithms()
    image_reader.set_config(c)
    image_reader.get_config(c)
    image_converter.set_config(c)
    image_converter.get_config(c)
    feature_tracker.set_config(c)
    feature_tracker.get_config(c)

    if args.generate_config:
        check_config(c)
        c.write(args.generate_config)
        log.info("Generated configuration: %s", args.generate_config)
        sys.exit(0)

    if not check_config(c):
        raise RuntimeError("Configuration invalid. "
                           "Check above warning messages.")

    image_list_fp = c.get_value('image_list_file')
    mask_list_fp = c.get_value('mask_list_file')
    output_tracks_fp = c.get_value('output_tracks_file')

    # Read in image files, and optional mask files
    # iterate over track features function
    track_set = TrackSet()
    with open(image_list_fp) as f:
        frame_list = [l.strip() for l in f]
    if mask_list_fp:
        with open(mask_list_fp) as f:
            mask_list = [l.strip() for l in f]
    else:
        mask_list = [None] * len(frame_list)
    assert len(frame_list) == len(mask_list), \
        "Image and mask list is not congruent in size (%d images != %d " \
        "masks)" % (len(frame_list), len(mask_list))

    # Track over frames + masks
    for i, (f, m) in enumerate(itertools.izip(frame_list, mask_list)):
        frame = image_converter.convert(image_reader.load(f))
        mask = (m and image_converter.convert(image_reader.load(m))) or None
        feature_tracker.track(track_set, i, frame, mask)

    safe_create_dir(os.path.dirname(output_tracks_fp))
    track_set.write_tracks_file(output_tracks_fp)


if __name__ == '__main__':
    main()
