#!/usr/bin/env python
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

MAPTK Python version of track features tool, primarily for example Python
interface usage.

"""
# -*- coding: utf-8 -*-
__author__ = 'purg'

import logging
import os
import os.path

from maptk import (
    AlgorithmPluginManager,
    ConfigBlock,
    TrackSet
)
from maptk.algo import (
    ConvertImage,
    ImageIo,
    TrackFeatures
)


logging.basicConfig()


def get_default_config():
    c = ConfigBlock("track_features")
    c.set_value("image_list_file", "",
                "Path to an input file containing new-line separated paths "
                "to sequential image files")
    c.set_value("mask_list_file", "",
                "Optional path to an input file containing new-line "
                "separated paths to mask images. This list should be "
                "parallel in association to files specified in "
                "``image_list_file``. Mask image must be the same size as "
                "the image they are associated with.\n"
                "\n"
                "Leave this blank if no image masking is desired.")
    c.set_value("invert_masks", "false",
                "If true, all mask images will be inverted after loading. "
                "This is useful if mask images read in use positive "
                "values to indicated masked areas instead of non-masked "
                "areas.")
    c.set_value("expect_multichannel_masks", "false",
                "A majority of the time, mask images are a single channel, "
                "however it is feasibly possible that certain "
                "implementations may use multi-channel masks. If this is "
                "true we will expect multiple-channel mask images, "
                "warning when a single-channel mask is provided. If this "
                "is false we error upon seeing a multi-channel mask "
                "image.")
    c.set_value("output_tracks_file", "",
                "Path to a file to write output tracks to. If this "
                "file exists, it will be overwritten.")
    # Required algorithm does not have an implemented interface yet
    # c.set_value("output_homography_file", "",
    #             "Optional path to a file to write source-to-reference "
    #             "homographies for each frame. Leave blank to disable this "
    #             "output. The output_homography_generator algorithm type "
    #             "only needs to be set if this is set.")
    return c


class TrackFeaturesTool (object):

    @property
    def log(self):
        return logging.getLogger("TrackFeaturesTool")

    def __init__(self):
        AlgorithmPluginManager.register_plugins()

        # Algorithms
        self.algo_convert_img = ConvertImage("convert_image")
        self.algo_image_io = ImageIo("image_reader")
        self.algo_track_features = TrackFeatures("feature_tracker")

        # Other tool variables
        self.image_list_filepath = None
        self.mask_list_filepath = None
        self.invert_masks = False
        self.expect_multichannel_masks = False
        self.output_tracks_filepath = None

    def get_configuration(self):
        """
        :return: the current tool configuration
        :rtype: ConfigBlock
        """
        c = ConfigBlock("track_features")
        c.set_value("image_list_file",
                    self.image_list_filepath or "",
                    "Path to an input file containing new-line separated paths "
                    "to sequential image files")
        c.set_value("mask_list_file",
                    self.mask_list_filepath or "",
                    "Optional path to an input file containing new-line "
                    "separated paths to mask images. This list should be "
                    "parallel in association to files specified in "
                    "``image_list_file``. Mask image must be the same size as "
                    "the image they are associated with.\n"
                    "\n"
                    "Leave this blank if no image masking is desired.")
        c.set_value("invert_masks",
                    str(self.invert_masks).lower(),
                    "If true, all mask images will be inverted after loading. "
                    "This is useful if mask images read in use positive "
                    "values to indicated masked areas instead of non-masked "
                    "areas.")
        c.set_value("expect_multichannel_masks",
                    str(self.expect_multichannel_masks).lower(),
                    "A majority of the time, mask images are a single channel, "
                    "however it is feasibly possible that certain "
                    "implementations may use multi-channel masks. If this is "
                    "true we will expect multiple-channel mask images, "
                    "warning when a single-channel mask is provided. If this "
                    "is false we error upon seeing a multi-channel mask "
                    "image.")
        c.set_value("output_tracks_file",
                    self.output_tracks_filepath or "",
                    "Path to a file to write output tracks to. If this "
                    "file exists, it will be overwritten.")
        # Required algorithm does not have an implemented interface yet
        # c.set_value("output_homography_file",
        #             self.output_homography_filepath or "",
        #             "Optional path to a file to write source-to-reference "
        #             "homographies for each frame. Leave blank to disable this "
        #             "output. The output_homography_generator algorithm type "
        #             "only needs to be set if this is set.")

        self.algo_convert_img.get_config(c)
        self.algo_image_io.get_config(c)
        self.algo_track_features.get_config(c)

        return c

    def set_configuration(self, config):
        """
        Set the tool configuration
        :param config: The configuration to set
        :type config: ConfigBlock
        """
        self.algo_convert_img.set_config(config)
        self.algo_image_io.set_config(config)
        self.algo_track_features.set_config(config)

        abspath = lambda p: os.path.abspath(os.path.expanduser(p))

        self.image_list_filepath = config.get_value('image_list_file',
                                                    self.image_list_filepath)
        if self.image_list_filepath:
            self.image_list_filepath = abspath(self.image_list_filepath)
        self.mask_list_filepath = config.get_value('mask_list_file',
                                                   self.mask_list_filepath)
        if self.mask_list_filepath:
            self.mask_list_filepath = abspath(self.mask_list_filepath)
        self.invert_masks = config.get_value_bool('invert_masks', self.invert_masks)
        self.expect_multichannel_masks = \
            config.get_value_bool('expect_multichannel_masks',
                            self.expect_multichannel_masks)
        self.output_tracks_filepath = \
            config.get_value('output_tracks_file', self.output_tracks_filepath)
        if self.output_tracks_filepath:
            self.output_tracks_filepath = abspath(self.output_tracks_filepath)

    def validate_configuration(self):
        """
        Check the current configuration for validity
        :return: True if valid, false if not.
        :type: bool
        """
        if not self.image_list_filepath:
            self.log.error("No image list file provided")
            return False
        if not os.path.isfile(self.image_list_filepath):
            self.log.error("Path given for image list does not refer to a file")
            return False
        if not self.output_tracks_filepath:
            self.log.error("No output tracks file privided")
            return False
        if self.mask_list_filepath and not os.path.isfile(self.mask_list_filepath):
            self.log.error("Specified a mask list filepath, but path did not "
                           "refer to an existing file.")
            return False

        return (
            # Algorithms should check out valid
            self.algo_convert_img.check_config(self.algo_convert_img.get_config())
            and self.algo_image_io.check_config(self.algo_image_io.get_config())
            and self.algo_track_features.check_config(self.algo_track_features.get_config())
        )

    def track_images(self):
        # Create list of input filepaths
        self.log.info("Reading input image list file...")
        with open(self.image_list_filepath, 'r') as image_list_file:
            input_filepaths = \
                [line.strip() for line in image_list_file.readlines()]

        # Create a list of mask images if we were given a list file
        mask_filepaths = None
        if self.mask_list_filepath:
            self.log.info("Reading mask image list file...")
            with open(self.mask_list_filepath) as mask_list_file:
                mask_filepaths = \
                    [line.strip() for line in mask_list_file.readlines()]

            if len(input_filepaths) != len(mask_filepaths):
                self.log.error("Input and Mask image lists were not congruent "
                               "in size.")
                return False

        # Check that the output tracks file is open-able and that the containing
        # directory exists.
        if not os.path.isdir(os.path.dirname(self.output_tracks_filepath)):
            self.log.info("Creating containing directory for output tracks "
                          "file: %s",
                          os.path.dirname(self.output_tracks_filepath))
            os.mkdir(os.path.dirname(self.output_tracks_filepath))
        self.log.info("Testing that output tracks file is open-able...")
        test_f = open(self.output_tracks_filepath, 'w')
        test_f.close()

        tracks = TrackSet()
        for frame_num in xrange(len(input_filepaths)):
            input_img = self.algo_convert_img.convert(
                self.algo_image_io.load(input_filepaths[frame_num])
            )
            mask_img = None
            if mask_filepaths:
                mask_img = self.algo_convert_img.convert(
                    self.algo_image_io.load(mask_filepaths[frame_num])
                )
            self.log.info("Processing frame %d...", frame_num)
            tracks = self.algo_track_features.track(tracks, frame_num,
                                                    input_img, mask_img)

        self.log.info("Frame processing complete, writing out track set...")
        tracks.write_tracks_file(self.output_tracks_filepath)


def cli_main():
    """
    Returns:
        0 - Success
        1 - Configuration invalid
        2 - Tracking failed
    """
    import optparse

    logging.getLogger().setLevel(logging.INFO)
    log = logging.getLogger("cli_main")

    parser = optparse.OptionParser()
    parser.add_option("-c", "--config",
                      help="Path to the configuration file to use.")
    parser.add_option("-o", "--output-config",
                      help="Output a configuration file for the current "
                           "configuration to the specified file path.")
    opts, args = parser.parse_args()

    if opts.config:
        opts.config = os.path.abspath(os.path.expanduser(opts.config))
    if opts.output_config:
        opts.output_config = os.path.abspath(os.path.expanduser(opts.output_config))

    tft = TrackFeaturesTool()

    if opts.config:
        log.info("Setting configuration file: %s", opts.config)
        tft.set_configuration(ConfigBlock.from_file(opts.config))

    if opts.output_config:
        log.info("Writing output configuration file: %s", opts.output_config)
        tft.get_configuration().write(opts.output_config)
        if not tft.validate_configuration():
            log.warning("Current configuration insufficient for running the "
                        "tool")
        return 0

    # Error out if current configuration not valid
    if not tft.validate_configuration():
        log.error("Current configuration insufficient for running the tool")
        return 1

    if tft.track_images():
        return 0
    else:
        return 2


if __name__ == "__main__":
    exit(cli_main())
