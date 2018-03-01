/*ckwg +29
 * Copyright 2017 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name Kitware, Inc. nor the names of any contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "VideoImport.h"

#include <vital/algo/video_input.h>

using kwiver::vital::algo::video_input;
using kwiver::vital::algo::video_input_sptr;
using kwiver::vital::config_block_sptr;

namespace
{
static char const* const BLOCK_VR = "video_reader";
}

QTE_IMPLEMENT_D_FUNC(VideoImport)

//-----------------------------------------------------------------------------
class VideoImportPrivate
{
public:
  VideoImportPrivate() {
    logger = kwiver::vital::get_logger("telesculptor.video_input");
  }

  video_input_sptr video_reader;
  config_block_sptr config;
  std::string videoPath;

  kwiver::vital::logger_handle_t logger;
};

//-----------------------------------------------------------------------------
VideoImport::VideoImport(config_block_sptr const& config, std::string const& path)
  : d_ptr(new VideoImportPrivate())
{
  QTE_D();

  d->config = config;
  d->videoPath = path;
  // connect(d, SIGNAL(finished()), this, SIGNAL(completed()));
}

//-----------------------------------------------------------------------------
VideoImport::~VideoImport()
{
  QTE_D();

  d->video_reader->close();
}

//-----------------------------------------------------------------------------
void VideoImport::run()
{
  QTE_D();

  if (!video_input::check_nested_algo_configuration(
    BLOCK_VR, d->config))
  {
    LOG_WARN(d->logger,
      "An error was found in the video source algorithm configuration.");
    return;
  }

  video_input::set_nested_algo_configuration(
    BLOCK_VR, d->config, d->video_reader);

  kwiver::vital::timestamp currentTimestamp;
  d->video_reader->open(d->videoPath);

  while (d->video_reader->next_frame(currentTimestamp))
  {
    VideoData vd;
    vd.frame = currentTimestamp.get_frame();
    vd.mdVec = d->video_reader->frame_metadata();

    std::cout << "FRAME: vd.frame = " << vd.frame << std::endl;
  }

  d->video_reader->close();
}
