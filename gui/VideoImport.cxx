/*ckwg +29
 * Copyright 2018-2019 by Kitware, Inc.
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

#include <qtStlUtil.h>

#include <QFileInfo>

#include <atomic>

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
    // Needs its own copy of config since parameters will changed.
    config = kwiver::vital::config_block::empty_config();
    canceled = false;
  }

  video_input_sptr video_reader;
  config_block_sptr config;
  std::string videoPath;

  kwiver::vital::local_geo_cs localGeoCs;

  kwiver::vital::logger_handle_t logger;

  std::atomic<bool> canceled;
};

//-----------------------------------------------------------------------------
VideoImport::VideoImport(QObject* parent)
  : QThread{parent}, d_ptr{new VideoImportPrivate{}}
{}

//-----------------------------------------------------------------------------
VideoImport::~VideoImport()
{
  QTE_D();

  this->wait();
  if (d->video_reader)
  {
    d->video_reader->close();
  }
}

//-----------------------------------------------------------------------------
void VideoImport::setData(config_block_sptr const& config,
                     std::string const& path,
                     kwiver::vital::local_geo_cs& lgcs)
{
  QTE_D();

  d->config->merge_config(config);

  d->videoPath = path;
  d->localGeoCs = lgcs;
}

//-----------------------------------------------------------------------------
void VideoImport::run()
{
  QTE_D();

  d->canceled = false;

  try
  {

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

    auto metadataMap =
      std::make_shared<kwiver::vital::metadata_map::map_metadata_t>();

    // If no metadata stream, exit early
    if (!d->video_reader->get_implementation_capabilities()
      .has_capability(kwiver::vital::algo::video_input::HAS_METADATA))
    {
      emit this->completed(metadataMap);
      d->video_reader->close();
      return;
    }

    auto num_frames = static_cast<int>(d->video_reader->num_frames());

    QString description = QString("&Scanning metadata in %1 (Frame %2). "
      "Cancel to ignore metadata.")
      .arg(QFileInfo{ qtString(d->videoPath) }.fileName());
    while (d->video_reader->next_frame(currentTimestamp) && !d->canceled)
    {
      auto frame = currentTimestamp.get_frame();
      auto mdVec = d->video_reader->frame_metadata();

      if (mdVec.size() > 0)
      {
        metadataMap->emplace(frame, mdVec);
      }

      QString desc = description.arg(frame);
      emit this->progressChanged(desc, frame * 100 / num_frames);
    }

    if (d->canceled)
    {
      // invalidate the metadata map
      metadataMap = nullptr;
    }

    emit this->progressChanged(QString("Loading video complete"), 100);
    emit this->completed(metadataMap);
  }
  catch (kwiver::vital::vital_exception const&)
  {
    emit this->progressChanged(QString("Loading Failed"), 100);
    emit this->completed({});
  }
  d->video_reader->close();
}

//-----------------------------------------------------------------------------
void VideoImport::cancel()
{
  QTE_D();

  d->canceled = true;
}
