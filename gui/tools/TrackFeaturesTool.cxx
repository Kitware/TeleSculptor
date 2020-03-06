/*ckwg +29
 * Copyright 2017-2020 by Kitware, Inc.
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

#include "TrackFeaturesTool.h"
#include "GuiCommon.h"

#include <arrows/core/colorize.h>
#include <maptk/version.h>

#include <vital/algo/image_io.h>
#include <vital/algo/convert_image.h>
#include <vital/algo/track_features.h>
#include <vital/algo/video_input.h>

#include <vital/types/metadata.h>
#include <vital/types/metadata_traits.h>

#include <algorithm>

#include <QMessageBox>

using kwiver::vital::algo::image_io;
using kwiver::vital::algo::convert_image;
using kwiver::vital::algo::convert_image_sptr;
using kwiver::vital::algo::track_features;
using kwiver::vital::algo::track_features_sptr;
using kwiver::vital::algo::video_input;
using kwiver::vital::algo::video_input_sptr;

namespace
{
static char const* const BLOCK_CI = "image_converter";
static char const* const BLOCK_TF = "feature_tracker";
static char const* const BLOCK_VR = "video_reader";
static char const* const BLOCK_MR = "mask_reader";
}

//-----------------------------------------------------------------------------
class TrackFeaturesToolPrivate
{
public:
  kwiver::vital::image_container_sptr getImage(
    video_input_sptr const& reader, bool readerIsValid = true) const;

  convert_image_sptr image_converter;
  track_features_sptr feature_tracker;
  video_input_sptr video_reader;
  video_input_sptr mask_reader;
  unsigned int max_frames = 500;
};

QTE_IMPLEMENT_D_FUNC(TrackFeaturesTool)

//-----------------------------------------------------------------------------
kwiver::vital::image_container_sptr TrackFeaturesToolPrivate::getImage(
  video_input_sptr const& reader, bool readerIsValid) const
{
  if (!readerIsValid)
  {
    return {};
  }

  auto const image = reader->frame_image();
  return this->image_converter->convert(image);
}

//-----------------------------------------------------------------------------
TrackFeaturesTool::TrackFeaturesTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new TrackFeaturesToolPrivate)
{
  this->setText("&Track Features");
  this->setToolTip(
    "<nobr>Detect feature points in the images, compute feature descriptors, "
    "</nobr>and match the features across images.  Also run loop closure if "
    "configured to do so.");
}

//-----------------------------------------------------------------------------
TrackFeaturesTool::~TrackFeaturesTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs TrackFeaturesTool::outputs() const
{
  return Tracks | ActiveFrame;
}

//-----------------------------------------------------------------------------
bool TrackFeaturesTool::execute(QWidget* window)
{
  QTE_D();

  if (!this->hasVideoSource())
  {
    QMessageBox::information(
      window, "Insufficient data", "This operation requires a video source.");
    return false;
  }

  // Merge project config with default config file
  auto const config = readConfig("gui_track_features.conf");

  // Check configuration
  if (!config)
  {
    QMessageBox::critical(
      window, "Configuration error",
      "No configuration data was found. Please check your installation.");
    return false;
  }

  auto const hasMask = !this->data()->maskPath.empty();
  config->merge_config(this->data()->config);
  if (!convert_image::check_nested_algo_configuration(BLOCK_CI, config) ||
      !track_features::check_nested_algo_configuration(BLOCK_TF, config) ||
      !video_input::check_nested_algo_configuration(BLOCK_VR, config) ||
      (hasMask && !video_input::check_nested_algo_configuration(BLOCK_MR, config)))

  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the algorithm configuration.");
    return false;
  }

  // Create algorithm from configuration
  convert_image::set_nested_algo_configuration(BLOCK_CI, config, d->image_converter);
  track_features::set_nested_algo_configuration(BLOCK_TF, config, d->feature_tracker);
  video_input::set_nested_algo_configuration(BLOCK_VR, config, d->video_reader);
  video_input::set_nested_algo_configuration(BLOCK_MR, config, d->mask_reader);

  // The maximum number of frames to track
  d->max_frames = config->get_value<unsigned int>("feature_tracker:max_frames",
                                                  d->max_frames);

  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
void TrackFeaturesTool::run()
{
  QTE_D();

  auto const maxFrame = this->data()->maxFrame;
  auto const hasMask = !this->data()->maskPath.empty();

  kwiver::vital::metadata_map_sptr md_map;
  // get metadata from the tool, if possible, so we do not
  // need to scan the whole video again
  if (this->sfmConstraints())
  {
    md_map = this->sfmConstraints()->get_metadata();
  }

  auto tracks = this->tracks();
  kwiver::vital::frame_id_t start_frame = this->activeFrame();
  kwiver::vital::timestamp currentTimestamp;

  d->video_reader->open(this->data()->videoPath);
  if (hasMask)
  {
    d->mask_reader->open(this->data()->maskPath);
  }

  if (!md_map)
  {
    md_map = d->video_reader->metadata_map();
  }

  std::vector<kwiver::vital::frame_id_t> valid_frames;
  if (md_map)
  {
    auto fs = md_map->frames();
    valid_frames = std::vector<kwiver::vital::frame_id_t>(fs.begin(), fs.end());
  }

  const size_t num_frames = valid_frames.size();
  std::vector<kwiver::vital::frame_id_t> selected_frames;
  if (num_frames > d->max_frames)
  {
    // select max_frames distributed throughout the video
    for (unsigned i = 0; i < d->max_frames; ++i)
    {
      size_t idx = (i * num_frames) / d->max_frames;
      if (valid_frames[idx] >= start_frame)
      {
        selected_frames.push_back(valid_frames[idx]);
      }
    }
  }
  else
  {
    selected_frames = valid_frames;
  }

  // seek to the valid frame before the current frame
  // so that the first advance will bring us to the start frame
  auto frame_itr = std::lower_bound(valid_frames.begin(),
                                    valid_frames.end(), start_frame);
  if (frame_itr != valid_frames.end() &&
      frame_itr != valid_frames.begin())
  {
    auto sf = *(frame_itr - 1);
    d->video_reader->seek_frame(currentTimestamp, sf);
    if (hasMask)
    {
      kwiver::vital::timestamp dummyTimestamp;
      d->mask_reader->seek_frame(dummyTimestamp, sf);
    }
  }

  this->updateProgress(static_cast<int>(start_frame), maxFrame);

  for (auto target_frame : selected_frames)
  {
    bool valid = true;
    kwiver::vital::frame_id_t frame = 0;

    // step to find the next target frame
    do
    {
      valid = d->video_reader->next_frame(currentTimestamp);
      if (hasMask)
      {
        kwiver::vital::timestamp dummyTimestamp;
        valid = valid && d->mask_reader->next_frame(dummyTimestamp);
      }
      frame = currentTimestamp.get_frame();
    } while (valid && frame < target_frame);
    if (!valid)
    {
      break;
    }

    auto const image = d->getImage(d->video_reader);
    auto const mask = d->getImage(d->mask_reader, hasMask);

    auto const mdv = d->video_reader->frame_metadata();
    if (!mdv.empty())
    {
      image->set_metadata(mdv[0]);
    }

    // Update tool progress
    this->updateProgress(static_cast<int>(frame), maxFrame);

    tracks = d->feature_tracker->track(tracks, frame, image, mask);
    if (tracks)
    {
      tracks = kwiver::arrows::core::extract_feature_colors(tracks, *image, frame);
    }

    // make a copy of the tool data
    auto data = std::make_shared<ToolData>();
    data->copyTracks(tracks);
    data->activeFrame = frame;
    data->progress = progress();
    data->description = description().toStdString();

    emit updated(data);
    if( this->isCanceled() )
    {
      break;
    }
  }
  d->video_reader->close();
  if (hasMask)
  {
    d->mask_reader->close();
  }
  this->updateTracks(tracks);
  this->setActiveFrame(start_frame);
  // mark progress 100% complete
  this->updateProgress(100);
}
