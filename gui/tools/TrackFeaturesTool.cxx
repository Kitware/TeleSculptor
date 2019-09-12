/*ckwg +29
 * Copyright 2017-2018 by Kitware, Inc.
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

#include <maptk/colorize.h>
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

namespace
{

// uniformly sample max_num items from in_data and add to out_data
template <typename T>
void uniform_subsample(std::vector<T> const& in_data,
                       std::vector<T>& out_data, size_t max_num)
{
  const size_t data_size = in_data.size();
  if (data_size <= max_num)
  {
    // append all the data
    out_data.insert(out_data.end(), in_data.begin(), in_data.end());
    return;
  }

  // select max_num distributed throughout the vector
  for (unsigned i = 0; i < max_num; ++i)
  {
    size_t idx = (i * data_size) / max_num;
    out_data.push_back(in_data[idx]);
  }
}

// select which frames to use for tracking
std::vector<kwiver::vital::frame_id_t>
select_frames(std::vector<kwiver::vital::frame_id_t> const& valid_frames,
              std::vector<kwiver::vital::frame_id_t> const& camera_frames,
              size_t max_frames)
{
  std::vector<kwiver::vital::frame_id_t> selected_frames;
  std::vector<kwiver::vital::frame_id_t> remaining_valid_frames;
  // if initial cameras are available, give these frames priority
  if (!camera_frames.empty())
  {
    // intersect with valid frames to make sure camera frames are valid
    std::vector<kwiver::vital::frame_id_t> valid_camera_frames;
    std::set_intersection(camera_frames.begin(), camera_frames.end(),
                          valid_frames.begin(), valid_frames.end(),
                          std::back_inserter(valid_camera_frames));

    // take max_frames uniformily out of camera_frames
    uniform_subsample(valid_camera_frames, selected_frames, max_frames);

    // remove selected frames from valid frames
    std::set_difference(valid_frames.begin(), valid_frames.end(),
                        selected_frames.begin(), selected_frames.end(),
                        std::back_inserter(remaining_valid_frames));
  }
  else
  {
    remaining_valid_frames = valid_frames;
  }

  if (selected_frames.size() < max_frames &&
      !remaining_valid_frames.empty())
  {
    const size_t num_needed = max_frames - selected_frames.size();
    uniform_subsample(remaining_valid_frames, selected_frames, num_needed);
    std::sort(selected_frames.begin(), selected_frames.end());
  }

  return selected_frames;
}

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

  std::vector<kwiver::vital::frame_id_t> camera_frames;
  if (this->hasCameras())
  {
    for (auto const& cam : this->cameras()->cameras())
    {
      camera_frames.push_back(cam.first);
    }
  }

  std::vector<kwiver::vital::frame_id_t> selected_frames =
    select_frames(valid_frames, camera_frames, d->max_frames);

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
    if (target_frame < start_frame)
    {
      continue;
    }
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
      tracks = kwiver::maptk::extract_feature_colors(tracks, *image, frame);
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
