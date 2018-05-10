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

#include "TrackFeaturesSprokitTool.h"
#include "GuiCommon.h"

#include <fstream>
#include <sstream>
#include <time.h>

#include <vital/algo/convert_image.h>
#include <vital/algo/video_input.h>
#include <vital/algo/keyframe_selection.h>
#include <vital/algo/track_features.h>
#include <vital/algo/close_loops.h>

#include <vital/config/config_block_io.h>
#include <vital/types/metadata.h>

#include <qtStlUtil.h>
#include <QMessageBox>

#include <sprokit/pipeline/pipeline.h>
#include <sprokit/processes/kwiver_type_traits.h>
#include <sprokit/processes/adapters/embedded_pipeline.h>
#include <sprokit/pipeline_util/literal_pipeline.h>
#include <arrows/core/track_set_impl.h>


using kwiver::vital::algo::convert_image;
using kwiver::vital::algo::convert_image_sptr;
using kwiver::vital::algo::video_input;
using kwiver::vital::algo::video_input_sptr;
using kwiver::vital::algo::keyframe_selection;
using kwiver::vital::algo::keyframe_selection_sptr;
using kwiver::vital::algo::track_features;
using kwiver::vital::algo::track_features_sptr;
using kwiver::vital::algo::close_loops;
using kwiver::vital::algo::close_loops_sptr;

namespace
{
  static char const* const BLOCK_CI = "image_converter";
  static char const* const BLOCK_VR = "video_reader";
  static char const* const BLOCK_KS = "keyframe_selector";
  static char const* const BLOCK_DF = "detect_features";
  static char const* const BLOCK_CL = "close_loops";
}  // end anonymous namespace


//-----------------------------------------------------------------------------
class TrackFeaturesSprokitToolPrivate
{
public:
  TrackFeaturesSprokitToolPrivate();

  convert_image_sptr image_converter;
  video_input_sptr video_reader;
  kwiver::embedded_pipeline ep;

  keyframe_selection_sptr m_keyframe_selection;
  track_features_sptr m_detect_if_keyframe;
  close_loops_sptr m_loop_closer;

};

TrackFeaturesSprokitToolPrivate
::TrackFeaturesSprokitToolPrivate()
{
}

QTE_IMPLEMENT_D_FUNC(TrackFeaturesSprokitTool)

//-----------------------------------------------------------------------------
TrackFeaturesSprokitTool::TrackFeaturesSprokitTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new TrackFeaturesSprokitToolPrivate)
{
  this->setText("&Track Features");
  this->setToolTip(
    "<nobr>Track features through the video and identify keyframes. "
    "</nobr>Compute descriptors on the keyframes and match to a visual "
    "index to close loops.");
}

//-----------------------------------------------------------------------------
TrackFeaturesSprokitTool::~TrackFeaturesSprokitTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs TrackFeaturesSprokitTool::outputs() const
{
  return Tracks | ActiveFrame;
}

//-----------------------------------------------------------------------------
bool TrackFeaturesSprokitTool::execute(QWidget* window)
{
  QTE_D();

  if (!this->hasVideoSource())
  {
    QMessageBox::information(
      window, "Insufficient data", "This operation requires a video source.");
    return false;
  }

  // Load configuration
  auto const config = readConfig("gui_track_features_sprokit.conf");

  // Check configuration
  if (!config)
  {
    QMessageBox::critical(
      window, "Configuration error",
      "No configuration data was found. Please check your installation.");
    return false;
  }

  config->merge_config(this->data()->config);
  if (!convert_image::check_nested_algo_configuration(BLOCK_CI, config) ||
      !video_input::check_nested_algo_configuration(BLOCK_VR, config)   ||
      !keyframe_selection::check_nested_algo_configuration(BLOCK_KS,config) ||
      !track_features::check_nested_algo_configuration(BLOCK_DF, config) ||
      !close_loops::check_nested_algo_configuration(BLOCK_CL, config) )
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the algorithm configuration.");
    return false;
  }

  // Create algorithm from configuration
  convert_image::set_nested_algo_configuration(BLOCK_CI, config, d->image_converter);
  video_input::set_nested_algo_configuration(BLOCK_VR, config, d->video_reader);
  keyframe_selection::set_nested_algo_configuration(BLOCK_KS, config, d->m_keyframe_selection);
  track_features::set_nested_algo_configuration(BLOCK_DF, config, d->m_detect_if_keyframe);
  close_loops::set_nested_algo_configuration(BLOCK_CL, config, d->m_loop_closer);

  std::stringstream pipe_str(create_pipeline_config(window));
  if (pipe_str.str().empty())
  {
    return false;
  }

  // create an embedded pipeline
  try
  {
    d->ep.build_pipeline(pipe_str);
  }
  catch (sprokit::pipeline_exception const& e)
  {
    QMessageBox::critical(
      window, "Configuration error",
      QString("Error parsing Sprokit pipeline.\n") + e.what());
    return false;
  }

  return AbstractTool::execute(window);
}

std::string
TrackFeaturesSprokitTool
::create_pipeline_config(QWidget* window)
{
  std::stringstream ss;

  std::string pipe_file = findConfig("track_features_embedded.pipe");
  // Load from a pipeline description file if found
  if (!pipe_file.empty())
  {
    // Open pipeline description
    std::ifstream pipe_str;
    pipe_str.open(pipe_file, std::ifstream::in);

    if (!pipe_str)
    {
      QMessageBox::critical(
        window, "Configuration error",
        QString::fromStdString("Unable to open file: " + pipe_file));
      return "";
    }
    ss << pipe_str.rdbuf();
    return ss.str();
  }
  else
  {
    // find the path to the vocabulary file
    std::string voc_path = findConfig("kwiver_voc.yml.gz");
    if (voc_path.empty())
    {
      QMessageBox::critical(
        window, "Configuration error",
        "No vocabulary data was found. Please check your installation.");
      return "";
    }

    ss << SPROKIT_PROCESS("input_adapter", "input")

      << SPROKIT_PROCESS("feature_tracker", "tracker")
      << SPROKIT_CONFIG("track_features:type", "ocv_KLT")
      << SPROKIT_CONFIG("track_features:ocv_KLT:new_feat_exclusionary_radius_image_fraction", "0.02")
      << SPROKIT_CONFIG("track_features:ocv_KLT:redetect_frac_lost_threshold", "0.7")
      << SPROKIT_CONFIG("track_features:ocv_KLT:feature_detector:type", "ocv_FAST")
      << SPROKIT_CONFIG("track_features:ocv_KLT:feature_detector:ocv_FAST:threshold", "50")
      << SPROKIT_CONFIG("track_features:ocv_KLT:feature_detector:ocv_FAST:nonmaxSuppression", "true")

      << SPROKIT_PROCESS("output_adapter", "output")
      << SPROKIT_CONFIG_BLOCK("_pipeline:_edge")
      << SPROKIT_CONFIG("capacity", "2")

      << SPROKIT_CONNECT("input", "image", "tracker", "image")
      << SPROKIT_CONNECT("input", "timestamp", "tracker", "timestamp")
      << SPROKIT_CONNECT("tracker", "feature_track_set", "tracker", "feature_track_set")
      << SPROKIT_CONNECT("tracker", "feature_track_set", "output", "klt_frame_track_set")
      ;
  }

  return ss.str();
}

//-----------------------------------------------------------------------------
void
TrackFeaturesSprokitTool
::run()
{
  QTE_D();

  auto const maxFrame = this->data()->maxFrame;

  // Start pipeline and wait for it to finish
  d->ep.start();

  const kwiver::vital::frame_id_t frame = this->activeFrame();
  kwiver::vital::timestamp currentTimestamp;

  d->video_reader->open(this->data()->videoPath);

  // Seek to just before active frame TODO: check status?
  if (frame > 1)
  {
    d->video_reader->seek_frame(currentTimestamp, frame - 1);
  }

  this->updateProgress(static_cast<int>(frame), maxFrame);
  this->setDescription("Parsing video frames");

  typedef std::unique_ptr<kwiver::vital::track_set_implementation> tsi_uptr;

  kwiver::vital::feature_track_set_sptr accumulated_tracks =
    std::make_shared<kwiver::vital::feature_track_set>(
    tsi_uptr(new kwiver::arrows::core::frame_index_track_set_impl()));

  double disp_period = 2;
  time_t last_disp_time;
  time(&last_disp_time);

  while (d->video_reader->next_frame(currentTimestamp))
  {
    auto const image = d->video_reader->frame_image();
    auto const converted_image = d->image_converter->convert(image);

    // Update tool progress
    this->updateProgress(
      static_cast<int>(currentTimestamp.get_frame()), maxFrame);

    auto const mdv = d->video_reader->frame_metadata();
    if (!mdv.empty())
    {
      converted_image->set_metadata(mdv[0]);
    }

    // Create dataset for input
    auto ds = kwiver::adapter::adapter_data_set::create();
    ds->add_value("image", converted_image);
    ds->add_value("timestamp", currentTimestamp);
    d->ep.send(ds);

    if (this->isCanceled())
    {
      d->video_reader->close();
      break;
    }
    if (!d->ep.empty())
    {
      auto rds = d->ep.receive();
      auto ix = rds->find("klt_frame_track_set");
      if( ix != rds->end())
      {
        auto klt_frame_tracks = ix->second->get_datum<kwiver::vital::feature_track_set_sptr>();
        //we have klt frames from current frame, yipee
        accumulated_tracks->merge_in_other_track_set(klt_frame_tracks);

        time_t cur_time;
        time(&cur_time);
        double seconds_since_last_disp = difftime(cur_time, last_disp_time);
        if (seconds_since_last_disp > disp_period)
        {
          last_disp_time = cur_time;
          auto data = std::make_shared<ToolData>();
          data->copyTracks(accumulated_tracks);
          data->activeFrame = *(accumulated_tracks->all_frame_ids().rbegin());
          emit updated(data);
        }
      }
    }
  }
  d->ep.send_end_of_input();

  while (!d->ep.at_end())
  {
    auto rds = d->ep.receive();
    auto ix = rds->find("klt_frame_track_set");
    if (ix != rds->end())
    {
      auto klt_frame_tracks = ix->second->get_datum<kwiver::vital::feature_track_set_sptr>();
      accumulated_tracks->merge_in_other_track_set(klt_frame_tracks);

      time_t cur_time;
      time(&cur_time);
      double seconds_since_last_disp = difftime(cur_time, last_disp_time);
      if (seconds_since_last_disp > disp_period)
      {
        last_disp_time = cur_time;
        auto data = std::make_shared<ToolData>();
        data->copyTracks(accumulated_tracks);
        data->activeFrame = *(accumulated_tracks->all_frame_ids().rbegin());
        emit updated(data);
      }
    }
  }
  d->ep.wait();

  //select the keyframes
  auto kf_tracks = std::static_pointer_cast<kwiver::vital::feature_track_set>(d->m_keyframe_selection->select(accumulated_tracks));

  //do the feature extraction on keyframes
  auto matchable_tracks = kf_tracks;

  d->video_reader->close();
  d->video_reader->open(this->data()->videoPath);

  if (frame > 1)
  {
    d->video_reader->seek_frame(currentTimestamp, frame - 1);
  }

  while (d->video_reader->next_frame(currentTimestamp))
  {
    auto const image = d->video_reader->frame_image();
    auto const converted_image = d->image_converter->convert(image);

    matchable_tracks = std::static_pointer_cast<kwiver::vital::feature_track_set>(d->m_detect_if_keyframe->track(matchable_tracks,currentTimestamp.get_frame(),converted_image));
    time_t cur_time;
    time(&cur_time);
    double seconds_since_last_disp = difftime(cur_time, last_disp_time);
    if (seconds_since_last_disp > disp_period)
    {
      last_disp_time = cur_time;
      auto data = std::make_shared<ToolData>();
      data->copyTracks(matchable_tracks);
      data->activeFrame = currentTimestamp.get_frame();
      emit updated(data);
    }
  }

  auto keyframes = matchable_tracks->keyframes();
  auto loop_detected_tracks = matchable_tracks;
  for (auto fid : keyframes)
  {
    loop_detected_tracks = d->m_loop_closer->stitch(fid, loop_detected_tracks, kwiver::vital::image_container_sptr());

    time_t cur_time;
    time(&cur_time);
    double seconds_since_last_disp = difftime(cur_time, last_disp_time);
    if (seconds_since_last_disp > disp_period)
    {
      last_disp_time = cur_time;
      auto data = std::make_shared<ToolData>();
      data->copyTracks(loop_detected_tracks);
      data->activeFrame = fid;
      emit updated(data);
    }
  }

  d->video_reader->close();
  this->updateTracks(loop_detected_tracks);
  this->setActiveFrame(frame);

}
