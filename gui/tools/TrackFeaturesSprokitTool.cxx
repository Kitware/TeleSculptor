// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "TrackFeaturesSprokitTool.h"
#include "GuiCommon.h"

#include <fstream>
#include <sstream>

#include <vital/algo/algorithm.txx>
#include <vital/algo/convert_image.h>
#include <vital/algo/video_input.h>

#include <vital/config/config_block_io.h>
#include <vital/types/metadata.h>

#include <qtStlUtil.h>
#include <QMessageBox>

#include <sprokit/pipeline/pipeline.h>
#include <sprokit/processes/kwiver_type_traits.h>
#include <sprokit/processes/adapters/embedded_pipeline.h>
#include <sprokit/pipeline_util/literal_pipeline.h>

using kwiver::vital::algo::convert_image;
using kwiver::vital::algo::convert_image_sptr;
using kwiver::vital::algo::video_input;
using kwiver::vital::algo::video_input_sptr;

namespace
{
  static char const* const BLOCK_CI = "image_converter";
  static char const* const BLOCK_VR = "video_reader";
}  // end anonymous namespace

//-----------------------------------------------------------------------------
class TrackFeaturesSprokitToolPrivate
{
public:
  TrackFeaturesSprokitToolPrivate();

  convert_image_sptr image_converter;
  video_input_sptr video_reader;
  kwiver::embedded_pipeline ep;
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
  this->setText("&Track Features Streaming");
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
  auto const config = readConfig("gui_track_features.conf");

  // Check configuration
  if (!config)
  {
    QMessageBox::critical(
      window, "Configuration error",
      "No configuration data was found. Please check your installation.");
    return false;
  }

  config->merge_config(this->data()->config);
  if (!kwiver::vital::check_nested_algo_configuration<convert_image>(BLOCK_CI, config) ||
      !kwiver::vital::check_nested_algo_configuration<video_input>(BLOCK_VR, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the algorithm configuration.");
    return false;
  }

  // Create algorithm from configuration
  kwiver::vital::set_nested_algo_configuration<convert_image>(BLOCK_CI, config, d->image_converter);
  kwiver::vital::set_nested_algo_configuration<video_input>(BLOCK_VR, config, d->video_reader);

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

      << SPROKIT_PROCESS("keyframe_selection_process", "keyframes")
      << SPROKIT_CONFIG("keyframe_selection_1:type", "basic")
      << SPROKIT_CONFIG("keyframe_selection_1:basic:fraction_tracks_lost_to_necessitate_new_keyframe", "0.1")

      << SPROKIT_PROCESS("detect_features_if_keyframe_process", "detect_if_keyframe")
      << SPROKIT_CONFIG("augment_keyframes:type","augment_keyframes")
      << SPROKIT_CONFIG("augment_keyframes:augment_keyframes:kf_only_feature_detector:type", "ocv_ORB")
      << SPROKIT_CONFIG("augment_keyframes:augment_keyframes:kf_only_descriptor_extractor:type", "ocv_ORB")
      << SPROKIT_CONFIG("augment_keyframes:augment_keyframes:kf_only_feature_detector:ocv_ORB:n_features", "2000")

      << SPROKIT_PROCESS("close_loops_process", "loop_detector")
      << SPROKIT_CONFIG("close_loops:type", "appearance_indexed")
      << SPROKIT_CONFIG("close_loops:appearance_indexed:min_loop_inlier_matches", "50")
      << SPROKIT_CONFIG("close_loops:appearance_indexed:match_features:type", "homography_guided")
      << SPROKIT_CONFIG("close_loops:appearance_indexed:match_features:homography_guided:homography_estimator:type", "ocv")
      << SPROKIT_CONFIG("close_loops:appearance_indexed:match_features:homography_guided:feature_matcher1:type", "ocv_brute_force")
      << SPROKIT_CONFIG("close_loops:appearance_indexed:bag_of_words_matching:type", "dbow2")
      << SPROKIT_CONFIG("close_loops:appearance_indexed:bag_of_words_matching:dbow2:feature_detector:type", "ocv_ORB")
      << SPROKIT_CONFIG("close_loops:appearance_indexed:bag_of_words_matching:dbow2:descriptor_extractor:type", "ocv_ORB")
      << SPROKIT_CONFIG("close_loops:appearance_indexed:bag_of_words_matching:dbow2:image_io:type", "ocv")
      << SPROKIT_CONFIG("close_loops:appearance_indexed:bag_of_words_matching:dbow2:max_num_candidate_matches_from_vocabulary_tree", "20")
      << SPROKIT_CONFIG("close_loops:appearance_indexed:bag_of_words_matching:dbow2:training_image_list_path", "")
      << SPROKIT_CONFIG("close_loops:appearance_indexed:bag_of_words_matching:dbow2:vocabulary_path", voc_path)

      << SPROKIT_PROCESS("output_adapter", "output")
      << SPROKIT_CONFIG_BLOCK("_pipeline:_edge")
      << SPROKIT_CONFIG("capacity", "2")

      << SPROKIT_CONNECT("input", "image", "tracker", "image")
      << SPROKIT_CONNECT("input", "timestamp", "tracker", "timestamp")
      << SPROKIT_CONNECT("tracker", "feature_track_set", "tracker", "feature_track_set")
      << SPROKIT_CONNECT("tracker", "feature_track_set", "keyframes", "next_tracks")
      << SPROKIT_CONNECT("input", "timestamp", "keyframes", "timestamp")
      << SPROKIT_CONNECT("keyframes", "feature_track_set", "keyframes", "loop_back_tracks")
      << SPROKIT_CONNECT("keyframes", "feature_track_set", "detect_if_keyframe", "next_tracks")
      << SPROKIT_CONNECT("detect_if_keyframe", "feature_track_set", "detect_if_keyframe", "loop_back_tracks")
      << SPROKIT_CONNECT("input", "image", "detect_if_keyframe", "image")
      << SPROKIT_CONNECT("input", "timestamp", "detect_if_keyframe", "timestamp")
      << SPROKIT_CONNECT("detect_if_keyframe", "feature_track_set", "loop_detector", "next_tracks")
      << SPROKIT_CONNECT("loop_detector", "feature_track_set", "loop_detector", "loop_back_tracks")
      << SPROKIT_CONNECT("input", "timestamp", "loop_detector", "timestamp")
      << SPROKIT_CONNECT("loop_detector", "feature_track_set", "output", "feature_track_set");
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

  kwiver::vital::frame_id_t frame = this->activeFrame();
  kwiver::vital::timestamp currentTimestamp;

  d->video_reader->open(this->data()->videoPath);

  // Seek to just before active frame TODO: check status?
  if (frame > 1)
  {
    d->video_reader->seek_frame(currentTimestamp, frame - 1);
  }

  this->updateProgress(static_cast<int>(frame), maxFrame);
  this->setDescription("Parsing video frames");

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
      auto ix = rds->find("feature_track_set");
      if (ix != rds->end())
      {
        auto out_tracks = ix->second->get_datum<kwiver::vital::feature_track_set_sptr>();
        // make a copy of the tool data
        auto data = std::make_shared<ToolData>();
        data->copyTracks(out_tracks);
        data->activeFrame = out_tracks->last_frame();
        data->progress = progress();
        data->description = description().toStdString();
        emit updated(data);
      }
    }
  }
  d->ep.send_end_of_input();

  kwiver::vital::feature_track_set_sptr out_tracks;
  while (!d->ep.at_end())
  {
    auto rds = d->ep.receive();
    auto ix = rds->find("feature_track_set");
    if (ix != rds->end())
    {
      out_tracks = ix->second->get_datum<kwiver::vital::feature_track_set_sptr>();
      // make a copy of the tool data
      auto data = std::make_shared<ToolData>();
      data->copyTracks(out_tracks);
      data->activeFrame = out_tracks->last_frame();
      emit updated(data);
    }
  }
  d->ep.wait();

  this->updateTracks(out_tracks);

}
