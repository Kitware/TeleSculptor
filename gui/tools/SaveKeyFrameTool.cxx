// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "SaveKeyFrameTool.h"
#include "GuiCommon.h"

#include <iomanip>

#include <vital/algo/algorithm.txx>
#include <vital/algo/image_io.h>
#include <vital/algo/video_input.h>
#include <kwiversys/SystemTools.hxx>

#include <QDebug>
#include <QDir>
#include <QMessageBox>

using kwiver::vital::algo::video_input;
using kwiver::vital::algo::video_input_sptr;
using kwiver::vital::algo::image_io;
using kwiver::vital::algo::image_io_sptr;

namespace
{
static char const* const BLOCK_VR = "video_reader";
static char const* const BLOCK_IW = "image_writer";
static char const* const KEYFRAMES_TAG = "output_frames_dir";
static char const* const KEYFRAMES_PATH = "results/frames";
}

QTE_IMPLEMENT_D_FUNC(SaveKeyFrameTool)

//-----------------------------------------------------------------------------
class SaveKeyFrameToolPrivate
{
public:
  video_input_sptr video_reader;
  image_io_sptr image_writer;
};

//-----------------------------------------------------------------------------
SaveKeyFrameTool::SaveKeyFrameTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new SaveKeyFrameToolPrivate)
{
  this->data()->logger =
    kwiver::vital::get_logger("telesculptor.tools.save_key_frames");

  this->setText("Save Key Frames");
  this->setToolTip(
    "Saves the key frames from the Track Features to disk.");
}

//-----------------------------------------------------------------------------
SaveKeyFrameTool::~SaveKeyFrameTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs SaveKeyFrameTool::outputs() const
{
  return KeyFrames;
}

//-----------------------------------------------------------------------------
bool SaveKeyFrameTool::execute(QWidget* window)
{
  QTE_D();

  if (!this->hasTracks() || !this->hasVideoSource())
  {
    QMessageBox::information(
      window, "Insufficient data",
      "This operation requires tracks and a video source.");
    return false;
  }

  if (!kwiver::vital::check_nested_algo_configuration<video_input>(
    BLOCK_VR, this->data()->config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the video source algorithm configuration.");
    return false;
  }

  // Merge project config with default config file
  auto const config = readConfig("gui_keyframe_image_writer.conf");

  // Check configuration
  if (!config)
  {
    QMessageBox::critical(
      window, "Configuration error",
      "No configuration data was found. Please check your installation.");
    return false;
  }

  config->merge_config(this->data()->config);
  if (!kwiver::vital::check_nested_algo_configuration<image_io>(BLOCK_IW, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the image writer algorithm configuration.");
    return false;
  }

  kwiver::vital::set_nested_algo_configuration<video_input>(
    BLOCK_VR, this->data()->config, d->video_reader);
  kwiver::vital::set_nested_algo_configuration<image_io>(
    BLOCK_IW, config, d->image_writer);

  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
void SaveKeyFrameTool::run()
{
  QTE_D();

  std::string keyframesDir;
  if (this->data()->config->has_value(KEYFRAMES_TAG))
  {
    keyframesDir = this->data()->config->get_value<std::string>(KEYFRAMES_TAG);
  }
  else
  {
    keyframesDir = KEYFRAMES_PATH;
  }

  // Create keyframes directory if it doesn't exist
  if (!kwiversys::SystemTools::FileExists(keyframesDir))
  {
    kwiversys::SystemTools::MakeDirectory(keyframesDir);
  }

  kwiver::vital::timestamp currentTimestamp;
  d->video_reader->open(this->data()->videoPath);

  for (auto const& frame: this->tracks()->keyframes())
  {
    if (d->video_reader->seek_frame(currentTimestamp, frame))
    {
      auto md = d->video_reader->frame_metadata();
      auto filename = keyframesDir + "/" + frameName(frame, md) + ".png";
      try
      {
        d->image_writer->save(filename, d->video_reader->frame_image());
      }
      catch (std::exception const& e)
      {
        LOG_WARN(this->data()->logger, "Error writing frame to "
                                       << filename << ": " <<  e.what());
      }
    }
    else
    {
      LOG_WARN(this->data()->logger, "Key frame " << frame
                                     << " not available in video source.");
    }

    if( this->isCanceled() )
    {
      break;
    }
  }

  if (!this->data()->config->has_value(KEYFRAMES_TAG))
  {
    this->data()->config->set_value(KEYFRAMES_TAG, KEYFRAMES_PATH);
  }

  d->video_reader->close();
}
