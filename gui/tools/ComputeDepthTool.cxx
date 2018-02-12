/*ckwg +29
 * Copyright 2018 by Kitware, Inc.
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

#include "ComputeDepthTool.h"

#include <maptk/colorize.h>
#include <maptk/version.h>

#include <vital/algo/image_io.h>
#include <vital/algo/convert_image.h>
#include <vital/algo/compute_depth.h>

#include <vital/config/config_block_io.h>
#include <vital/types/metadata.h>
#include <vital/types/metadata_traits.h>
#include <vital/algo/video_input.h>

#include <qtStlUtil.h>

#include <QtGui/QApplication>
#include <QtGui/QMessageBox>

#include <QtCore/QDir>

#include <algorithm>

using kwiver::vital::algo::image_io;
using kwiver::vital::algo::image_io_sptr;
using kwiver::vital::algo::convert_image;
using kwiver::vital::algo::convert_image_sptr;
using kwiver::vital::algo::compute_depth;
using kwiver::vital::algo::compute_depth_sptr;
using kwiver::vital::algo::video_input;
using kwiver::vital::algo::video_input_sptr;

namespace
{
static char const* const BLOCK_VR = "video_reader";
static char const* const BLOCK_CI = "image_converter";
static char const* const BLOCK_CD = "compute_depth";

//-----------------------------------------------------------------------------
kwiver::vital::config_block_sptr readConfig(std::string const& name)
{
  try
  {
    using kwiver::vital::read_config_file;

    auto const exeDir = QDir(QApplication::applicationDirPath());
    auto const prefix = stdString(exeDir.absoluteFilePath(".."));
    return read_config_file(name, "maptk", MAPTK_VERSION, prefix);
  }
  catch (...)
  {
    return {};
  }
}

}

//-----------------------------------------------------------------------------
class ComputeDepthToolPrivate
{
public:
  video_input_sptr video_reader;
  convert_image_sptr image_converter;
  compute_depth_sptr depth_algo;
};

QTE_IMPLEMENT_D_FUNC(ComputeDepthTool)

//-----------------------------------------------------------------------------
ComputeDepthTool::ComputeDepthTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new ComputeDepthToolPrivate)
{
  this->setText("&Compute Depth Maps");
  this->setToolTip("Computes depth maps on key images.");
}

//-----------------------------------------------------------------------------
ComputeDepthTool::~ComputeDepthTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs ComputeDepthTool::outputs() const
{
  return Depth;
}

//-----------------------------------------------------------------------------
bool ComputeDepthTool::execute(QWidget* window)
{
  QTE_D();

  if (!this->hasVideoSource())
  {
    QMessageBox::information(
      window, "Insufficient data", "This operation requires a video source.");
    return false;
  }

  // Load configuration
  auto const config = readConfig("gui_compute_depth.conf");

  // Check configuration
  if (!config)
  {
    QMessageBox::critical(
      window, "Configuration error",
      "No configuration data was found. Please check your installation.");
    return false;
  }

  if (!image_io::check_nested_algo_configuration(BLOCK_VR, config) ||
      !convert_image::check_nested_algo_configuration(BLOCK_CI, config) ||
      !compute_depth::check_nested_algo_configuration(BLOCK_CD, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the algorithm configuration.");
    return false;
  }

  // Create algorithm from configuration
  video_input::set_nested_algo_configuration(BLOCK_VR, this->data()->config, d->video_reader);
  convert_image::set_nested_algo_configuration(BLOCK_CI, config, d->image_converter);
  compute_depth::set_nested_algo_configuration(BLOCK_CD, config, d->depth_algo);

  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
void ComputeDepthTool::run()
{
  QTE_D();

  int frame = this->activeFrame();
  landmark_map_sptr landmarks_map = this->landmarks();
  camera_map_sptr cameras_map = this->cameras();

  d->video_reader->open(this->data()->videoPath);
  
  std::vector<kwiver::vital::image_container_sptr> frames;
  std::vector<kwiver::vital::camera_sptr> cameras;
  std::vector<kwiver::vital::landmark_sptr> landmarks;
  const unsigned int numsupport = 10;
  int halfsupport = numsupport / 2;

  if (d->video_reader->num_frames() < numsupport + 1)
    return;

  // compute support range
  int start = frame - halfsupport;
  int end = frame + halfsupport;
  if (start < 0) 
  {
    end += abs(start);
    start = 0;    
  }
  if (end >= d->video_reader->num_frames())
  {
    start -= end - d->video_reader->num_frames() + 1;
    end = d->video_reader->num_frames() - 1;
  }
  
  kwiver::vital::timestamp currentTimestamp;
  d->video_reader->seek_frame(currentTimestamp, start - 1);

  //collect images and cameras
  for (int i = start; i <= end; ++i)
  {
    auto const image = d->video_reader->frame_image();
    auto const converted_image = d->image_converter->convert(image);

    frames.push_back(converted_image);
    cameras.push_back(cameras_map->cameras()[i]);
    d->video_reader->next_frame(currentTimestamp);
  }

  //convert landmarks to vector
  for (kwiver::vital::landmark_map::map_landmark_t::const_iterator itr = landmarks_map->landmarks().begin(); itr != landmarks_map->landmarks().end(); itr++)
  {
    landmarks.push_back(itr->second);
  }

  //compute depth
  kwiver::vital::image_container_sptr depth = d->depth_algo->compute(frames, cameras, landmarks, frame);

  // make a copy of the tool data
  auto data = std::make_shared<ToolData>();
  data->copyDepth(depth);
  emit updated(data);
}
