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
#include "ComputeAllDepthTool.h"
#include "GuiCommon.h"

#include <arrows/core/depth_utils.h>
#include <vital/algo/compute_depth.h>
#include <vital/algo/image_io.h>
#include <vital/algo/video_input.h>
#include <vital/config/config_block_io.h>
#include <vital/types/metadata.h>

#include <QMessageBox>
#include <qtStlUtil.h>

#include <algorithm>

#include <vtkDoubleArray.h>
#include <vtkImageData.h>
#include <vtkIntArray.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>

using kwiver::vital::algo::compute_depth;
using kwiver::vital::algo::compute_depth_sptr;
using kwiver::vital::algo::image_io;
using kwiver::vital::algo::image_io_sptr;
using kwiver::vital::algo::video_input;
using kwiver::vital::algo::video_input_sptr;

namespace
{
static char const* const BLOCK_VR = "video_reader";
static char const* const BLOCK_CD = "compute_depth";
}

//-----------------------------------------------------------------------------
class ComputeAllDepthToolPrivate
{
public:

  video_input_sptr video_reader;
  compute_depth_sptr depth_algo;
  int start_frame, end_frame, num_depth, num_support;
};

QTE_IMPLEMENT_D_FUNC(ComputeAllDepthTool)

//-----------------------------------------------------------------------------
ComputeAllDepthTool::ComputeAllDepthTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new ComputeAllDepthToolPrivate)
{
  this->data()->logger =
    kwiver::vital::get_logger("telesculptor.tools.compute_depth");

  this->setText("&Batch Compute Depth Maps");
  this->setToolTip("Computes depth maps for multiple images.");
}

//-----------------------------------------------------------------------------
ComputeAllDepthTool::~ComputeAllDepthTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs ComputeAllDepthTool::outputs() const
{
  return Depth | ActiveFrame | BatchDepth;
}

//-----------------------------------------------------------------------------
bool ComputeAllDepthTool::execute(QWidget* window)
{
  QTE_D();
  // Check inputs
  if (!this->hasVideoSource() || !this->hasCameras() || !this->hasLandmarks())
  {
    QMessageBox::information(
      window, "Insufficient data",
      "This operation requires a video source, cameras, and landmarks");
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

  config->merge_config(this->data()->config);
  if (!video_input::check_nested_algo_configuration(BLOCK_VR, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the video_input configuration.");
    return false;
  }

  if (!compute_depth::check_nested_algo_configuration(BLOCK_CD, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the compute_depth configuration.");
    return false;
  }

  // Create algorithm from configuration
  config->merge_config(this->data()->config);
  video_input::set_nested_algo_configuration(BLOCK_VR, config, d->video_reader);
  compute_depth::set_nested_algo_configuration(BLOCK_CD, config, d->depth_algo);

  d->start_frame = config->get_value<int>("batch_depth:first_frame", 0);
  d->end_frame = config->get_value<int>("batch_depth:end_frame", -1);
  d->num_depth = config->get_value<int>("batch_depth:num_depth", -1);

  d->num_support = config->get_value<int>("compute_depth:num_support", 10);


  return AbstractTool::execute(window);
}


//-----------------------------------------------------------------------------
void ComputeAllDepthTool::run()
{
  QTE_D();
  using kwiver::vital::camera_perspective;

  auto const& lm = this->landmarks()->landmarks();
  auto const& cm = this->cameras()->cameras();
  const int halfsupport = d->num_support;
  const int ref_frame = 0; //local ref frame

  std::vector<kwiver::vital::frame_id_t> frames_in_range;
  for (auto itr = cm.begin(); itr != cm.end(); itr++)
  {
    if (itr->first >= d->start_frame && (d->end_frame < 0 || itr->first <= d->end_frame))
      frames_in_range.push_back(itr->first);
  }

  kwiver::vital::frame_id_t total_frames = frames_in_range.size() - halfsupport;

  int stride = 1;
  if (d->num_depth > 0 && d->num_depth < total_frames)
    stride = (int)total_frames / d->num_depth;

  d->video_reader->open(this->data()->videoPath);

  //convert landmarks to vector
  std::vector<kwiver::vital::landmark_sptr> landmarks_out;
  foreach (auto const& l, lm)
  {
    landmarks_out.push_back(l.second);
  }

  this->setDescription("Estimating Depth");
  this->updateProgress(0, 100);
  auto data = std::make_shared<ToolData>();
  data->activeFrame = 0;
  emit updated(data);
  for (unsigned int f = 0; f < total_frames; f+=stride)
  {
    this->updateProgress(f, total_frames);
    int frame = frames_in_range[f];

    std::vector<kwiver::vital::image_container_sptr> frames_out;
    std::vector<kwiver::vital::camera_perspective_sptr> cameras_out;
    std::vector<kwiver::vital::frame_id_t> frame_ids;

    kwiver::vital::timestamp currentTimestamp;
    d->video_reader->seek_frame(currentTimestamp, frame);
    LOG_DEBUG(this->data()->logger,
              "Reference frame is " << currentTimestamp.get_frame());
    auto cam = cm.find(currentTimestamp.get_frame());
    if (cam == cm.end() || !cam->second)
    {
      LOG_DEBUG(this->data()->logger,
                "No camera available on the selected frame");
      return;
    }

    auto const image = d->video_reader->frame_image();
    if (!image)
    {
      LOG_DEBUG(this->data()->logger,
                "No image available on the selected frame");
      return;
    }
    auto const mdv = d->video_reader->frame_metadata();
    if (!mdv.empty())
    {
      image->set_metadata(mdv[0]);
    }
    frames_out.push_back(image);
    cameras_out.push_back(std::dynamic_pointer_cast<camera_perspective>(cam->second));
    frame_ids.push_back(currentTimestamp.get_frame());

    // find halfsupport more frames with valid cameras and images
    while (d->video_reader->good() && frames_out.size() <= halfsupport)
    {
      d->video_reader->next_frame(currentTimestamp);
      cam = cm.find(currentTimestamp.get_frame());
      if (cam == cm.end() || !cam->second)
      {
        continue;
      }
      auto const image = d->video_reader->frame_image();
      if (!image)
      {
        continue;
      }
      LOG_DEBUG(this->data()->logger,
                "Adding support frame " << currentTimestamp.get_frame());
      auto const mdv = d->video_reader->frame_metadata();
      if (!mdv.empty())
      {
        image->set_metadata(mdv[0]);
      }
      frames_out.push_back(image);
      cameras_out.push_back(std::dynamic_pointer_cast<camera_perspective>(cam->second));
      frame_ids.push_back(currentTimestamp.get_frame());
    }

    kwiver::vital::image_container_sptr ref_img = frames_out[ref_frame];

    vtkBox* roi = this->ROI();
    double minptd[3], maxptd[3];
    roi->GetXMin(minptd);
    roi->GetXMax(maxptd);
    kwiver::vital::vector_3d minpt(minptd);
    kwiver::vital::vector_3d maxpt(maxptd);

    kwiver::vital::bounding_box<int> crop = kwiver::arrows::core::project_3d_bounds(
      minpt, maxpt, *cameras_out[ref_frame], ref_img->width(), ref_img->height());

    double height_min, height_max;
    kwiver::arrows::core::height_range_from_3d_bounds(minpt, maxpt, height_min, height_max);

    //compute depth
    auto depth = d->depth_algo->compute(frames_out, cameras_out,
                                        height_min, height_max,
                                        ref_frame, crop);
    auto image_data = depth_to_vtk(depth, frames_out[ref_frame], crop.min_x(), crop.width(),
                                   crop.min_y(), crop.height());

    auto data = std::make_shared<ToolData>();
    data->copyDepth(image_data);
    data->activeFrame = frame;
    emit updated(data);
    if (this->isCanceled())
      return;
    //this->updateDepth(image_data);
  }
}

