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
  const int total_support = 2 * halfsupport + 1;


  std::vector<kwiver::vital::frame_id_t> frames_in_range;
  for (auto itr = cm.begin(); itr != cm.end(); itr++)
  {
    if (itr->first >= d->start_frame && (d->end_frame < 0 || itr->first <= d->end_frame))
      frames_in_range.push_back(itr->first);
  }

  kwiver::vital::frame_id_t num_frames =
    frames_in_range.size() - 2 * halfsupport;

  const size_t num_depth_maps =
    std::min(static_cast<size_t>(d->num_depth),
             static_cast<size_t>(num_frames));

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
  for (size_t c = 0; c < num_depth_maps; ++c)
  {
    size_t curr_frame_idx = halfsupport + (c * num_frames) / (num_depth_maps-1);
    this->updateProgress(static_cast<int>(c),
                         static_cast<int>(num_depth_maps));
    auto fitr = frames_in_range.begin() + curr_frame_idx;

    // Compute an iterator range containing total_support entries and
    // centered at the current frame.  When at the boundary the window
    // shifts to be not centered, but retains the same size.
    auto fitr_begin = (fitr - frames_in_range.begin() > halfsupport) ?
      fitr - halfsupport : frames_in_range.begin();
    auto fitr_end = frames_in_range.end();
    if (fitr_end - fitr_begin > total_support)
    {
      fitr_end = fitr_begin + total_support;
    }
    else
    {
      // if cut off at the end, back up the beginning
      fitr_begin = (fitr_end - frames_in_range.begin() > total_support) ?
        fitr_end - total_support : frames_in_range.begin();
    }

    std::vector<kwiver::vital::image_container_sptr> frames_out;
    std::vector<kwiver::vital::camera_perspective_sptr> cameras_out;
    std::vector<kwiver::vital::frame_id_t> frame_ids;
    int ref_frame = 0; //local ref frame

    kwiver::vital::timestamp currentTimestamp;
    d->video_reader->seek_frame(currentTimestamp, *fitr_begin);
    // collect all the frames
    for (auto f = fitr_begin; f < fitr_end; ++f)
    {
      while (currentTimestamp.get_frame() < *f)
      {
        d->video_reader->next_frame(currentTimestamp);
      }
      if (currentTimestamp.get_frame() != *f)
      {
        LOG_WARN(this->data()->logger, "Could not find frame " << *f);
        continue;
      }
      auto cam = cm.find(*f);
      auto const image = d->video_reader->frame_image();
      if (!image)
      {
        LOG_WARN(this->data()->logger, "No image available on frame " << *f);
        continue;
      }
      auto const mdv = d->video_reader->frame_metadata();
      if (!mdv.empty())
      {
        image->set_metadata(mdv[0]);
      }
      if (*f == *fitr)
      {
        ref_frame = static_cast<int>(frames_out.size());
      }
      frames_out.push_back(image);
      cameras_out.push_back(std::dynamic_pointer_cast<camera_perspective>(cam->second));
      frame_ids.push_back(*f);
    }

    kwiver::vital::image_container_sptr ref_img = frames_out[ref_frame];

    vtkBox* roi = this->ROI();
    double minptd[3], maxptd[3];
    roi->GetXMin(minptd);
    roi->GetXMax(maxptd);
    kwiver::vital::vector_3d minpt(minptd);
    kwiver::vital::vector_3d maxpt(maxptd);

    kwiver::vital::bounding_box<int> crop = kwiver::arrows::core::project_3d_bounds(
      minpt, maxpt, *cameras_out[ref_frame],
      static_cast<int>(ref_img->width()),
      static_cast<int>(ref_img->height()));

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
    data->activeFrame = *fitr;
    emit updated(data);
    if (this->isCanceled())
      return;
    //this->updateDepth(image_data);
  }
}

