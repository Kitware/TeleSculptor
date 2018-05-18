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
#include "GuiCommon.h"

#include <vital/algo/image_io.h>
#include <vital/algo/compute_depth.h>
#include <vital/algo/video_input.h>
#include <vital/config/config_block_io.h>
#include <vital/types/metadata.h>

#include <qtStlUtil.h>
#include <QMessageBox>

#include <algorithm>

#include <vtkDoubleArray.h>
#include <vtkImageData.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>

using kwiver::vital::algo::image_io;
using kwiver::vital::algo::image_io_sptr;
using kwiver::vital::algo::compute_depth;
using kwiver::vital::algo::compute_depth_sptr;
using kwiver::vital::algo::video_input;
using kwiver::vital::algo::video_input_sptr;

namespace
{
static char const* const BLOCK_VR = "video_reader";
static char const* const BLOCK_CD = "compute_depth";
}

//-----------------------------------------------------------------------------
class ComputeDepthToolPrivate
{
public:
  video_input_sptr video_reader;
  compute_depth_sptr depth_algo;
  kwiver::vital::image_container_sptr ref_img;
  kwiver::vital::frame_id_t ref_frame;
};

QTE_IMPLEMENT_D_FUNC(ComputeDepthTool)

//-----------------------------------------------------------------------------
ComputeDepthTool::ComputeDepthTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new ComputeDepthToolPrivate)
{
  this->data()->logger =
    kwiver::vital::get_logger("telesculptor.tools.compute_depth");

  this->setText("&Compute Depth Map");
  this->setToolTip("Computes depth map on current image.");
}

//-----------------------------------------------------------------------------
ComputeDepthTool::~ComputeDepthTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs ComputeDepthTool::outputs() const
{
  return Depth | ActiveFrame;
}

//-----------------------------------------------------------------------------
bool ComputeDepthTool::execute(QWidget* window)
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

  if(!compute_depth::check_nested_algo_configuration(BLOCK_CD, config))
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

  // Set the callback to receive updates
  using std::placeholders::_1;
  using std::placeholders::_2;
  typedef compute_depth::callback_t callback_t;
  callback_t cb = std::bind(&ComputeDepthTool::callback_handler, this, _1, _2);
  d->depth_algo->set_callback(cb);

  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkImageData>
depth_to_vtk(kwiver::vital::image_container_sptr depth_img,
             kwiver::vital::image_container_sptr color_img)
{
  int ni = static_cast<int>(depth_img->width());
  int nj = static_cast<int>(depth_img->height());
  vtkNew<vtkDoubleArray> uniquenessRatios;
  uniquenessRatios->SetName("Uniqueness Ratios");
  uniquenessRatios->SetNumberOfValues(ni*nj);

  vtkNew<vtkDoubleArray> bestCost;
  bestCost->SetName("Best Cost Values");
  bestCost->SetNumberOfValues(ni*nj);

  vtkNew<vtkUnsignedCharArray> color;
  color->SetName("Color");
  color->SetNumberOfComponents(3);
  color->SetNumberOfTuples(ni*nj);

  vtkNew<vtkDoubleArray> depths;
  depths->SetName("Depths");
  depths->SetNumberOfComponents(1);
  depths->SetNumberOfTuples(ni*nj);

  vtkIdType pt_id = 0;

  for (int y = nj - 1; y >= 0; y--)
  {
    for (int x = 0; x < ni; x++)
    {
      uniquenessRatios->SetValue(pt_id, 0);
      bestCost->SetValue(pt_id, 0);
      depths->SetValue(pt_id, depth_img->get_image().at<double>(x, y));
      color->SetTuple3(pt_id, (int)color_img->get_image().at<unsigned char>(x, y, 0),
                              (int)color_img->get_image().at<unsigned char>(x, y, 1),
                              (int)color_img->get_image().at<unsigned char>(x, y, 2));
      pt_id++;
    }
  }

  vtkSmartPointer<vtkImageData> imageData = vtkSmartPointer<vtkImageData>::New();
  imageData->SetSpacing(1, 1, 1);
  imageData->SetOrigin(0, 0, 0);
  imageData->SetDimensions(ni, nj, 1);
  imageData->GetPointData()->AddArray(depths.Get());
  imageData->GetPointData()->AddArray(color.Get());
  imageData->GetPointData()->AddArray(uniquenessRatios.Get());
  imageData->GetPointData()->AddArray(bestCost.Get());
  return imageData;
}

//-----------------------------------------------------------------------------
void ComputeDepthTool::run()
{
  QTE_D();
  using kwiver::vital::camera_perspective;

  int frame = this->activeFrame();

  auto const& lm = this->landmarks()->landmarks();
  auto const& cm = this->cameras()->cameras();

  std::vector<kwiver::vital::image_container_sptr> frames_out;
  std::vector<kwiver::vital::camera_perspective_sptr> cameras_out;
  std::vector<kwiver::vital::landmark_sptr> landmarks_out;
  std::vector<kwiver::vital::frame_id_t> frame_ids;
  const int halfsupport = 10;
  int ref_frame = 0;

  d->video_reader->open(this->data()->videoPath);

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


  //convert landmarks to vector
  foreach(auto const& l, lm)
  {
    landmarks_out.push_back(l.second);
  }

  d->ref_img = frames_out[ref_frame];
  d->ref_frame = frame;

  //compute depth
  auto depth = d->depth_algo->compute(frames_out, cameras_out,
                                      landmarks_out, ref_frame);
  auto image_data = depth_to_vtk(depth, frames_out[ref_frame]);

  this->updateDepth(image_data);
}

//-----------------------------------------------------------------------------
bool
ComputeDepthTool::callback_handler(kwiver::vital::image_container_sptr depth,
                                   unsigned int iterations)
{
  // make a copy of the tool data
  auto data = std::make_shared<ToolData>();
  auto depthData = depth_to_vtk(depth, this->d_ptr->ref_img);

  data->copyDepth(depthData);
  data->activeFrame = d_ptr->ref_frame;

  emit updated(data);
  return !this->isCanceled();
}
