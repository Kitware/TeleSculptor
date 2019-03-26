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

#include <arrows/core/depth_utils.h>
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
#include <vtkIntArray.h>

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
  ComputeDepthToolPrivate() : crop(0,0,0,0) {}
  video_input_sptr video_reader;
  compute_depth_sptr depth_algo;
  unsigned int max_iterations;
  int num_support;
  kwiver::vital::image_container_sptr ref_img;
  kwiver::vital::frame_id_t ref_frame;
  kwiver::vital::bounding_box<int> crop;
};

QTE_IMPLEMENT_D_FUNC(ComputeDepthTool)

//-----------------------------------------------------------------------------
ComputeDepthTool::ComputeDepthTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new ComputeDepthToolPrivate)
{
  this->data()->logger =
    kwiver::vital::get_logger("telesculptor.tools.compute_depth");

  this->setText("&Compute Single Depth Map");
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

  // TODO: find a more general way to get the number of iterations
  std::string iterations_key = ":super3d:iterations";
  d->max_iterations = config->get_value<unsigned int>(BLOCK_CD + iterations_key, 0);
  d->num_support = config->get_value<int>("compute_depth:num_support", 20);

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
             kwiver::vital::image_container_sptr color_img,
             int i0, int ni, int j0, int nj)
{
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

  vtkNew<vtkIntArray> crop;
  crop->SetName("Crop");
  crop->SetNumberOfComponents(1);
  crop->SetNumberOfValues(4);
  crop->SetValue(0, i0);  crop->SetValue(1, ni);
  crop->SetValue(2, j0);  crop->SetValue(3, nj);

  vtkIdType pt_id = 0;

  auto dep_im = depth_img->get_image();
  auto col_im = color_img->get_image();

  for (int y = nj - 1; y >= 0; y--)
  {
    for (int x = 0; x < ni; x++)
    {
      uniquenessRatios->SetValue(pt_id, 0);
      bestCost->SetValue(pt_id, 0);
      depths->SetValue(pt_id, dep_im.at<double>(x, y));
      color->SetTuple3(pt_id, (int)col_im.at<unsigned char>(x + i0, y + j0, 0),
                              (int)col_im.at<unsigned char>(x + i0, y + j0, 1),
                              (int)col_im.at<unsigned char>(x + i0, y + j0, 2));
      pt_id++;
    }
  }

  vtkSmartPointer<vtkImageData> imageData = vtkSmartPointer<vtkImageData>::New();
  imageData->SetSpacing(1, 1, 1);
  imageData->SetOrigin(0, 0, 0);
  imageData->SetDimensions(ni , nj, 1);
  imageData->GetPointData()->AddArray(depths.Get());
  imageData->GetPointData()->AddArray(color.Get());
  imageData->GetPointData()->AddArray(uniquenessRatios.Get());
  imageData->GetPointData()->AddArray(bestCost.Get());
  imageData->GetFieldData()->AddArray(crop.Get());
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
  const int halfsupport = d->num_support;
  int ref_frame = 0;

  this->setDescription("Collecting Video Frames");
  auto data = std::make_shared<ToolData>();
  data->activeFrame = frame;
  emit updated(data);

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

  // Convert landmarks to vector
  landmarks_out.reserve(lm.size());
  foreach(auto const& l, lm)
  {
    landmarks_out.push_back(l.second);
  }

  d->ref_img = frames_out[ref_frame];
  d->ref_frame = frame; //absolute ref frame in video

  vtkBox* roi = this->ROI();
  double minptd[3], maxptd[3];
  roi->GetXMin(minptd);
  roi->GetXMax(maxptd);
  kwiver::vital::vector_3d minpt(minptd);
  kwiver::vital::vector_3d maxpt(maxptd);

  d->crop =  kwiver::arrows::core::project_3d_bounds(minpt, maxpt, *cameras_out[ref_frame],
    static_cast<int>(d->ref_img->width()),
    static_cast<int>(d->ref_img->height()));

  double height_min, height_max;
  kwiver::arrows::core::height_range_from_3d_bounds(minpt, maxpt, height_min, height_max);

  //compute depth
  this->setDescription("Computing Cost Volume");
  data = std::make_shared<ToolData>();
  data->activeFrame = frame;
  emit updated(data);
  auto depth = d->depth_algo->compute(frames_out, cameras_out,
                                      height_min, height_max,
                                      ref_frame, d->crop);
  auto image_data = depth_to_vtk(depth, frames_out[ref_frame], d->crop.min_x(), d->crop.width(),
                                 d->crop.min_y(), d->crop.height());

  this->updateDepth(image_data);
}

//-----------------------------------------------------------------------------
bool
ComputeDepthTool::callback_handler(kwiver::vital::image_container_sptr depth,
                                   unsigned int iterations)
{
  QTE_D();
  // make a copy of the tool data
  auto data = std::make_shared<ToolData>();
  auto depthData = depth_to_vtk(depth, d->ref_img, d->crop.min_x(), d->crop.width(),
                                d->crop.min_y(), d->crop.height());

  data->copyDepth(depthData);
  data->activeFrame = d->ref_frame;
  this->setDescription("Optimizing Depth");
  this->updateProgress(iterations, d->max_iterations);

  emit updated(data);
  return !this->isCanceled();
}
