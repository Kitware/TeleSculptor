/*ckwg +29
 * Copyright 2018-2020 by Kitware, Inc.
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
#include <vital/algo/compute_depth.h>
#include <vital/algo/image_io.h>
#include <vital/algo/video_input.h>
#include <vital/config/config_block_io.h>
#include <vital/exceptions/base.h>
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
static char const* const BLOCK_MR = "mask_reader";
} // namespace

//-----------------------------------------------------------------------------
class ComputeDepthToolPrivate
{
public:
  ComputeDepthToolPrivate()
    : crop(0, 0, 0, 0) {}
  video_input_sptr video_reader;
  video_input_sptr mask_reader;
  compute_depth_sptr depth_algo;
  unsigned int max_iterations;
  int num_support;
  kwiver::vital::image_of<unsigned char> ref_img;
  kwiver::vital::image_of<unsigned char> ref_mask;
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

  if (!compute_depth::check_nested_algo_configuration(BLOCK_CD, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the compute_depth configuration.");
    return false;
  }

  auto const hasMask = !this->data()->maskPath.empty();
  if (hasMask && !video_input::check_nested_algo_configuration(BLOCK_MR, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the mask reader configuration.");
    return false;
  }

  // Create algorithm from configuration
  config->merge_config(this->data()->config);
  video_input::set_nested_algo_configuration(BLOCK_VR, config, d->video_reader);
  compute_depth::set_nested_algo_configuration(BLOCK_CD, config, d->depth_algo);
  video_input::set_nested_algo_configuration(BLOCK_MR, config, d->mask_reader);

  // TODO: find a more general way to get the number of iterations
  std::string iterations_key = ":super3d:iterations";
  d->max_iterations = config->get_value<unsigned int>(BLOCK_CD + iterations_key, 0);
  d->num_support = config->get_value<int>("compute_depth:num_support", 10);

  // Set the callback to receive updates
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;
  typedef compute_depth::callback_t callback_t;
  callback_t cb = std::bind(&ComputeDepthTool::callback_handler, this,
                            _1, _2, _3, _4);
  d->depth_algo->set_callback(cb);

  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkImageData>
depth_to_vtk(const kwiver::vital::image_of<double>& depth_img,
             const kwiver::vital::image_of<unsigned char>& color_img,
             int i0, int ni, int j0, int nj,
             const kwiver::vital::image_of<double>& uncertainty_img,
             const kwiver::vital::image_of<unsigned char>&mask_img)
{
  if (depth_img.size() == 0 ||
      color_img.size() == 0 ||
      depth_img.width() + i0 > color_img.width() ||
      depth_img.height() + j0 > color_img.height())
  {
    return nullptr;
  }

  vtkNew<vtkDoubleArray> uncertainty;
  uncertainty->SetName("Uncertainty");
  uncertainty->SetNumberOfValues(ni * nj);

  vtkNew<vtkDoubleArray> weight;
  weight->SetName("Weight");
  weight->SetNumberOfValues(ni * nj);

  vtkNew<vtkUnsignedCharArray> color;
  color->SetName("Color");
  color->SetNumberOfComponents(3);
  color->SetNumberOfTuples(ni * nj);

  vtkNew<vtkDoubleArray> depths;
  depths->SetName("Depths");
  depths->SetNumberOfComponents(1);
  depths->SetNumberOfTuples(ni * nj);

  vtkNew<vtkIntArray> crop;
  crop->SetName("Crop");
  crop->SetNumberOfComponents(1);
  crop->SetNumberOfValues(4);
  crop->SetValue(0, i0);
  crop->SetValue(1, ni);
  crop->SetValue(2, j0);
  crop->SetValue(3, nj);

  vtkIdType pt_id = 0;

  for (int y = nj - 1; y >= 0; y--)
  {
    for (int x = 0; x < ni; x++)
    {
      if (mask_img.size() > 0)
      {
        if (mask_img(x, y) > 127)
          weight->SetValue(pt_id, 1.0);
        else
          weight->SetValue(pt_id, 0.0);
      }
      else
      {
        weight->SetValue(pt_id, 1.0);
      }

      if (uncertainty_img.size() > 0)
      {
        uncertainty->SetValue(pt_id, uncertainty_img(x, y));
      }
      else
      {
        //TODO pick a meaningful default value
        uncertainty->SetValue(pt_id, 0);
      }

      depths->SetValue(pt_id, depth_img(x, y));
      
      color->SetTuple3(pt_id,
                       (int)color_img(x + i0, y + j0, 0),
                       (int)color_img(x + i0, y + j0, 1),
                       (int)color_img(x + i0, y + j0, 2));
      pt_id++;
    }
  }

  vtkSmartPointer<vtkImageData> imageData = vtkSmartPointer<vtkImageData>::New();
  imageData->SetSpacing(1, 1, 1);
  imageData->SetOrigin(0, 0, 0);
  imageData->SetDimensions(ni, nj, 1);
  imageData->GetPointData()->AddArray(depths.Get());
  imageData->GetPointData()->AddArray(color.Get());
  imageData->GetPointData()->AddArray(uncertainty.Get());
  imageData->GetPointData()->AddArray(weight.Get());
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
  auto const hasMask = !this->data()->maskPath.empty();

  std::vector<kwiver::vital::image_container_sptr> frames_out;
  std::vector<kwiver::vital::camera_perspective_sptr> cameras_out;
  std::vector<kwiver::vital::image_container_sptr> masks_out;
  std::vector<kwiver::vital::landmark_sptr> landmarks_out;
  std::vector<kwiver::vital::frame_id_t> frame_ids;
  const int halfsupport = d->num_support;
  const int total_support = 2 * d->num_support + 1;
  int ref_frame = 0;

  this->setDescription("Collecting Video Frames");
  auto data = std::make_shared<ToolData>();
  data->activeFrame = frame;
  emit updated(data);

  // get a sorted list of all frames which have cameras
  std::vector<kwiver::vital::frame_id_t> frames;
  frames.reserve(cm.size());
  for (auto const p : cm)
  {
    frames.push_back(p.first);
  }
  // find an iterator for the current frame
  auto fitr = std::lower_bound(frames.begin(), frames.end(), frame);
  if (fitr == frames.end() || *fitr != frame)
  {
    const std::string msg = "No camera available on the selected frame";
    LOG_DEBUG(this->data()->logger, msg);
    throw kwiver::vital::invalid_value(msg);
  }

  // Compute an iterator range containing total_support entries and
  // centered at the current frame.  When at the boundary the window
  // shifts to be not centered, but retains the same size.
  auto fitr_begin = (fitr - frames.begin() > halfsupport) ?
                    fitr - halfsupport : frames.begin();
  auto fitr_end = frames.end();
  if (fitr_end - fitr_begin > total_support)
  {
    fitr_end = fitr_begin + total_support;
  }
  else
  {
    // if cut off at the end, back up the beginning
    fitr_begin = (fitr_end - frames.begin() > total_support) ?
                 fitr_end - total_support : frames.begin();
  }

  d->video_reader->open(this->data()->videoPath);
  if (hasMask)
  {
    d->mask_reader->open(this->data()->maskPath);
  }

  kwiver::vital::timestamp currentTimestamp;
  // seek to the first frame
  d->video_reader->seek_frame(currentTimestamp, *fitr_begin);
  if (hasMask)
  {
    d->mask_reader->seek_frame(currentTimestamp, *fitr_begin);
  }

  // collect all the frames
  for (auto f = fitr_begin; f < fitr_end; ++f)
  {
    while (currentTimestamp.get_frame() < *f)
    {
      d->video_reader->next_frame(currentTimestamp);
      if (hasMask)
        d->mask_reader->next_frame(currentTimestamp);
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
    if (*f == frame)
    {
      ref_frame = static_cast<int>(frames_out.size());
    }
    frames_out.push_back(image);
    cameras_out.push_back(std::dynamic_pointer_cast<camera_perspective>(cam->second));
    frame_ids.push_back(*f);

    if (hasMask)
    {
      masks_out.push_back(d->mask_reader->frame_image());
    }
  }

  LOG_DEBUG(this->data()->logger, "ref frame at index " << ref_frame
                                  << " out of " << frames_out.size());

  // Convert landmarks to vector
  landmarks_out.reserve(lm.size());
  foreach (auto const& l, lm)
  {
    landmarks_out.push_back(l.second);
  }

  d->ref_img = frames_out[ref_frame]->get_image();
  if (hasMask)
  {
    d->ref_mask = masks_out[ref_frame]->get_image();
  }
  d->ref_frame = frame; //absolute ref frame in video

  vtkBox* roi = this->ROI();
  double minptd[3], maxptd[3];
  roi->GetXMin(minptd);
  roi->GetXMax(maxptd);
  kwiver::vital::vector_3d minpt(minptd);
  kwiver::vital::vector_3d maxpt(maxptd);

  d->crop = kwiver::arrows::core::project_3d_bounds(minpt, maxpt, *cameras_out[ref_frame],
                                                    static_cast<int>(d->ref_img.width()),
                                                    static_cast<int>(d->ref_img.height()));

  double height_min, height_max;
  kwiver::arrows::core::height_range_from_3d_bounds(minpt, maxpt, height_min, height_max);

  //compute depth
  this->setDescription("Computing Cost Volume");
  data = std::make_shared<ToolData>();
  data->activeFrame = frame;
  emit updated(data);

  kwiver::vital::image_container_sptr uncertainty = nullptr;
  auto depth = d->depth_algo->compute(frames_out, cameras_out,
                                      height_min, height_max,
                                      ref_frame, d->crop,
                                      uncertainty, masks_out);
  if (!depth)
  {
    // processing was terminated before any result was produced
    return;
  }

  kwiver::vital::image_of<double> uncertainty_img;
  if (uncertainty)
  {
    uncertainty_img = kwiver::vital::image_of<double>(uncertainty->get_image());
  }
  else
  {
    uncertainty_img = kwiver::vital::image_of<double>();
  }
  kwiver::vital::image_of<double> depth_img(depth->get_image());
  auto image_data = depth_to_vtk(depth_img, d->ref_img,
                                 d->crop.min_x(), d->crop.width(),
                                 d->crop.min_y(), d->crop.height(),
                                 uncertainty_img, d->ref_mask);

  this->updateDepth(image_data);
}

//-----------------------------------------------------------------------------
bool
ComputeDepthTool::callback_handler(kwiver::vital::image_container_sptr depth,
                                   std::string const& status,
                                   unsigned int percent_complete,
                                   kwiver::vital::image_container_sptr uncertainty)
{
  QTE_D();
  // make a copy of the tool data
  auto data = std::make_shared<ToolData>();
  if (depth)
  {
    kwiver::vital::image_of<double> depth_img(depth->get_image());
    kwiver::vital::image_of<double> uncertainty_img;
    if (uncertainty)
    {
      uncertainty_img = kwiver::vital::image_of<double>(uncertainty->get_image());
    }
    else
    {
      uncertainty_img = kwiver::vital::image_of<double>();
    }
    auto depthData = depth_to_vtk(depth_img, d->ref_img,
                                  d->crop.min_x(), d->crop.width(),
                                  d->crop.min_y(), d->crop.height(),
                                  uncertainty_img, d->ref_mask);
    data->copyDepth(depthData);
  }
  data->activeFrame = d->ref_frame;
  this->setDescription(QString::fromStdString(status));
  this->updateProgress(percent_complete);

  emit updated(data);
  return !this->isCanceled();
}
