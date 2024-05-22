// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "ComputeDepthTool.h"
#include "ComputeAllDepthTool.h"
#include "GuiCommon.h"

#include <arrows/core/depth_utils.h>
#include <vital/algo/algorithm.txx>
#include <vital/algo/compute_depth.h>
#include <vital/algo/image_io.h>
#include <vital/algo/video_input.h>
#include <vital/config/config_block_io.h>
#include <vital/types/metadata.h>

#include <QMessageBox>
#include <qtStlUtil.h>

#include <algorithm>
#include <functional>

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
}

//-----------------------------------------------------------------------------
class ComputeAllDepthToolPrivate
{
public:
  ComputeAllDepthToolPrivate()
    : crop(0, 0, 0, 0) {}

  video_input_sptr video_reader;
  video_input_sptr mask_reader;
  compute_depth_sptr depth_algo;
  int start_frame, end_frame, num_depth, num_support;
  double angle_span;
  kwiver::vital::image_of<unsigned char> ref_img;
  kwiver::vital::image_of<unsigned char> ref_mask;
  kwiver::vital::frame_id_t active_frame;
  kwiver::vital::bounding_box<int> crop;
  size_t depth_count;
  size_t num_depth_maps;
};

QTE_IMPLEMENT_D_FUNC(ComputeAllDepthTool)

//-----------------------------------------------------------------------------
ComputeAllDepthTool::ComputeAllDepthTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new ComputeAllDepthToolPrivate)
{
  this->data()->logger =
    kwiver::vital::get_logger("telesculptor.tools.batch_compute_depth");

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
  return Depth | ActiveFrame;
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
  if (!kwiver::vital::check_nested_algo_configuration<video_input>(BLOCK_VR, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the video_input configuration.");
    return false;
  }

  if (!kwiver::vital::check_nested_algo_configuration<compute_depth>(BLOCK_CD, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the compute_depth configuration.");
    return false;
  }

  auto const hasMask = !this->data()->maskPath.empty();
  if (hasMask && !kwiver::vital::check_nested_algo_configuration<video_input>(BLOCK_MR, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the mask reader configuration.");
    return false;
  }

  // Create algorithm from configuration
  config->merge_config(this->data()->config);
  kwiver::vital::set_nested_algo_configuration<video_input>(BLOCK_VR, config, d->video_reader);
  kwiver::vital::set_nested_algo_configuration<compute_depth>(BLOCK_CD, config, d->depth_algo);
  kwiver::vital::set_nested_algo_configuration<video_input>(BLOCK_MR, config, d->mask_reader);

  d->start_frame = config->get_value<int>("batch_depth:first_frame", 0);
  d->end_frame = config->get_value<int>("batch_depth:end_frame", -1);
  d->num_depth = config->get_value<int>("batch_depth:num_depth", -1);

  d->num_support = config->get_value<int>("compute_depth:num_support", 20);
  d->angle_span = config->get_value<double>("compute_depth:angle_span", 15.0);

  // Set the callback to receive updates
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;
  typedef compute_depth::callback_t callback_t;
  callback_t cb = std::bind(&ComputeAllDepthTool::callback_handler,
                            this, _1, _2, _3, _4);

  d->depth_algo->set_callback(cb);

  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
void ComputeAllDepthTool::run()
{
  QTE_D();
  using kwiver::vital::camera_perspective;
  using kwiver::arrows::core::find_similar_cameras_angles;
  using kwiver::arrows::core::gather_depth_frames;
  using namespace std::placeholders;
  auto const hasMask = !this->data()->maskPath.empty();

  auto const& lm = this->landmarks()->landmarks();
  auto const& cm = this->cameras()->cameras();

  std::vector<kwiver::vital::frame_id_t> frames_in_range;
  for (auto itr = cm.begin(); itr != cm.end(); itr++)
  {
    if (itr->first >= d->start_frame && (d->end_frame < 0 || itr->first <= d->end_frame))
      frames_in_range.push_back(itr->first);
  }

  d->num_depth_maps =
    std::min(static_cast<size_t>(d->num_depth),
             frames_in_range.size());

  d->video_reader->open(this->data()->videoPath);
  if (hasMask)
  {
    d->mask_reader->open(this->data()->maskPath);
  }

  //convert landmarks to vector
  std::vector<kwiver::vital::landmark_sptr> landmarks_out;
  foreach (auto const& l, lm)
  {
    landmarks_out.push_back(l.second);
  }

  this->setDescription("Estimating Depth");
  this->updateProgress(0, 100);
  auto data = std::make_shared<ToolData>();
  emit updated(data);
  for (size_t c = 0; c < d->num_depth_maps && !this->isCanceled(); ++c)
  {
    d->depth_count = c;
    size_t curr_frame_idx = (c * (frames_in_range.size()-1)) /
                            (d->num_depth_maps-1);
    this->updateProgress(static_cast<int>(c),
                         static_cast<int>(d->num_depth_maps));
    auto fitr = frames_in_range.begin() + curr_frame_idx;
    d->active_frame = *fitr;

    kwiver::vital::camera_perspective_map pcm;
    pcm.set_from_base_camera_map(cm);
    auto const ref_cam_itr = cm.find(*fitr);
    if (ref_cam_itr == cm.end())
    {
      const std::string msg = "No camera available on the selected frame";
      LOG_DEBUG(this->data()->logger, msg);
      throw kwiver::vital::invalid_value(msg);
    }
    camera_perspective_sptr ref_cam =
      std::dynamic_pointer_cast<camera_perspective>(ref_cam_itr->second);
    if (!ref_cam)
    {
      const std::string msg = "Reference camera is not perspective";
      LOG_DEBUG(this->data()->logger, msg);
      throw kwiver::vital::invalid_value(msg);
    }

    auto similar_cameras =
      find_similar_cameras_angles(*ref_cam, pcm, d->angle_span, d->num_support);
    // make sure the reference frame is included
    similar_cameras->insert(*fitr, ref_cam);

    // collect all the frames
    std::vector<kwiver::vital::image_container_sptr> frames_out;
    std::vector<kwiver::vital::camera_perspective_sptr> cameras_out;
    std::vector<kwiver::vital::image_container_sptr> masks_out;
    video_input_sptr mask_reader = hasMask ? d->mask_reader : nullptr;
    auto cb = std::bind(&ComputeAllDepthTool::gather_status_handler,
                        this, _1, _2, static_cast<unsigned int>(c),
                        static_cast<unsigned int>(d->num_depth_maps), *fitr);
    int ref_frame = gather_depth_frames(*similar_cameras, d->video_reader,
                                        mask_reader, *fitr,
                                        cameras_out, frames_out, masks_out, cb);

    d->ref_img = frames_out[ref_frame]->get_image();
    if (hasMask)
    {
      d->ref_mask = masks_out[ref_frame]->get_image();
    }

    vtkBox* roi = this->ROI();
    double minptd[3], maxptd[3];
    roi->GetXMin(minptd);
    roi->GetXMax(maxptd);
    kwiver::vital::vector_3d minpt(minptd);
    kwiver::vital::vector_3d maxpt(maxptd);

    d->crop = kwiver::arrows::core::project_3d_bounds(
      minpt, maxpt, *cameras_out[ref_frame],
      static_cast<int>(d->ref_img.width()),
      static_cast<int>(d->ref_img.height()));

    double height_min, height_max;
    kwiver::arrows::core::height_range_from_3d_bounds(minpt, maxpt, height_min, height_max);

    kwiver::vital::image_container_sptr uncertainty = nullptr;
    //compute depth
    auto depth = d->depth_algo->compute(frames_out, cameras_out,
                                        height_min, height_max,
                                        static_cast<unsigned int>(ref_frame),
                                        d->crop, uncertainty, masks_out);
    if (!depth)
    {
      // depth computation terminated early or failed to produce a result
      continue;
    }

    kwiver::vital::image_of<double> uncertainty_img;
    if (uncertainty)
    {
      uncertainty_img = kwiver::vital::image_of<double>(uncertainty->get_image());
    }
    kwiver::vital::image_of<double> depth_img(depth->get_image());
    auto image_data = depth_to_vtk(depth_img, d->ref_img,
                                   d->crop.min_x(), d->crop.width(),
                                   d->crop.min_y(), d->crop.height(),
                                   uncertainty_img, d->ref_mask);

    auto data = std::make_shared<ToolData>();
    data->copyDepth(image_data);
    data->activeFrame = *fitr;
    emit saved(data);
  }
}

//-----------------------------------------------------------------------------
bool
ComputeAllDepthTool
::callback_handler(kwiver::vital::image_container_sptr depth,
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
  // Compute overall percent complete accounting for the percentage of depth
  // frames complete and the percent complete of the current depth frame.
  percent_complete = static_cast<unsigned int>(
                       (d->depth_count * 100 + percent_complete) /
                         d->num_depth_maps);
  // Create overall status message
  std::stringstream ss;
  ss << "Depth frame "<<d->depth_count << " of " << d->num_depth_maps
     << ": " << status;

  data->activeFrame = d->active_frame;
  this->setDescription(QString::fromStdString(ss.str()));
  this->updateProgress(percent_complete);

  emit updated(data);
  return !this->isCanceled();
}

/// handler for callback on image gathering status
bool
ComputeAllDepthTool
::gather_status_handler(unsigned int curr_frame,
                        unsigned int num_frames,
                        unsigned int curr_depth_map,
                        unsigned int num_depth_maps,
                        frame_id_t active_frame_id)
{
  // update status message
  std::stringstream ss;
  ss << "Depth frame " << curr_depth_map << " of " << num_depth_maps
    << ": Accumulating frame " << curr_frame
    << " of " << num_frames;
  this->setDescription(QString::fromStdString(ss.str()));

  auto data = std::make_shared<ToolData>();
  data->active_depth = nullptr;
  data->activeFrame = active_frame_id;
  emit updated(data);
  return !this->isCanceled();
}
