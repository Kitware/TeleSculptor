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

#include "FuseDepthTool.h"
#include "GuiCommon.h"

#include <vital/algo/image_io.h>
#include <vital/algo/integrate_depth_maps.h>
#include <vital/algo/video_input.h>
#include <vital/config/config_block_io.h>
#include <vital/types/metadata.h>
#include <vital/types/vector.h>

#include <arrows/mvg/sfm_utils.h>
#include <arrows/vtk/depth_utils.h>

#include <qtStlUtil.h>
#include <QMessageBox>

#include <algorithm>

#include <vtkIntArray.h>
#include <vtkDoubleArray.h>
#include <vtkImageData.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>
#include <vtkXMLImageDataReader.h>
#include <vtkStructuredGrid.h>
#include <vtkCellData.h>
#include <vtkCellDataToPointData.h>

using kwiver::vital::algo::image_io;
using kwiver::vital::algo::image_io_sptr;
using kwiver::vital::algo::integrate_depth_maps;
using kwiver::vital::algo::integrate_depth_maps_sptr;

namespace
{
static char const* const BLOCK_IDM = "integrate_depth_maps";
}

//-----------------------------------------------------------------------------
class FuseDepthToolPrivate
{
public:
  integrate_depth_maps_sptr fuse_algo;
};

QTE_IMPLEMENT_D_FUNC(FuseDepthTool)

//-----------------------------------------------------------------------------
FuseDepthTool::FuseDepthTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new FuseDepthToolPrivate)
{
  this->data()->logger =
    kwiver::vital::get_logger("telesculptor.tools.fuse_depth");

  this->setText("&Fuse Depth Maps");
  this->setToolTip("Fuses all depth maps.");
}

//-----------------------------------------------------------------------------
FuseDepthTool::~FuseDepthTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs FuseDepthTool::outputs() const
{
  return Fusion;
}

//-----------------------------------------------------------------------------
bool FuseDepthTool::execute(QWidget* window)
{
  QTE_D();
  // Check inputs
  if (!this->hasDepthLookup())
  {
    QMessageBox::information(
      window, "Insufficient data",
      "This operation requires depth maps");
    return false;
  }

    // Check inputs
  if (!this->hasCameras())
  {
    QMessageBox::information(
      window, "Insufficient data",
      "This operation requires cameras");
    return false;
  }
  // Load configuration
  auto const config = readConfig("gui_integrate_depth_maps.conf");

  // Check configuration
  if (!config)
  {
    QMessageBox::critical(
      window, "Configuration error",
      "No configuration data was found. Please check your installation.");
    return false;
  }

  config->merge_config(this->data()->config);

  if(!integrate_depth_maps::check_nested_algo_configuration(BLOCK_IDM, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the integrate_depth_maps configuration.");
    return false;
  }

  // Create algorithm from configuration
  integrate_depth_maps::
    set_nested_algo_configuration(BLOCK_IDM, config, d->fuse_algo);

  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
void FuseDepthTool::run()
{
  using namespace kwiver::vital;
  QTE_D();

  auto const& depths = this->depthLookup();
  auto const& cameras = this->cameras()->cameras();
  vtkBox *roi = this->ROI();

  std::vector<camera_perspective_sptr> cameras_out;
  std::vector<image_container_sptr> depths_out;
  std::vector<image_container_sptr> weights_out;

  for (auto const& item : *depths)
  {
    auto camitr = cameras.find(item.first);
    if (camitr == cameras.end())
      continue;
    camera_perspective_sptr cam =
      std::dynamic_pointer_cast<camera_perspective>(camitr->second);
    bounding_box<int> crop;
    image_container_sptr depth, weight, uncertainty, color;
    kwiver::arrows::vtk::load_depth_map(
      item.second, crop, depth, weight, uncertainty, color);
    depths_out.push_back(depth);
    weights_out.push_back(weight);
    cameras_out.push_back(kwiver::arrows::mvg::crop_camera(cam, crop));
  }

  double minptd[3];
  roi->GetXMin(minptd);
  vector_3d minpt(minptd);

  double maxptd[3];
  roi->GetXMax(maxptd);
  vector_3d maxpt(maxptd);

  image_container_sptr volume;
  vector_3d spacing;
  d->fuse_algo->integrate(minpt, maxpt,
                          depths_out, weights_out, cameras_out,
                          volume, spacing);

  vtkSmartPointer<vtkImageData> vtk_volume =
    kwiver::arrows::vtk::volume_to_vtk(volume, minpt, spacing);

  this->updateFusion(vtk_volume);
}


