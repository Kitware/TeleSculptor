/*ckwg +29
 * Copyright 2018-2019 by Kitware, Inc.
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
  integrate_depth_maps::set_nested_algo_configuration(BLOCK_IDM, config, d->fuse_algo);

  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
void load_depth_map(const std::string &filename, int &i0, int &ni, int &j0, int &nj,
                    kwiver::vital::image_container_sptr &depth_out,
                    kwiver::vital::image_container_sptr &weight_out)
{
  vtkNew<vtkXMLImageDataReader> depthReader;
  depthReader->SetFileName(filename.c_str());
  depthReader->Update();
  vtkImageData *img = depthReader->GetOutput();

  vtkIntArray* crop = static_cast<vtkIntArray*>(img->GetFieldData()->GetArray("Crop"));
  if (crop)
  {
    i0 = crop->GetValue(0);    ni = crop->GetValue(1);
    j0 = crop->GetValue(2);    nj = crop->GetValue(3);
  }
  else
  {
    i0 = j0 = 0;
    ni = img->GetDimensions()[0];    nj = img->GetDimensions()[1];
  }

  vtkDoubleArray *depths = dynamic_cast<vtkDoubleArray *>(img->GetPointData()->GetArray("Depths"));
  vtkDoubleArray *weights = dynamic_cast<vtkDoubleArray*>(img->GetPointData()->GetArray("Best Cost Values"));

  int dims[3];
  img->GetDimensions(dims);

  kwiver::vital::image_of<double> depth(dims[0], dims[1], dims[2]);
  kwiver::vital::image_of<double> weight(dims[0], dims[1], dims[2]);

  vtkIdType pt_id = 0;
  for (int x = 0; x < dims[0]; x++)
  {
    for (int y = 0; y < dims[1]; y++)
    {
      depth(x, y) = depths->GetValue(pt_id);
      weight(x, y) = weights->GetValue(pt_id);
      pt_id++;
    }
  }

  depth_out = std::make_shared<kwiver::vital::simple_image_container>(depth);
  weight_out = std::make_shared<kwiver::vital::simple_image_container>(weight);
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkStructuredGrid>
volume_to_vtk(kwiver::vital::image_container_sptr volume, kwiver::vital::vector_3d const& origin, kwiver::vital::vector_3d const& spacing)
{
  vtkSmartPointer<vtkImageData> grid = vtkSmartPointer<vtkImageData>::New();
  grid->SetOrigin(origin[0], origin[1], origin[2]);
  // vtk cells are dim - 1 for some reason
  grid->SetDimensions(static_cast<int>(volume->width() + 1),
                      static_cast<int>(volume->height() + 1),
                      static_cast<int>(volume->depth() + 1));
  grid->SetSpacing(spacing[0], spacing[1], spacing[2]);

  // initialize output
  vtkNew<vtkDoubleArray> vals;
  vals->SetName("reconstruction_scalar");
  vals->SetNumberOfComponents(1);
  vals->SetNumberOfTuples(volume->width() * volume->height() * volume->depth());

  vtkIdType pt_id = 0;
  const kwiver::vital::image &vol = volume->get_image();

  for (unsigned int k = 0; k < volume->depth(); k++)
  {
    for (unsigned int j = 0; j < volume->height(); j++)
    {
      for (unsigned int i = 0; i < volume->width(); i++)
      {
        vals->SetTuple1(pt_id++, vol.at<double>(i, j, k));
      }
    }
  }

  grid->GetCellData()->AddArray(vals.Get());

  vtkSmartPointer<vtkCellDataToPointData> celltopts =
    vtkSmartPointer<vtkCellDataToPointData>::New();

  celltopts->SetInputData(grid);
  celltopts->Update();
  vtkStructuredGrid* output = (vtkStructuredGrid *)celltopts->GetOutput();
  output->GetPointData()->SetActiveScalars("reconstruction_scalar");
  return vtkSmartPointer<vtkStructuredGrid>(output);
}

//-----------------------------------------------------------------------------

kwiver::vital::camera_perspective_sptr crop_camera(const kwiver::vital::camera_perspective_sptr& cam,
                                                  int i0, int ni, int j0, int nj)
{
  kwiver::vital::simple_camera_intrinsics newIntrinsics(*cam->intrinsics());
  kwiver::vital::vector_2d pp = newIntrinsics.principal_point();

  pp[0] -= i0;
  pp[1] -= j0;

  newIntrinsics.set_principal_point(pp);

  kwiver::vital::simple_camera_perspective cropCam(cam->center(),
                                                   cam->rotation(),
                                                   newIntrinsics);

  return std::dynamic_pointer_cast<kwiver::vital::camera_perspective>(cropCam.clone());
}

//-----------------------------------------------------------------------------
void FuseDepthTool::run()
{
  QTE_D();

  auto const& depths = this->depthLookup();
  auto const& cameras = this->cameras()->cameras();
  vtkBox *roi = this->ROI();

  std::vector<kwiver::vital::camera_perspective_sptr> cameras_out;
  std::vector<kwiver::vital::image_container_sptr> depths_out;
  std::vector<kwiver::vital::image_container_sptr> weights_out;

  for (std::map<kwiver::vital::frame_id_t, std::string>::iterator itr = depths->begin(); itr != depths->end(); itr++)
  {
    auto camitr = cameras.find(itr->first);
    if (camitr == cameras.end())
      continue;
    kwiver::vital::camera_perspective_sptr cam = std::dynamic_pointer_cast<kwiver::vital::camera_perspective>(camitr->second);
    int i0, ni, j0, nj;
    kwiver::vital::image_container_sptr depth, weight;
    load_depth_map(itr->second, i0, ni, j0, nj, depth, weight);
    depths_out.push_back(depth);
    weights_out.push_back(weight);
    cameras_out.push_back(crop_camera(cam, i0, ni, j0, nj));
  }

  double minptd[3];
  roi->GetXMin(minptd);
  kwiver::vital::vector_3d minpt(minptd);

  double maxptd[3];
  roi->GetXMax(maxptd);
  kwiver::vital::vector_3d maxpt(maxptd);

  kwiver::vital::image_container_sptr volume;
  kwiver::vital::vector_3d spacing;
  d->fuse_algo->integrate(minpt, maxpt, depths_out, weights_out, cameras_out, volume, spacing);

  vtkSmartPointer<vtkStructuredGrid> vtk_volume = volume_to_vtk(volume, minpt, spacing);

  this->updateFusion(vtk_volume);
}


