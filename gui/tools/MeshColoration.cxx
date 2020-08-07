/*ckwg +29
 * Copyright 2016 by Kitware, SAS; Copyright 2017-2018 by Kitware, Inc.
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

#include "MeshColoration.h"

#include <kwiversys/SystemTools.hxx>
#include "vtkMaptkCamera.h"

// VTK includes
#include "vtkCellData.h"
#include "vtkDoubleArray.h"
#include "vtkFloatArray.h"
#include "vtkImageData.h"
#include "vtkVector.h"
#include "vtkNew.h"
#include "vtkPointData.h"
#include "vtkPointDataToCellData.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderer.h"
#include "vtkRendererCollection.h"
#include "vtkRenderWindow.h"
#include "vtkSequencePass.h"
#include "vtkSmartPointer.h"
#include "vtkWindowToImageFilter.h"
#include "vtkXMLImageDataWriter.h"
#include "vtkXMLPolyDataWriter.h"


// Other includes
#include <algorithm>
#include <numeric>
#include <sstream>
#include <iomanip>

typedef kwiversys::SystemTools  ST;

namespace
{
static char const* const BLOCK_VR = "video_reader";
static char const* const BLOCK_MR = "mask_reader";

//----------------------------------------------------------------------------
/// Compute median of a vector
template <typename T>
static void ComputeMedian(std::vector<T> vector, double& median)
{
  std::sort(vector.begin(), vector.end());
  size_t middleIndex = vector.size() / 2;
  if (vector.size() % 2 == 0)
    {
    median = (vector[middleIndex] + vector[middleIndex - 1]) / 2;
    }
  else
    {
    median = vector[middleIndex];
    }
}

  static kwiver::vital::logger_handle_t main_logger( kwiver::vital::get_logger( "mesh_coloration" ) );

} // end anonymous namspace


MeshColoration::MeshColoration()
{
  this->Input = nullptr;
  this->Output = nullptr;
  this->Sampling = 1;
  this->Frame = -1;
  this->AverageColor = true;
  this->Error = false;
  this->OcclusionThreshold = 0.0;
  this->RemoveOccluded = true;
  this->RemoveMasked = true;
}

MeshColoration::MeshColoration(kwiver::vital::config_block_sptr& videoConfig,
                               std::string const& videoPath,
                               kwiver::vital::config_block_sptr& maskConfig,
                               std::string const& maskPath,
                               kwiver::vital::camera_map_sptr& cameras)
  : MeshColoration()
{
  this->videoPath = videoPath;
  kwiver::vital::algo::video_input::set_nested_algo_configuration(
    BLOCK_VR, videoConfig, this->videoReader);
  this->maskPath = maskPath;
  auto const hasMask = !this->maskPath.empty();
  if (hasMask && ! kwiver::vital::algo::video_input::check_nested_algo_configuration(
        BLOCK_MR, maskConfig))
  {
    LOG_ERROR(main_logger,
      "An error was found in the mask reader configuration.");
    return;
  }
  if (hasMask)
  {
    kwiver::vital::algo::video_input::set_nested_algo_configuration(
      BLOCK_MR, maskConfig, this->maskReader);
  }
  this->cameras = cameras;
}

MeshColoration::~MeshColoration()
{
  this->DataList.clear();
}

void MeshColoration::SetInput(vtkSmartPointer<vtkPolyData> mesh)
{
  this->Input = mesh;
}

vtkSmartPointer<vtkPolyData> MeshColoration::GetInput()
{
  return this->Input;
}

void MeshColoration::SetOutput(vtkSmartPointer<vtkPolyData> mesh)
{
  this->Output = mesh;
}

vtkSmartPointer<vtkPolyData> MeshColoration::GetOutput()
{
  return this->Output;
}



void MeshColoration::SetFrameSampling(int sample)
{
  if (sample < 1)
  {
    return;
  }
  this->Sampling = sample;
}


void MeshColoration::run()
{
  LOG_INFO(main_logger, "Initialize camera and image list: frame " << this->Frame);
  initializeDataList(this->Frame);
  int numFrames = static_cast<int>(this->DataList.size());

  if (this->Input == 0 || numFrames == 0 )
  {
    if (this->Input == 0)
    {
      LOG_ERROR(main_logger, "Error when input has been set");
    }
    else
    {
      LOG_INFO(main_logger, "No camera for this frame");
    }
    LOG_INFO(main_logger, "Done: frame " << this->Frame);
    emit resultReady(nullptr);
    return;
  }

  vtkPoints* meshPointList = this->Input->GetPoints();
  if (meshPointList == 0)
  {
    LOG_ERROR(main_logger, "invalid mesh points");
    LOG_INFO(main_logger, "Done: frame " << this->Frame);
    emit resultReady(nullptr);
    return;
  }
  vtkIdType nbMeshPoint = meshPointList->GetNumberOfPoints();

  // per frame colors
  std::vector<vtkSmartPointer<vtkUnsignedCharArray>> perFrameColor;
  // average colors
  vtkNew<vtkUnsignedCharArray> meanValues;
  vtkNew<vtkUnsignedCharArray> medianValues;
  vtkNew<vtkIntArray> projectedDMValue;
  // Store each rgb value for each depth map
  std::vector<double> list0;
  std::vector<double> list1;
  std::vector<double> list2;
  struct DepthBuffer
  {
    DepthBuffer()
    {
      Range[0] = Range[1] = 0;
    }
    vtkSmartPointer<vtkFloatArray> Buffer;
    double Range[2];

  };
  std::vector<DepthBuffer> depthBuffer(numFrames);
  if (this->RemoveOccluded)
  {
    LOG_INFO(main_logger, "Creating depth buffers: " << numFrames << " ...");
    auto renWin = CreateDepthBufferPipeline();

    int i = 0;
    for (auto it = depthBuffer.begin(); it != depthBuffer.end(); ++it)
    {
      kwiver::vital::camera_perspective_sptr camera = this->DataList[i].Camera_ptr;
      kwiver::vital::image_of<uint8_t> const& colorImage = this->DataList[i].Image;
      int width = colorImage.width();
      int height = colorImage.height();
      DepthBuffer db;
      db.Buffer = RenderDepthBuffer(renWin, camera, width, height, db.Range);
      *it = db;
      ++i;
    }
  }
  if (this->AverageColor)
  {
    // Contains rgb values
    meanValues->SetNumberOfComponents(3);
    meanValues->SetNumberOfTuples(nbMeshPoint);
    meanValues->FillComponent(0, 0);
    meanValues->FillComponent(1, 0);
    meanValues->FillComponent(2, 0);
    meanValues->SetName("MeanColoration");

    medianValues->SetNumberOfComponents(3);
    medianValues->SetNumberOfTuples(nbMeshPoint);
    medianValues->FillComponent(0, 0);
    medianValues->FillComponent(1, 0);
    medianValues->FillComponent(2, 0);
    medianValues->SetName("MedianColoration");

    projectedDMValue->SetNumberOfComponents(1);
    projectedDMValue->SetNumberOfTuples(nbMeshPoint);
    projectedDMValue->FillComponent(0, 0);
    projectedDMValue->SetName("NbProjectedDepthMap");
  }
  else
  {
    perFrameColor.resize(numFrames);
    vtkNew<vtkIntArray> cameraIndex;
    cameraIndex->SetNumberOfComponents(1);
    cameraIndex->SetNumberOfTuples(numFrames);
    cameraIndex->SetName("camera_index");
    this->Output->GetFieldData()->AddArray(cameraIndex);
    int i = 0;
    for (auto it = perFrameColor.begin(); it != perFrameColor.end(); ++it)
    {
      (*it) = vtkSmartPointer<vtkUnsignedCharArray>::New();
      (*it)->SetNumberOfComponents(4); // RGBA, we use A=0 for invalid pixels and A=255 otherwise
      (*it)->SetNumberOfTuples(nbMeshPoint);
      unsigned char* p = (*it)->GetPointer(0);
      std::fill(p, p + nbMeshPoint*4, 0);
      std::ostringstream ostr;
      kwiver::vital::frame_id_t frame = this->DataList[i].Frame;
      cameraIndex->SetValue(i, frame);
      ostr << "frame_" << std::setfill('0') << std::setw(4) << frame;
      (*it)->SetName(ostr.str().c_str());
      this->Output->GetPointData()->AddArray(*it);
      ++i;
    }
  }
  for (vtkIdType id = 0; id < nbMeshPoint; id++)
  {
    if (id % 100000 == 0)
    {
      LOG_INFO(main_logger, "Color " << id << " of " << nbMeshPoint << " points");
    }
    if (this->AverageColor)
    {
      list0.reserve(numFrames);
      list1.reserve(numFrames);
      list2.reserve(numFrames);
    }

    // Get mesh position from id
    kwiver::vital::vector_3d position;
    meshPointList->GetPoint(id, position.data());
    kwiver::vital::vector_3d pointNormal;
    Input->GetPointData()->GetArray("Normals")->GetTuple(id, pointNormal.data());

    for (int idData = 0; idData < numFrames; idData++)
    {
      kwiver::vital::camera_perspective_sptr camera = this->DataList[idData].Camera_ptr;
      // Check if the 3D point is in front of the camera
      double depth = camera->depth(position);
      if (depth <= 0.0)
      {
        continue;
      }

      // test that we are viewing the front side of the mesh
      kwiver::vital::vector_3d cameraPointVec = position - camera->center();
      if (cameraPointVec.dot(pointNormal)>0.0)
      {
        continue;
      }

      // project 3D point to pixel coordinates
      auto pixelPosition = camera->project(position);
      kwiver::vital::image_of<uint8_t> const& colorImage = this->DataList[idData].Image;
      int width = colorImage.width();
      int height = colorImage.height();

      if (pixelPosition[0] < 0.0 ||
          pixelPosition[1] < 0.0 ||
          pixelPosition[0] >= width ||
          pixelPosition[1] >= height)
      {
        continue;
      }
      bool hasMask = true;
      kwiver::vital::image_of<uint8_t> const& maskImage =
        this->DataList[idData].MaskImage;
      if (pixelPosition[0] < 0.0 ||
          pixelPosition[1] < 0.0 ||
          pixelPosition[0] >= maskImage.width() ||
          pixelPosition[1] >= maskImage.height())
      {
        hasMask = false;
      }
      try
      {
        int x = static_cast<int>(pixelPosition[0]);
        int y = static_cast<int>(pixelPosition[1]);
        kwiver::vital::rgb_color rgb = colorImage.at(x, y);
        bool showPoint = true;
        if (hasMask)
        {
          showPoint = (maskImage.at(x, y).r > 0);
        }

        float depthBufferValue = 0;
        if (this->RemoveOccluded)
        {
          double* range = depthBuffer[idData].Range;
          float depthBufferValueNorm =
            depthBuffer[idData].Buffer->GetValue(x + width * (height - y - 1));
          depthBufferValue = range[0] + (range[1] - range[0]) * depthBufferValueNorm;
        }
        if ((! this->RemoveOccluded ||
             depthBufferValue + this->OcclusionThreshold > depth) &&
            (! this->RemoveMasked || showPoint))
        {
          if (this->AverageColor)
          {
            list0.push_back(rgb.r);
            list1.push_back(rgb.g);
            list2.push_back(rgb.b);
          }
          else
          {
            unsigned char rgba[] = {rgb.r, rgb.g, rgb.b, 255};
            perFrameColor[idData]->SetTypedTuple(id, rgba);
          }
        }
      }
      catch (std::out_of_range)
      {
        continue;
      }
    }

    if (this->AverageColor)
    {
      // If we get elements
      if (list0.size() != 0)
      {
        double sum0 = std::accumulate(list0.begin(), list0.end(), 0);
        double sum1 = std::accumulate(list1.begin(), list1.end(), 0);
        double sum2 = std::accumulate(list2.begin(), list2.end(), 0);
        double nbVal = (double)list0.size();
        meanValues->SetTuple3(id, sum0 / (double)nbVal, sum1 / (double)nbVal, sum2 / (double)nbVal);
        double median0, median1, median2;
        ComputeMedian<double>(list0, median0);
        ComputeMedian<double>(list1, median1);
        ComputeMedian<double>(list2, median2);
        medianValues->SetTuple3(id, median0, median1, median2);
        projectedDMValue->SetTuple1(id, list0.size());
      }

      list0.clear();
      list1.clear();
      list2.clear();
    }
  }
  if (this->AverageColor)
  {
    this->Input->GetPointData()->AddArray(meanValues);
    this->Input->GetPointData()->AddArray(medianValues);
    this->Input->GetPointData()->AddArray(projectedDMValue);
  }
  LOG_INFO(main_logger, "Done: frame " << this->Frame);
  emit resultReady(this);
}

void MeshColoration::pushData(
  kwiver::vital::camera_map::map_camera_t::value_type cam_itr,
  kwiver::vital::timestamp& ts, bool hasMask)
{
  auto cam_ptr =
    std::dynamic_pointer_cast<kwiver::vital::camera_perspective>(cam_itr.second);
  if (cam_ptr && this->videoReader->seek_frame(ts, cam_itr.first) &&
      (! hasMask || this->maskReader->seek_frame(ts, cam_itr.first)))
  {
    try
    {
      kwiver::vital::image_container_sptr
        image(this->videoReader->frame_image());
      if (hasMask)
      {
        kwiver::vital::image_container_sptr
          maskImage(this->maskReader->frame_image());
        this->DataList.push_back(ColorationData(
                                   image, maskImage, cam_ptr, cam_itr.first));
      }
      else
      {
        kwiver::vital::image_container_sptr maskImage;
        this->DataList.push_back(ColorationData(
                                   image, maskImage, cam_ptr, cam_itr.first));
      }
    }
    catch (kwiver::vital::image_type_mismatch_exception)
    {
    }
  }
}


void MeshColoration::initializeDataList(int frameId)
{
  this->videoReader->open(this->videoPath);
  kwiver::vital::timestamp ts;
  auto cam_map = this->cameras->cameras();
  bool hasMask = true;
  if (this->maskPath.empty())
  {
    hasMask = false;
  }
  else
  {
    try
    {
      this->maskReader->open(this->maskPath);
    }
    catch(std::exception& e)
    {
      hasMask = false;
      LOG_ERROR(main_logger, "Cannot open mask file: " << this->maskPath);
    }
  }
  //Take a subset of images
  if (frameId < 0)
  {
    unsigned int counter = 0;
    for (auto const& cam_itr : cam_map)
    {
      if ((counter++) % this->Sampling != 0)
      {
        continue;
      }
      pushData(cam_itr, ts, hasMask);
    }
  }
  //Take the current image
  else
  {
    auto cam_itr = cam_map.find(frameId);
    if (cam_itr != cam_map.end())
    {
      pushData(*cam_itr, ts, hasMask);
    }
  }
  this->videoReader->close();
  if (hasMask)
  {
    this->maskReader->close();
  }
}


vtkSmartPointer<vtkRenderWindow> MeshColoration::CreateDepthBufferPipeline()
{
  vtkNew<vtkRenderer> ren;
  auto renWin = vtkSmartPointer<vtkRenderWindow>::New();
  renWin->OffScreenRenderingOn();
  renWin->SetMultiSamples(0);
  renWin->AddRenderer(ren);
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputDataObject(this->Input);
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  ren->AddActor(actor);
  return renWin;
}


vtkSmartPointer<vtkFloatArray> MeshColoration::RenderDepthBuffer(
  vtkSmartPointer<vtkRenderWindow> renWin,
  kwiver::vital::camera_perspective_sptr camera_ptr,
  int width, int height, double depthRange[2])
{
  renWin->SetSize(width, height);
  double* bounds = this->Input->GetBounds();
  double bb[8][3] = {{bounds[0], bounds[2], bounds[4]},
                     {bounds[1], bounds[2], bounds[4]},
                     {bounds[0], bounds[3], bounds[4]},
                     {bounds[1], bounds[3], bounds[4]},

                     {bounds[0], bounds[2], bounds[5]},
                     {bounds[1], bounds[2], bounds[5]},
                     {bounds[0], bounds[3], bounds[5]},
                     {bounds[1], bounds[3], bounds[5]}};
  depthRange[0] = std::numeric_limits<double>::max();
  depthRange[1] = std::numeric_limits<double>::lowest();
  for (int i = 0; i < 8; ++i)
  {
    double depth = camera_ptr->depth(kwiver::vital::vector_3d(bb[i][0], bb[i][1], bb[i][2]));
    if (depth < depthRange[0])
    {
      depthRange[0] = depth;
    }
    if (depth > depthRange[1])
    {
      depthRange[1] = depth;
    }
  }
  // we only render points in front of the camera
  if (depthRange[0] < 0)
  {
    depthRange[0] = 0;
  }
  vtkNew<vtkMaptkCamera> cam;
  int imageDimensions[2] = {width, height};
  cam->SetCamera(camera_ptr);
  cam->SetImageDimensions(imageDimensions);
  cam->Update();
  cam->SetClippingRange(depthRange[0], depthRange[1]);
  vtkRenderer* ren = renWin->GetRenderers()->GetFirstRenderer();
  vtkCamera* camera = ren->GetActiveCamera();
  camera->ShallowCopy(cam);
  renWin->Render();

  vtkNew<vtkWindowToImageFilter> filter;
  filter->SetInput(renWin);
  filter->SetScale(1);
  filter->SetInputBufferTypeToZBuffer();
  filter->Update();
  vtkSmartPointer<vtkFloatArray> zBuffer =
    vtkFloatArray::SafeDownCast(filter->GetOutput()->GetPointData()->GetArray(0));
  return zBuffer;
}
