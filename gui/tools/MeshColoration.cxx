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

// VTK includes
#include "vtkDoubleArray.h"
#include "vtkImageData.h"
#include "vtkVector.h"
#include "vtkNew.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"
#include "vtkSmartPointer.h"
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
}

namespace
{

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

} // end anonymous namspace


MeshColoration::MeshColoration()
{
  this->Input = nullptr;
  this->Output = nullptr;
  this->Sampling = 1;
  this->Frame = -1;
  this->AverageColor = true;
  this->Error = false;
}

MeshColoration::MeshColoration(kwiver::vital::config_block_sptr& config,
                               std::string const& videoPath,
                               kwiver::vital::camera_map_sptr& cameras)
  : MeshColoration()
{
  this->videoPath = videoPath;
  kwiver::vital::algo::video_input::set_nested_algo_configuration(
    BLOCK_VR, config, this->videoReader);
  this->cameras = cameras;
}

MeshColoration::~MeshColoration()
{
  this->DataList.clear();
}

void MeshColoration::SetInput(vtkPolyData* mesh)
{
  this->Input = mesh;
}

vtkPolyData* MeshColoration::GetInput()
{
  return this->Input;
}

void MeshColoration::SetOutput(vtkPolyData* mesh)
{
  this->Output = mesh;
}

vtkPolyData* MeshColoration::GetOutput()
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
  std::cout << "MeshColoration::begin" << std::endl;
  initializeDataList(this->Frame);

  int numFrames = static_cast<int>(this->DataList.size());

  if (this->Input == 0 || numFrames == 0 )
  {
    std::cerr << "Error when input has been set or during reading vti/krtd file path" << std::endl;
    emit resultReady(nullptr);

  }

  vtkPoints* meshPointList = this->Input->GetPoints();
  if (meshPointList == 0)
  {
    std::cerr << "invalid mesh points" <<std::endl;
    emit resultReady(nullptr);
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
    std::cout << "numFrames: " << numFrames << " numMeshPoints: " << nbMeshPoint << std::endl;
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
      if (camera->depth(position) <= 0.0)
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
      if (pixelPosition[0] < 0.0 ||
          pixelPosition[1] < 0.0 ||
          pixelPosition[0] >= colorImage.width() ||
          pixelPosition[1] >= colorImage.height())
      {
        continue;
      }
      try
      {
        unsigned i = static_cast<unsigned>(pixelPosition[0]);
        unsigned j = static_cast<unsigned>(pixelPosition[1]);
        kwiver::vital::rgb_color rgb = colorImage.at(i, j);
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
  emit resultReady(this);
  std::cout << "MeshColoration::end" << std::endl;
}

void MeshColoration::initializeDataList(int frameId)
{
  this->videoReader->open(this->videoPath);
  kwiver::vital::timestamp ts;
  auto cam_map = this->cameras->cameras();

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
      auto cam_ptr =
        std::dynamic_pointer_cast<kwiver::vital::camera_perspective>(cam_itr.second);
      if (cam_ptr && this->videoReader->seek_frame(ts, cam_itr.first))
      {
        try
        {
          kwiver::vital::image_of<uint8_t>
            image(this->videoReader->frame_image()->get_image());
          this->DataList.push_back(ColorationData(image, cam_ptr, cam_itr.first));
        }
        catch (kwiver::vital::image_type_mismatch_exception)
        {
        }
      }
    }
  }
  //Take the current image
  else
  {
    auto cam_itr = cam_map.find(frameId);
    if (cam_itr != cam_map.end())
    {
      auto cam_ptr =
        std::dynamic_pointer_cast<kwiver::vital::camera_perspective>(cam_itr->second);
      if (cam_ptr && this->videoReader->seek_frame(ts, frameId))
      {
        try
        {
          kwiver::vital::image_of<uint8_t>
            image(this->videoReader->frame_image()->get_image());
          this->DataList.push_back(ColorationData(image, cam_ptr, frameId));
        }
        catch (kwiver::vital::image_type_mismatch_exception)
        {
        }
      }
    }
  }
  this->videoReader->close();
}
