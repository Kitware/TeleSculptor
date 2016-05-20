// Copyright(c) 2016, Kitware SAS
// www.kitware.fr
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met :
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation and
// / or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "MeshColoration.h"

// VTK includes
#include "vtkDoubleArray.h"
#include "vtkImageData.h"
#include "vtkNew.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"
#include "vtkSmartPointer.h"

// Project includes
#include "Helper.h"
#include "ReconstructionData.h"

// Other includes
#include <numeric>

MeshColoration::MeshColoration()
{
  this->OutputMesh = nullptr;
  this->Sampling = 1;
}

MeshColoration::MeshColoration(vtkPolyData* mesh, std::string vti, std::string krtd)
  :MeshColoration()
{
  //this->OutputMesh = vtkPolyData::New();
  //this->OutputMesh->DeepCopy(mesh);

  std::vector<std::string> vtiList = help::ExtractAllFilePath(vti.c_str());
  std::vector<std::string> krtdList = help::ExtractAllFilePath(krtd.c_str());
  if (krtdList.size() < vtiList.size())
    {
    std::cerr << "Error, not enough krtd file for each vti file" << std::endl;
    return;
    }

  int nbDepthMap = (int)vtiList.size();

  for (int id = 0; id < nbDepthMap; id++)
    {
    ReconstructionData* data = new ReconstructionData(vtiList[id], krtdList[id]);
    this->DataList.push_back(data);
    }
}

MeshColoration::~MeshColoration()
{
  if (this->OutputMesh != nullptr)
    this->OutputMesh->Delete();
  for (int i = 0; i < this->DataList.size(); i++)
    {
    delete this->DataList[i];
    }
  this->DataList.clear();
}

void MeshColoration::SetInput(vtkPolyData* mesh)
{
  if (this->OutputMesh != nullptr)
    this->OutputMesh->Delete();
  //this->OutputMesh = vtkPolyData::New();
  //this->OutputMesh->DeepCopy(mesh);
  this->OutputMesh = mesh;
}

void MeshColoration::SetFrameSampling(int sample)
{
  if (sample < 1)
    return;
  this->Sampling = sample;
}

vtkPolyData* MeshColoration::GetOutput()
{
  return this->OutputMesh;
}

bool MeshColoration::ProcessColoration()
{
  int nbDepthMap = (int)this->DataList.size();

  if (this->OutputMesh == nullptr || nbDepthMap == 0 || this->Sampling >= nbDepthMap)
    {
    std::cerr << "Error when input has been set or during reading vti/krtd file path" << std::endl;
    return false;
    }

  vtkPoints* meshPointList = this->OutputMesh->GetPoints();
  vtkIdType nbMeshPoint = meshPointList->GetNumberOfPoints();
  int* depthMapDimensions = this->DataList[0]->GetDepthMap()->GetDimensions();

  // Contains rgb values
  vtkSmartPointer<vtkUnsignedCharArray> meanValues = vtkUnsignedCharArray::New();
  meanValues->SetNumberOfComponents(3);
  meanValues->SetNumberOfTuples(nbMeshPoint);
  meanValues->FillComponent(0, 0);
  meanValues->FillComponent(1, 0);
  meanValues->FillComponent(2, 0);
  meanValues->SetName("MeanColoration");

  vtkSmartPointer<vtkUnsignedCharArray> medianValues = vtkUnsignedCharArray::New();
  medianValues->SetNumberOfComponents(3);
  medianValues->SetNumberOfTuples(nbMeshPoint);
  medianValues->FillComponent(0, 0);
  medianValues->FillComponent(1, 0);
  medianValues->FillComponent(2, 0);
  medianValues->SetName("MedianColoration");

  vtkSmartPointer<vtkIntArray> projectedDMValue = vtkIntArray::New();
  projectedDMValue->SetNumberOfComponents(1);
  projectedDMValue->SetNumberOfTuples(nbMeshPoint);
  projectedDMValue->FillComponent(0, 0);
  projectedDMValue->SetName("NbProjectedDepthMap");

  // Store each rgb value for each depth map
  std::vector<double> list0;
  std::vector<double> list1;
  std::vector<double> list2;

  for (vtkIdType id = 0; id < nbMeshPoint; id++)
  {
    list0.reserve(nbDepthMap);
    list1.reserve(nbDepthMap);
    list2.reserve(nbDepthMap);

    // Get mesh position from id
    double position[3];
    meshPointList->GetPoint(id, position);

    for (int idData = 0; idData < nbDepthMap; idData++)
      {

      // Use sampling if we don't want to use all frames
      if (idData%this->Sampling != 0)
        continue;

      ReconstructionData* data = this->DataList[idData];

      // Transform to pixel index
      int pixelPosition[2];
      data->TransformWorldToDepthMapPosition(position, pixelPosition);
      // Test if pixel is inside depth map
      if (pixelPosition[0] < 0 || pixelPosition[1] < 0 ||
        pixelPosition[0] >= depthMapDimensions[0] ||
        pixelPosition[1] >= depthMapDimensions[1])
        {
        continue;
        }

      double color[3];
      data->GetColorValue(pixelPosition, color);

      list0.push_back(color[0]);
      list1.push_back(color[1]);
      list2.push_back(color[2]);
    }

    // If we get elements
    if (list0.size() != 0)
      {
      double sum0 = std::accumulate(list0.begin(), list0.end(), 0);
      double sum1 = std::accumulate(list1.begin(), list1.end(), 0);
      double sum2 = std::accumulate(list2.begin(), list2.end(), 0);
      double nbVal = (double)list0.size();
      meanValues->SetTuple3(id, sum0 / (double)nbVal, sum1 / (double)nbVal, sum2 / (double)nbVal);
      double median0, median1, median2;
      help::ComputeMedian<double>(list0, median0);
      help::ComputeMedian<double>(list1, median1);
      help::ComputeMedian<double>(list2, median2);
      medianValues->SetTuple3(id, median0, median1, median2);
      projectedDMValue->SetTuple1(id, list0.size());
      }

    list0.clear();
    list1.clear();
    list2.clear();
  }

  this->OutputMesh->GetPointData()->AddArray(meanValues);
  this->OutputMesh->GetPointData()->AddArray(medianValues);
  this->OutputMesh->GetPointData()->AddArray(projectedDMValue);

  return true;
}
