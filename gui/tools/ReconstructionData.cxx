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

#include "Helper.h"
#include "ReconstructionData.h"

#include <sstream>
#include <cmath>

// VTK includes
#include "vtkDoubleArray.h"
#include "vtkImageData.h"
#include "vtkVector.h"
#include "vtkMatrix3x3.h"
#include "vtkMatrix4x4.h"
#include "vtkPointData.h"
#include "vtkSmartPointer.h"
#include "vtkTransform.h"
#include "vtkUnsignedCharArray.h"
#include "vtkPNGReader.h"


ReconstructionData::ReconstructionData()
{
  this->DepthMap = 0;
  this->MatrixK = 0;
  this->MatrixRT = 0;
}

ReconstructionData::ReconstructionData(std::string depthPath,
                                       std::string matrixPath)
{
  // Read DEPTH MAP an fill this->DepthMap
  this->DepthMap = vtkImageData::New();
  ReconstructionData::ReadDepthMap(depthPath, this->DepthMap);


  this->TransformWorldToCamera = vtkTransform::New();
  this->TransformCameraToDepthMap = vtkTransform::New();

  // Read KRTD FILE
  vtkNew<vtkMatrix3x3> K;
  vtkNew<vtkMatrix4x4> RT;
  this->MatrixRT = vtkMatrix4x4::New();
  this->MatrixK = vtkMatrix3x3::New();
  this->Matrix4K = vtkMatrix4x4::New();
  help::ReadKrtdFile(matrixPath, K.Get(), RT.Get());

  // Set matrix K to  create matrix4x4 for K
  this->SetMatrixK(K.Get());
  this->SetMatrixRT(RT.Get());
}

ReconstructionData::~ReconstructionData()
{
  if (this->DepthMap)
    this->DepthMap->Delete();
  if (this->MatrixK)
    this->MatrixK->Delete();
  if (this->MatrixRT)
    this->MatrixRT->Delete();
  if (this->Matrix4K)
    this->Matrix4K->Delete();
}

void ReconstructionData::GetColorValue(int* pixelPosition, double rgb[3])
{
  vtkUnsignedCharArray* color =
    vtkUnsignedCharArray::SafeDownCast(this->DepthMap->GetPointData()->GetArray("PNGImage"));

  if (color == 0)
    {
    std::cerr << "Error, no 'Color' array exists" << std::endl;
    return;
    }

  int* depthDims = this->DepthMap->GetDimensions();

  int pix[3];
  pix[0] = pixelPosition[0];
  pix[1] = depthDims[1] - 1 - pixelPosition[1];
  pix[2] = 0;

  int id = this->DepthMap->ComputePointId(pix);
  double* temp = color->GetTuple3(id);
  for (size_t i = 0; i < 3; i++)
  {
    rgb[i] = temp[i];
  }
}

vtkImageData* ReconstructionData::GetDepthMap()
{
  return this->DepthMap;
}

vtkMatrix3x3* ReconstructionData::Get3MatrixK()
{
  return this->MatrixK;
}

vtkMatrix4x4* ReconstructionData::Get4MatrixK()
{
  return this->Matrix4K;
}

vtkVector3d ReconstructionData::GetCameraCenter()
{
  // -R.transpose() * T
  return vtkVector3d(-(this->MatrixRT->GetElement(0,0)*this->MatrixRT->GetElement(0,3)
                       + this->MatrixRT->GetElement(1,0)*this->MatrixRT->GetElement(1,3)
                       + this->MatrixRT->GetElement(2,0)*this->MatrixRT->GetElement(2,3)),
                     -(this->MatrixRT->GetElement(0,1)*this->MatrixRT->GetElement(0,3)
                       + this->MatrixRT->GetElement(1,1)*this->MatrixRT->GetElement(1,3)
                       + this->MatrixRT->GetElement(2,1)*this->MatrixRT->GetElement(2,3)),
                     -(this->MatrixRT->GetElement(0,2)*this->MatrixRT->GetElement(0,3)
                       + this->MatrixRT->GetElement(1,2)*this->MatrixRT->GetElement(1,3)
                       + this->MatrixRT->GetElement(2,2)*this->MatrixRT->GetElement(2,3)));
}

vtkMatrix4x4* ReconstructionData::GetMatrixTR()
{
  return this->MatrixRT;
}

void ReconstructionData::ApplyDepthThresholdFilter(double thresholdBestCost)
{
  if (this->DepthMap == 0)
    return;

  vtkDoubleArray* depths =
    vtkDoubleArray::SafeDownCast(this->DepthMap->GetPointData()->GetArray("Depths"));
  vtkDoubleArray* bestCost =
    vtkDoubleArray::SafeDownCast(this->DepthMap->GetPointData()->GetArray("Best Cost Values"));

  if (depths == 0)
    {
    std::cerr << "Error during threshold, depths is empty" << std::endl;
    return;
    }

  int nbTuples = depths->GetNumberOfTuples();

  if (bestCost->GetNumberOfTuples() != nbTuples)
    return;

  for (int i = 0; i < nbTuples; i++)
    {
    double v_bestCost = bestCost->GetTuple1(i);
    if (v_bestCost > thresholdBestCost)
      {
      depths->SetTuple1(i, -1);
      }
    }
}

void ReconstructionData::TransformWorldToDepthMapPosition(const double* worldCoordinate,
                                                          int pixelCoordinate[2])
{
  double cameraCoordinate[3];
  this->TransformWorldToCamera->TransformPoint(worldCoordinate, cameraCoordinate);
  double depthMapCoordinate[3];
  this->TransformCameraToDepthMap->TransformVector(cameraCoordinate, depthMapCoordinate);

  depthMapCoordinate[0] = depthMapCoordinate[0] / depthMapCoordinate[2];
  depthMapCoordinate[1] = depthMapCoordinate[1] / depthMapCoordinate[2];

  pixelCoordinate[0] = std::round(depthMapCoordinate[0]);
  pixelCoordinate[1] = std::round(depthMapCoordinate[1]);
}

void ReconstructionData::SetDepthMap(vtkImageData* data)
{
  if (this->DepthMap != 0)
    this->DepthMap->Delete();
  this->DepthMap = data;
  this->DepthMap->Register(0);
}

void ReconstructionData::SetMatrixK(vtkMatrix3x3* matrix)
{
  if (this->MatrixK != 0)
    this->MatrixK->Delete();
  this->MatrixK = matrix;
  this->MatrixK->Register(0);

  if (this->Matrix4K != 0)
    this->Matrix4K->Delete();
  this->Matrix4K = vtkMatrix4x4::New();
  this->Matrix4K->Identity();
  for (int i = 0; i < 3; i++)
    {
    for (int j = 0; j < 3; j++)
      {
      this->Matrix4K->SetElement(i, j, this->MatrixK->GetElement(i, j));
      }
    }

  this->TransformCameraToDepthMap->SetMatrix(this->Matrix4K);
}

void ReconstructionData::SetMatrixRT(vtkMatrix4x4* matrix)
{
  if (this->MatrixRT != 0)
    this->MatrixRT->Delete();
  this->MatrixRT = matrix;
  this->MatrixRT->Register(0);
  this->TransformWorldToCamera->SetMatrix(this->MatrixRT);
}

void ReconstructionData::ReadDepthMap(std::string path, vtkImageData* out)
{
  vtkSmartPointer<vtkPNGReader> depthMapReader = vtkSmartPointer<vtkPNGReader>::New();
  depthMapReader->SetFileName(path.c_str());
  depthMapReader->Update();
  out->ShallowCopy(depthMapReader->GetOutput());
}
