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
#include "vtkVector.h"
#include "vtkNew.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"
#include "vtkSmartPointer.h"
#include <vtksys/SystemTools.hxx>

// Project includes
#include "ReconstructionData.h"

// Other includes
#include <algorithm>
#include <numeric>
#include <sstream>


namespace
{

//----------------------------------------------------------------------------
// Description
// Split string by separated char
static void SplitString(const std::string &s, char delim,
                        std::vector<std::string> &elems)
{
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim))
    {
    elems.push_back(item);
    }
}

//----------------------------------------------------------------------------
// Description
// Get the filename path (copy from vtk in order to allow debug)
static std::string GetFilenamePath(const std::string& filename)
{
  std::string fn = filename;
  vtksys::SystemTools::ConvertToUnixSlashes(fn);

  std::string::size_type slash_pos = fn.rfind("/");
  if (slash_pos != std::string::npos)
  {
    std::string  ret = fn.substr(0, slash_pos);
    if (ret.size() == 2 && ret[1] == ':')
    {
      return ret + '/';
    }
    if (ret.empty())
    {
      return "/";
    }
    return ret;
  }
  else
  {
    return "";
  }
}

//----------------------------------------------------------------------------
// Description
// Read global path and extract all contained path
static std::vector<std::string> ExtractAllFilePath(const char* globalPath)
{
  std::vector<std::string> pathList;

  // Open file which contains the list of all file
  std::ifstream container(globalPath);
  if (!container.is_open())
  {
    std::cerr << "Unable to open : " << globalPath << std::endl;
    return pathList;
  }

  // Extract path of globalPath from globalPath
  //std::string pwd_str = globalPath;
  std::string directoryPath = GetFilenamePath(std::string(globalPath));
  // Get current working directory
  if (directoryPath == "")
  {
    directoryPath = vtksys::SystemTools::GetCurrentWorkingDirectory();
  }

  std::string path;
  while (!container.eof())
  {
    std::getline(container, path);
    // only get the file name, not the whole path
    std::vector <std::string> elems;
    SplitString(path, ' ', elems);

    // check if there are an empty line
    if (elems.size() == 0)
    {
      continue;
    }

    // Create the real data path to access depth map file
    pathList.push_back(directoryPath + "/" + elems[elems.size() - 1]);
  }

  return pathList;
}

//----------------------------------------------------------------------------
// Description
// Read global path and extract all contained path
static std::vector<std::string> ExtractAllKRTDFilePath(const char* globalPath, const char* framelist)
{
  std::vector<std::string> pathList;

  // Open file which contains the list of all file
  std::ifstream container(framelist);
  if (!container.is_open())
  {
    std::cerr << "Unable to open : " << framelist << std::endl;
    return pathList;
  }

  std::string path;
  while (!container.eof())
  {
    std::getline(container, path);
    // only get the file name, not the whole path
    std::vector <std::string> elems;
    SplitString(path, ' ', elems);
    // check if there are an empty line
    if( elems.empty() )
    {
      continue;
    }
    std::vector <std::string> filename;
    SplitString(elems[elems.size() - 1], '/', filename);

    // Create the real data path to access depth map file
    std::vector <std::string> elemsWithoutExtension;
    SplitString(filename[filename.size() - 1], '.', elemsWithoutExtension);
    pathList.push_back(std::string(globalPath) + "/" + elemsWithoutExtension[0] + ".krtd");
  }

  return pathList;
}

//----------------------------------------------------------------------------
// Description
// Compute median of a vector
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
  this->OutputMesh = 0;
  this->Sampling = 1;
}

MeshColoration::MeshColoration(vtkPolyData* mesh, std::string frameList, std::string krtdFolder)
  :MeshColoration()
{
  //this->OutputMesh = vtkPolyData::New();
  //this->OutputMesh->DeepCopy(mesh);

  this->frameList = ExtractAllFilePath(frameList.c_str());
  this->krtdFolder = ExtractAllKRTDFilePath(krtdFolder.c_str(), frameList.c_str());
  if (this->krtdFolder.size() < frameList.size())
    {
    std::cerr << "Error, not enough krtd file for each vti file" << std::endl;
    return;
    }

}

MeshColoration::~MeshColoration()
{
  if (this->OutputMesh != 0)
    this->OutputMesh->Delete();
  for (int i = 0; i < this->DataList.size(); i++)
    {
    delete this->DataList[i];
    }
  this->DataList.clear();
}

void MeshColoration::SetInput(vtkPolyData* mesh)
{
  if (this->OutputMesh != 0)
    this->OutputMesh->Delete();
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

bool MeshColoration::ProcessColoration(std::string currentVtiPath)
{
  initializeDataList(currentVtiPath);

  int nbDepthMap = (int)this->DataList.size();

  if (this->OutputMesh == 0 || nbDepthMap == 0 /*|| this->Sampling >= nbDepthMap*/)
    {
    std::cerr << "Error when input has been set or during reading vti/krtd file path" << std::endl;
    return false;
    }

  vtkPoints* meshPointList = this->OutputMesh->GetPoints();
  if (meshPointList == 0)
  {
    std::cerr << "invalid mesh points" <<std::endl;
    return false;
  }
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
    double pointNormald[3];
    OutputMesh->GetPointData()->GetArray("Normals")->GetTuple(id, pointNormald);
    vtkVector3d pointNormal(pointNormald);



    for (int idData = 0; idData < nbDepthMap; idData++)
      {
      ReconstructionData* data = this->DataList[idData];
      vtkVector3d cameraCenter = data->GetCameraCenter();
      // Check if the 3D point is in front of the camera
      vtkVector3d cameraPointVec = vtkVector3d( position[0] - cameraCenter(0),
                                                position[1] - cameraCenter(1),
                                                position[2] - cameraCenter(2) );
      if (cameraPointVec.Dot(pointNormal)>0.0)
        continue;
      // project 3D point to pixel coordinates
      int pixelPosition[2];
      data->TransformWorldToDepthMapPosition(position, pixelPosition);
      // Test if pixel is inside depth map
      if (pixelPosition[0] < 0 || pixelPosition[0] >= depthMapDimensions[0] ||
          pixelPosition[1] < 0 || pixelPosition[1] >= depthMapDimensions[1])
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

  this->OutputMesh->GetPointData()->AddArray(meanValues);
  this->OutputMesh->GetPointData()->AddArray(medianValues);
  this->OutputMesh->GetPointData()->AddArray(projectedDMValue);

  return true;
}

void MeshColoration::initializeDataList(std::string currentVtiPath)
{
    int nbDepthMap = (int)frameList.size();

  //Take a subset of depthmap
  if (currentVtiPath == "")
  {
    for (int id = 0; id < nbDepthMap; id++)
    {
      if (id%Sampling == 0)
      {
        ReconstructionData* data = new ReconstructionData(frameList[id], krtdFolder[id]);
        this->DataList.push_back(data);
      }
    }
  }
  //Take the current depthmap
  else
  {
    std::string currentDepthmapName = currentVtiPath.substr(currentVtiPath.find_last_of("/"));
    for (int id = 0; id < nbDepthMap; id++)
    {
      std::string depthmapName = frameList[id].substr(frameList[id].find_last_of("/"));
      if (currentDepthmapName == depthmapName)
      {
        ReconstructionData* data = new ReconstructionData(frameList[id], krtdFolder[id]);
        this->DataList.push_back(data);
        break;
      }
    }
  }

}
